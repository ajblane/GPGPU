// 
// Copyright (C) 2011-2014 Jeff Bush
// 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
// 
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the
// Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
// Boston, MA  02110-1301, USA.
// 


`include "defines.sv"

//
// L1 Instruction/Data Cache
//
// This is virtually indexed/virtually tagged and non-blocking.
// It has one cycle of latency.  During each cycle, tag memory and
// the four way memory banks are accessed in parallel.  Combinational
// logic them determines which bank the result should be pulled from.
//
// The input address is treated as follows:
//
//  [ tag ] [ set ] [ offset into line (6 bits) ]
//
// set has a configurable number of bits.  The tag is whatever bits remain.
//
// Note that the L2 cache controls what is in the L1 cache. When there are misses
// or even stores, we always inform the L2 cache, which pushes lines back into
// the L1 cache.
//

module l1_cache
	#(parameter unit_id_t UNIT_ID = 0,
	parameter CORE_ID = 0)
	(input                               clk,
	input                                reset,
	
	// Incoming request
	input                                access_i,
	input [25:0]                         request_addr,
	input [`STRAND_INDEX_WIDTH - 1:0]    strand_i,
	input                                synchronized_i,

	// Response (one cycle later)
	output [`CACHE_LINE_BITS - 1:0]      data_o,
	output                               cache_hit_o,
	output                               load_collision_o,
	
	// Cache miss completion
	output [`STRANDS_PER_CORE - 1:0]     load_complete_strands_o,

	// L2 interface
	input                                l2req_ready,
	output l2req_packet_t                l2req_packet,
	input l2rsp_packet_t                 l2rsp_packet,
	
	// Performance counter event
	output logic                         pc_event_cache_hit,
	output logic                         pc_event_cache_miss);
	
	logic[`L1_WAY_INDEX_WIDTH - 1:0] lru_way;
	logic access_latched;
	logic	synchronized_latched;
	logic[25:0] request_addr_latched;
	logic[`STRAND_INDEX_WIDTH - 1:0] strand_latched;
	logic load_collision1;
	logic[`L1_WAY_INDEX_WIDTH - 1:0] hit_way;
	logic data_in_cache;
	logic[`STRANDS_PER_CORE - 1:0] sync_load_wait;
	logic[`STRANDS_PER_CORE - 1:0] sync_load_complete;

	wire is_for_me = l2rsp_packet.unit == UNIT_ID && l2rsp_packet.core == CORE_ID;
	wire[`L1_SET_INDEX_WIDTH - 1:0] requested_set = request_addr[`L1_SET_INDEX_WIDTH - 1:0];

	wire[`L1_SET_INDEX_WIDTH - 1:0] l2_response_set = l2rsp_packet.address[`L1_SET_INDEX_WIDTH - 1:0];
	wire[`L1_TAG_WIDTH - 1:0] l2_response_tag = l2rsp_packet.address[25:`L1_SET_INDEX_WIDTH];

	wire got_load_response = l2rsp_packet.valid && is_for_me && l2rsp_packet.op == L2RSP_LOAD_ACK;

	// l2rsp_packet.update indicates if a L1 tag should be cleared for an dinvalidate
	// response
	wire invalidate_one_way = l2rsp_packet.valid && l2rsp_packet.op == L2RSP_DINVALIDATE
		&& UNIT_ID == UNIT_DCACHE && l2rsp_packet.update[CORE_ID];
	wire invalidate_all_ways = l2rsp_packet.valid && UNIT_ID == UNIT_ICACHE && l2rsp_packet.op
		== L2RSP_IINVALIDATE;
	l1_cache_tag tag_mem(
		.hit_way_o(hit_way),
		.cache_hit_o(data_in_cache),
		.update_i(got_load_response),	
		.update_way_i(l2rsp_packet.way[`L1_WAY_INDEX_WIDTH * CORE_ID+:`L1_WAY_INDEX_WIDTH]),
		.update_tag_i(l2_response_tag),
		.update_set_i(l2_response_set),
		.*);

	// Check the unit for loads to differentiate between icache and dcache.
	// We don't check the unit for store acks
	wire update_data = l2rsp_packet.valid 
		&& ((l2rsp_packet.op == L2RSP_LOAD_ACK && is_for_me) 
		|| (l2rsp_packet.op == L2RSP_STORE_ACK && l2rsp_packet.update[CORE_ID] && UNIT_ID == UNIT_DCACHE));

	logic[`CACHE_LINE_BITS - 1:0] way_read_data[0:`L1_NUM_WAYS - 1];
	genvar way;
	generate
		for (way = 0; way < `L1_NUM_WAYS; way++)
		begin : makeway
			sram_1r1w #(.DATA_WIDTH(`CACHE_LINE_BITS), .SIZE(`L1_NUM_SETS)) way_data (
				.clk(clk),
				.rd_addr(requested_set),
				.rd_data(way_read_data[way]),
				.rd_enable(access_i),
				.wr_addr(l2_response_set),
				.wr_data(l2rsp_packet.data),
				.wr_enable(update_data 
					&& l2rsp_packet.way[`L1_WAY_INDEX_WIDTH * CORE_ID+:`L1_WAY_INDEX_WIDTH] 
					== way));
		end
	endgenerate

	// We've fetched the value from all four ways in parallel.  Now
	// we know which way contains the data we care about, so select
	// that one.
	assign data_o = way_read_data[hit_way];

	// If there is a hit, move that way to the MRU.	 If there is a miss,
	// move the victim way to the MRU position so it doesn't get evicted on 
	// the next data access.
	wire[`L1_WAY_INDEX_WIDTH - 1:0] new_mru_way = data_in_cache ? hit_way : lru_way;
	wire update_mru = data_in_cache || (access_latched && !data_in_cache);
	
	cache_lru #(.NUM_SETS(`L1_NUM_SETS)) lru(
		.set_i(requested_set),
		.lru_way_o(lru_way),
		.*);

	// A load collision occurs when the L2 cache returns a specific cache line
	// in the same cycle we are about to request one. The L1 cache guarantees 
	// that it will not re-request a line when one is already L1 resident or
	// one has been requested.  The load_miss_queue handles the second part of this,
	// and the first part is automatic when the line is already loaded, but
	// there is an edge case where the pending request is neither in the load_miss_queue
	// (being cleared now), nor in the cache data (hasn't been latched yet).
	// Detect that here.
	wire load_collision2 = got_load_response
		&& l2rsp_packet.address == request_addr_latched
		&& access_latched;

	logic need_sync_rollback;

	// Note: do not mark as a load collision if we need a rollback for
	// a synchronized load command (which effectively forces an L2 read 
	// even if the data is present).
	assign load_collision_o = (load_collision1 || load_collision2)
		&& !need_sync_rollback;	

	// Note that a synchronized load always queues a load from the L2 cache,
	// even if the data is in the cache. It must do that to guarantee atomicity.
	wire queue_cache_load = (need_sync_rollback || !data_in_cache) 
		&& access_latched && !load_collision_o;

	// If we do a synchronized load and this is a cache hit, re-load
	// data into the same way that is it is already in.  Otherwise, suggest
	// the LRU way to the L2 cache.
	wire[`L1_WAY_INDEX_WIDTH - 1:0] load_way = synchronized_latched && data_in_cache ? 
		hit_way : lru_way;

	wire[`STRANDS_PER_CORE - 1:0] sync_req_oh = (access_i && synchronized_i) ? (1 << strand_i) : 0;
	wire[`STRANDS_PER_CORE - 1:0] sync_ack_oh = ((l2rsp_packet.valid && is_for_me) ? (1 << l2rsp_packet.strand) : 0)
		& sync_load_wait;

	// Synchronized accesses always take a cache miss on the first load
	assign cache_hit_o = data_in_cache && !need_sync_rollback;

	l1_load_miss_queue #(.UNIT_ID(UNIT_ID), .CORE_ID(CORE_ID)) load_miss_queue(
		.clk(clk),
		.request_i(queue_cache_load),
		.synchronized_i(synchronized_latched),
		.request_addr(request_addr_latched),
		.victim_way_i(load_way),
		.strand_i(strand_latched),
		.*);

	// Performance counter events
	always_comb
	begin
		pc_event_cache_hit = 0;
		pc_event_cache_miss = 0;
		if (access_latched && !load_collision_o)
		begin
			if (cache_hit_o)
				pc_event_cache_hit = 1;
			else if (!need_sync_rollback)
				pc_event_cache_miss = 1;
		end
	end

	always_ff @(posedge clk, posedge reset)
	begin
		if (reset)
		begin
			/*AUTORESET*/
			// Beginning of autoreset for uninitialized flops
			access_latched <= 1'h0;
			load_collision1 <= 1'h0;
			need_sync_rollback <= 1'h0;
			request_addr_latched <= 26'h0;
			strand_latched <= {(1+(`STRAND_INDEX_WIDTH-1)){1'b0}};
			sync_load_complete <= {(1+(`STRANDS_PER_CORE-1)){1'b0}};
			sync_load_wait <= {(1+(`STRANDS_PER_CORE-1)){1'b0}};
			synchronized_latched <= 1'h0;
			// End of automatics
		end
		else
		begin
			assert((sync_load_wait & sync_req_oh) == 0); // Block strand can't issue load
			assert((sync_load_wait & sync_load_complete) == 0); 
				// load complete and load wait can't be set simultaneously

			// A bit of a kludge to work around a hazard where a request
			// is made in the same cycle a load finishes of the same line.
			// It will not be in tag ram, but if a load is initiated, we'll
			// end up with the cache data in 2 ways.
			load_collision1 <= got_load_response
				&& l2rsp_packet.address == request_addr
				&& access_i;
	
			access_latched <= access_i;
			synchronized_latched <= synchronized_i;
			request_addr_latched <= request_addr;
			strand_latched <= strand_i;
			sync_load_wait <= ((sync_load_wait & ~sync_ack_oh) | (sync_req_oh & ~sync_load_complete));
			sync_load_complete <= (sync_load_complete | sync_ack_oh) & ~sync_req_oh;
			need_sync_rollback <= (sync_req_oh & ~sync_load_complete) != 0;
		end
	end
endmodule

// Local Variables:
// verilog-typedef-regexp:"_t$"
// End:

