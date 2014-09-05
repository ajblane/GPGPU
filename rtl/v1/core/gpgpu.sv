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
// Top level block for GPGPU
//

module gpgpu
	(input            clk,
	input             reset,
	output            processor_halt,

	// AXI external memory interface
	axi_interface     axi_bus,
	
	// Non-cacheable memory signals
	output            io_write_en,
	output            io_read_en,
	output[31:0]      io_address,
	output[31:0]      io_write_data,
	input [31:0]      io_read_data);

	/*AUTOWIRE*/
	// Beginning of automatic wires (for undeclared instantiated-module outputs)
	wire		l2req_ready;		// From l2_cache of l2_cache.v
	l2rsp_packet_t	l2rsp_packet;		// From l2_cache of l2_cache.v
	wire		pc_event_cond_branch_not_taken;// From core0 of core.v
	wire		pc_event_cond_branch_taken;// From core0 of core.v
	wire		pc_event_instruction_issue;// From core0 of core.v
	wire		pc_event_instruction_retire;// From core0 of core.v
	wire		pc_event_l1d_hit;	// From core0 of core.v
	wire		pc_event_l1d_miss;	// From core0 of core.v
	wire		pc_event_l1i_hit;	// From core0 of core.v
	wire		pc_event_l1i_miss;	// From core0 of core.v
	wire		pc_event_l2_hit;	// From l2_cache of l2_cache.v
	wire		pc_event_l2_miss;	// From l2_cache of l2_cache.v
	wire		pc_event_l2_wait;	// From l2_cache of l2_cache.v
	wire		pc_event_l2_writeback;	// From l2_cache of l2_cache.v
	wire		pc_event_mem_ins_issue;	// From core0 of core.v
	wire		pc_event_mispredicted_branch;// From core0 of core.v
	wire		pc_event_store;		// From l2_cache of l2_cache.v
	wire		pc_event_uncond_branch;	// From core0 of core.v
	wire		pc_event_vector_ins_issue;// From core0 of core.v
	// End of automatics
	
	l2req_packet_t l2req_packet;
	l2req_packet_t l2req_packet0;
	logic l2req_ready0;
	l2req_packet_t l2req_packet1;
	logic l2req_ready1;

	logic halt0;
	logic halt1;

	assign processor_halt = halt0 && halt1;


	/* core AUTO_TEMPLATE(
		.halt_o(halt0),
		.\(l2req_.*\)(\10[]),
		);
	*/
	core #(4'd0) core0(
		/*AUTOINST*/
			   // Interfaces
			   .l2req_packet	(l2req_packet0), // Templated
			   .l2rsp_packet	(l2rsp_packet),
			   // Outputs
			   .halt_o		(halt0),	 // Templated
			   .io_write_en		(io_write_en),
			   .io_read_en		(io_read_en),
			   .io_address		(io_address[31:0]),
			   .io_write_data	(io_write_data[31:0]),
			   .pc_event_l1d_hit	(pc_event_l1d_hit),
			   .pc_event_l1d_miss	(pc_event_l1d_miss),
			   .pc_event_l1i_hit	(pc_event_l1i_hit),
			   .pc_event_l1i_miss	(pc_event_l1i_miss),
			   .pc_event_mispredicted_branch(pc_event_mispredicted_branch),
			   .pc_event_instruction_issue(pc_event_instruction_issue),
			   .pc_event_instruction_retire(pc_event_instruction_retire),
			   .pc_event_uncond_branch(pc_event_uncond_branch),
			   .pc_event_cond_branch_taken(pc_event_cond_branch_taken),
			   .pc_event_cond_branch_not_taken(pc_event_cond_branch_not_taken),
			   .pc_event_vector_ins_issue(pc_event_vector_ins_issue),
			   .pc_event_mem_ins_issue(pc_event_mem_ins_issue),
			   // Inputs
			   .clk			(clk),
			   .reset		(reset),
			   .io_read_data	(io_read_data[31:0]),
			   .l2req_ready		(l2req_ready0));	 // Templated

	generate
		if (`NUM_CORES > 1)
		begin : next_core
			/* core AUTO_TEMPLATE(
				.halt_o(halt1),
				.io_.*(),
				.pc_event_.*(),
				.\(l2req_.*\)(\11[]),
				.halt_o(halt1),
				.io_read_data(32'd0),
				);
			*/
			core #(4'd1) core1(
				/*AUTOINST*/
					   // Interfaces
					   .l2req_packet	(l2req_packet1), // Templated
					   .l2rsp_packet	(l2rsp_packet),
					   // Outputs
					   .halt_o		(halt1),	 // Templated
					   .io_write_en		(),		 // Templated
					   .io_read_en		(),		 // Templated
					   .io_address		(),		 // Templated
					   .io_write_data	(),		 // Templated
					   .pc_event_l1d_hit	(),		 // Templated
					   .pc_event_l1d_miss	(),		 // Templated
					   .pc_event_l1i_hit	(),		 // Templated
					   .pc_event_l1i_miss	(),		 // Templated
					   .pc_event_mispredicted_branch(),	 // Templated
					   .pc_event_instruction_issue(),	 // Templated
					   .pc_event_instruction_retire(),	 // Templated
					   .pc_event_uncond_branch(),		 // Templated
					   .pc_event_cond_branch_taken(),	 // Templated
					   .pc_event_cond_branch_not_taken(),	 // Templated
					   .pc_event_vector_ins_issue(),	 // Templated
					   .pc_event_mem_ins_issue(),		 // Templated
					   // Inputs
					   .clk			(clk),
					   .reset		(reset),
					   .io_read_data	(32'd0),	 // Templated
					   .l2req_ready		(l2req_ready1));	 // Templated

			// Simple arbiter for cores
			logic select_core0 = 0;
			
			assign l2req_packet = select_core0 ? l2req_packet0 : l2req_packet1;
			assign l2req_packet.core = !select_core0;
			assign l2req_ready0 = select_core0 && l2req_ready;
			assign l2req_ready1 = !select_core0 && l2req_ready;
	
			always_ff @(posedge reset, posedge clk)
			begin
				if (reset)
					select_core0 <= 0;
				else if (l2req_ready)
					select_core0 <= !select_core0;
			end
		end
		else
		begin
			assign halt1 = 1;
			assign l2req_packet = l2req_packet0;
			assign l2req_ready0 = l2req_ready;
		end
	endgenerate

	l2_cache l2_cache(.*);

`ifdef ENABLE_PERFORMANCE_COUNTERS
	performance_counters #(.NUM_COUNTERS(17)) performance_counters(
		.pc_event({
			pc_event_mem_ins_issue,
			pc_event_vector_ins_issue,
			pc_event_l2_writeback,
			pc_event_l2_wait,
			pc_event_l2_hit,
			pc_event_l2_miss,
			pc_event_l1d_hit,
			pc_event_l1d_miss,
			pc_event_l1i_hit,
			pc_event_l1i_miss,
			pc_event_store,
			pc_event_instruction_issue,
			pc_event_instruction_retire,
			pc_event_mispredicted_branch,
			pc_event_uncond_branch,
			pc_event_cond_branch_taken,
			pc_event_cond_branch_not_taken
		}),
		.*);
`endif
	
endmodule

// Local Variables:
// verilog-typedef-regexp:"_t$"
// End:
