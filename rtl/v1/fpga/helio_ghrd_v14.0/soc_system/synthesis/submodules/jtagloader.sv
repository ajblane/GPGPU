//
// Copyright 2012 Brian Swetland
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

`timescale 1ns/1ns

module jtagloader(
	input clk,
	output we,
	output reg [31:0] addr,
	output [31:0] data,
	output reg reset,
//form the jtag interfce of the top module
        input tdi,
	output tdo,
	input tck,
	input ir_in,
	input virtual_state_sdr,
	input virtual_state_udr,
	input virtual_state_uir 
);

parameter IR_CTRL = 4'd0;
parameter IR_ADDR = 4'd1;
parameter IR_DATA = 4'd2;

initial reset = 0;

wire update;
wire [3:0] iir;
wire sdr, udr, uir;
reg [31:0] dr;
reg [3:0] ir;

assign tdo = dr[0];
assign iir = ir_in;
assign sdr = virtual_state_sdr;
assign udr = virtual_state_udr;
assign uir = virtual_state_uir;
/*
jtag jtag0(
	.tdi(tdi),
	.tdo(dr[0]),
	.tck(tck),
	.ir_in(iir),
	.virtual_state_sdr(sdr),
	.virtual_state_udr(udr),
	.virtual_state_uir(uir)
	);
*/
always @(posedge tck) begin
	if (uir) ir <= iir;
	if (sdr) dr <= { tdi, dr[31:1] };
	end

sync sync0(
	.in(udr),
	.clk_in(tck),
	.out(update),
	.clk_out(clk)
	);

assign data = dr;
assign we = update & (ir == IR_DATA);

always @(posedge clk)
	if (update) case (iir)
	IR_CTRL: reset <= dr[0];
	IR_ADDR: addr <= dr;
	IR_DATA: addr <= addr + 32'd4;
	endcase

endmodule

module sync(
	input clk_in,
	input clk_out,
	input in,
	output out
	);
reg toggle;
reg [2:0] sync;
always @(posedge clk_in)
	if (in) toggle <= ~toggle;
always @(posedge clk_out)
	sync <= { sync[1:0], toggle };
assign out = (sync[2] ^ sync[1]);
endmodule

