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
// Stage 1 of the floating point addition pipeline
// - Compute the amount to shift the exponents to align the significands
// - Swap significands if needed so the smaller one is in the second slot
// - Convert the significands to two's complement
//

module fp_adder_stage1
	(input                                    clk,
	input                                     reset,
	input arith_opcode_t                      ds_alu_op,
	input [31:0]                              operand1,
	input [31:0]                              operand2,
	output logic[5:0]                         add1_operand_align_shift,
	output logic[`FP_SIGNIFICAND_WIDTH + 2:0] add1_significand1,
	output logic[`FP_EXPONENT_WIDTH - 1:0]    add1_exponent1,
	output logic[`FP_SIGNIFICAND_WIDTH + 2:0] add1_significand2,
	output logic[`FP_EXPONENT_WIDTH - 1:0]    add1_exponent2,
	output logic                              add1_exponent2_larger);

	logic[`FP_SIGNIFICAND_WIDTH + 2:0] swapped_significand1_nxt;
	logic[`FP_SIGNIFICAND_WIDTH + 2:0] swapped_significand2_nxt;
	logic[5:0] operand_align_shift_nxt;
	logic[`FP_SIGNIFICAND_WIDTH + 2:0] twos_complement_significand1;
	logic[`FP_SIGNIFICAND_WIDTH + 2:0] twos_complement_significand2;

	wire sign1 = operand1[`FP_EXPONENT_WIDTH + `FP_SIGNIFICAND_WIDTH];
	wire[`FP_EXPONENT_WIDTH - 1:0] exponent1 = operand1[`FP_EXPONENT_WIDTH + `FP_SIGNIFICAND_WIDTH - 1:`FP_SIGNIFICAND_WIDTH];
	wire[`FP_SIGNIFICAND_WIDTH - 1:0] significand1 = operand1[`FP_SIGNIFICAND_WIDTH - 1:0];
	wire sign2 = operand2[`FP_EXPONENT_WIDTH + `FP_SIGNIFICAND_WIDTH];
	wire[`FP_EXPONENT_WIDTH - 1:0] exponent2 = operand2[`FP_EXPONENT_WIDTH + `FP_SIGNIFICAND_WIDTH - 1:`FP_SIGNIFICAND_WIDTH];
	wire[`FP_SIGNIFICAND_WIDTH - 1:0] significand2 = operand2[`FP_SIGNIFICAND_WIDTH - 1:0];

	// Compute exponent difference
	wire[`FP_EXPONENT_WIDTH:0] exponent_difference = exponent1 - exponent2; // Note extra carry bit
	wire exponent2_larger = exponent_difference[`FP_EXPONENT_WIDTH];

	// Take absolute value of the exponent difference to compute the shift amount
	wire[8:0] difference_abs = exponent2_larger 
		? ~exponent_difference + 1
		: exponent_difference;

	assign operand_align_shift_nxt = difference_abs > `FP_SIGNIFICAND_WIDTH + 2
		? `FP_SIGNIFICAND_WIDTH + 2
		: difference_abs;
		
	// Handling for subnormal numbers
	wire hidden_bit1 = exponent1 != 0;
	wire hidden_bit2 = exponent2 != 0;
	
	wire addition = ds_alu_op == OP_FADD;

	// Convert significands to 2s complement
	always_comb
	begin
		if (sign1)
			twos_complement_significand1 = ~{ 2'b00, hidden_bit1, significand1 } + 1;
		else
			twos_complement_significand1 = { 2'b00, hidden_bit1, significand1 };

		if (sign2 ^ !addition)
			twos_complement_significand2 = ~{ 2'b00, hidden_bit2, significand2 } + 1;
		else
			twos_complement_significand2 = { 2'b00, hidden_bit2, significand2 };
	end

	// Swap
	always_comb
	begin
		if (exponent2_larger)
		begin
			swapped_significand1_nxt = twos_complement_significand2;
			swapped_significand2_nxt = twos_complement_significand1;
		end
		else
		begin
			swapped_significand1_nxt = twos_complement_significand1;
			swapped_significand2_nxt = twos_complement_significand2;
		end
	end

	always_ff @(posedge clk, posedge reset)
	begin
		if (reset)
		begin
			/*AUTORESET*/
			// Beginning of autoreset for uninitialized flops
			add1_exponent1 <= {(1+(`FP_EXPONENT_WIDTH-1)){1'b0}};
			add1_exponent2 <= {(1+(`FP_EXPONENT_WIDTH-1)){1'b0}};
			add1_exponent2_larger <= 1'h0;
			add1_operand_align_shift <= 6'h0;
			add1_significand1 <= {(1+(`FP_SIGNIFICAND_WIDTH+2)){1'b0}};
			add1_significand2 <= {(1+(`FP_SIGNIFICAND_WIDTH+2)){1'b0}};
			// End of automatics
		end
		else
		begin
			add1_operand_align_shift <= operand_align_shift_nxt;
			add1_significand1 <= swapped_significand1_nxt;
			add1_significand2 <= swapped_significand2_nxt;
			add1_exponent1 <= exponent1;
			add1_exponent2 <= exponent2;
			add1_exponent2_larger <= exponent2_larger;
		end
	end	
endmodule

// Local Variables:
// verilog-typedef-regexp:"_t$"
// End:

