`include "constants.h"

module Main_decoder (op, regWrite, alu_op);
	input [5:0] op;
	input regWrite;
	output reg [1:0] alu_op;

	always @(*)
		case (op)
			`R_FORMAT: alu_op <= 2'b10;	// R-FORMAT
			`LW: alu_op <= 2'b00;		// LW
			`SW: alu_op <= 2'b00;		// SW
			`BEQ: alu_op <= 2'b01;		// BEQ
			 default: alu_op <= 2'b11;  // NOP
		endcase
endmodule

module Alu_decoder (alu_op, func_code, alu_ctrl);

	input [1:0] alu_op;
	input [5:0] func_code;
	output reg [3:0] alu_ctrl;

	always @(*)
		case (func_code)
			6'b100100: alu_ctrl <= 4'b0000;	// and
			6'b100101: alu_ctrl <= 4'b0001;	// or
			6'b100000: alu_ctrl <= 4'b0010;	// add
			6'b100010: alu_ctrl <= 4'b0110;	// substract
			6'b101010: alu_ctrl <= 4'b0111;	// slt
			6'b100111: alu_ctrl <= 4'b1100;	// nor
		default: alu_ctrl <= 4'b1111; 		// we should never reach this place!!!!!!!!!!!
	endcase
endmodule


