`include "constants.h"
`timescale 1ns/1ps

module Main_decoder (op, RegWrite, alu_op, RegDist, AluSrc, Branch, MemWrite, MemReg, MemRead);
	input [5:0] op;
	output reg [1:0] alu_op, Branch;
	output reg RegDist, RegWrite, AluSrc, MemWrite, MemReg, MemRead;

	always @(*)
		case (op)
			`R_FORMAT: 
				begin
					alu_op <= 2'b10;

					RegWrite <= 1'b1;
					RegDist <= 1'b1;
					AluSrc <= 1'b0;
					Branch <= 2'b00;
					MemWrite <= 1'b0;
					MemReg <= 1'b0;
					MemRead <= 1'b0;
				end
			`LW: 
				begin
					alu_op <= 2'b00;

					RegWrite <= 1'b1;
					RegDist <= 1'b0;
					AluSrc <= 1'b1;
					Branch <= 2'b00;
					MemWrite <= 1'b0;
					MemReg <= 1'b1;
					MemRead <= 1'b1;
				end
			`SW: 
				begin
					alu_op <= 2'b00;

					RegWrite <= 1'b0;
					RegDist <= 1'bx;
					AluSrc <= 1'b1;
					Branch <= 2'b00;
					MemWrite <= 1'b1;
					MemReg <= 1'bx;
					MemRead <= 1'b0;
				end
			`BEQ: 
				begin
					alu_op <= 2'b01;

					RegWrite <= 1'b0;
					RegDist <= 1'bx;
					AluSrc <= 1'b0;
					Branch <= 2'b11;
					MemWrite <= 1'b0; 
					MemReg <= 1'bx;
					MemRead <= 1'b0;
				end
			`BNE: 
				begin
					alu_op <= 2'b01;

					RegWrite <= 1'b0;
					RegDist <= 1'bx;
					AluSrc <= 1'b0;
					Branch <= 2'b10;
					MemWrite <= 1'b0; 
					MemReg <= 1'bx;
					MemRead <= 1'b0;
				end
			 default: alu_op <= 2'b11;  // ~NOP~ we should never reach this place
		endcase
endmodule

module Alu_decoder (alu_op, func_code, alu_ctrl);

	input [1:0] alu_op;
	input [5:0] func_code;
	output reg [3:0] alu_ctrl;

	always @(*)
		case (alu_op)
		2'b00: alu_ctrl <= 4'b0010; // if lw or sw then add
		2'b01: alu_ctrl <= 4'b0110;	// if beq or bne then sub
		2'b10:
			case (func_code)
				6'b100100: alu_ctrl <= 4'b0000;	// and
				6'b100101: alu_ctrl <= 4'b0001;	// or
				6'b100000: alu_ctrl <= 4'b0010;	// add
				6'b100010: alu_ctrl <= 4'b0110;	// substract
				6'b101010: alu_ctrl <= 4'b0111;	// slt
				6'b100111: alu_ctrl <= 4'b1100;	// nor
				default: alu_ctrl <= 4'b1111; 		// we should never reach this place!!!!!!!!!!!
			endcase
		default: alu_ctrl <= 4'b1111;
	endcase
endmodule


