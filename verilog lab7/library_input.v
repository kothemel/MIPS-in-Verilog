`include "constants.h"


///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ALU ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// Small ALU. Inputs: inA, inB. Output: out. 									  //
// Operations: bitwise and (op = 0)												  //
//             bitwise or  (op = 1)												  //
//             addition (op = 2)												  //
//             subtraction (op = 6)												  //
//             slt  (op = 7)													  //
//             nor (op = 12)													  //
////////////////////////////////////////////////////////////////////////////////////

module ALU (out, zero, inA, inB, alu_ctrl);

  parameter N = 32;
  input [3:0] alu_ctrl;
  input [N-1:0] inA, inB;
  output reg [N-1:0] out;
  output wire zero;
  
  assign zero = (out==0);
  
  always @ (alu_ctrl, inA, inB) begin
    
    case (alu_ctrl)
      4'b0000: out <= inA&inB;
      4'b0001: out <= inA | inB;       	// or
      4'b0010: out <= inA + inB;       	// add
      4'b0110: out <= inA - inB;       	// substract
      4'b0111: out <= inA < inB ? 1:0; 	// slt
      4'b1100: out <= ~(inA | inB);		// nor
      default: out <= 1'bx;
    endcase
  end
  
endmodule

///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Program Counter ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				//
// 				Sets the new program counter as the current pc					//
//																				//
//////////////////////////////////////////////////////////////////////////////////

module ProgramCounter (clock, reset, PC_new, PC);

	input reset, clock;
	input [31:0] PC_new;
	output reg [31:0] PC;

	always @(posedge clock, negedge reset)
		begin
			if (reset == 1'b0)	// an to reset einai 0 krata 0 kai ton program counter
				PC <= 0;
			else
				PC <= PC_new;	// alliws dwse ston trexon pc timh ish me thn epomenh dieu8unsh
		end

endmodule

///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Memory ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				//
// 		Memory (active 1024 words, from 10 address lsbs).						//
//      Read : enable ren, address addr, data dout								//
//      Write: enable wen, address addr, data din.								//
//																				//
//////////////////////////////////////////////////////////////////////////////////

module Memory (clock, reset, ren, wen, addr, din, dout);
  input         ren, wen;
  input clock, reset;
  input  [31:0] addr, din;
  output [31:0] dout;

  reg [31:0] data[4095:0];
  wire [31:0] dout;

  always @(ren or wen)
    if (ren & wen)
      $display ("\nMemory ERROR (time %0d): ren and wen both active!\n", $time);

  always @(posedge ren or posedge wen) begin
    if (addr[31:10] != 0)
      $display("Memory WARNING (time %0d): address msbs are not zero\n", $time);
  end  

  assign dout = ((reset==1'b1) && (wen==1'b0) && (ren==1'b1)) ? data[addr[9:0]] : 32'bx;
  
  always @(negedge clock , negedge reset, din, wen, ren, addr)
   begin
    if ((reset==1'b1) && (wen == 1'b1) && (ren==1'b0))
        data[addr[9:0]] = din;
   end

endmodule

///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Register File ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				 //
// 		Read ports: address raA, data rdA										 //
//                          address raB, data rdB								 //
//      Write port: address wa, data wd, enable wen.							 //
//																				 //
///////////////////////////////////////////////////////////////////////////////////

module RegFile (clock, reset, raA, raB, wa, wen, wd, rdA, rdB);


	output wire [31:0] rdA, rdB;
	input [31:0] wd;
	input [4:0] raA, raB, wa;
	input wen, reset, clock;
	reg [31:0] data [31:0];
	integer i;


	assign rdA = data[raA];
	assign rdB = data[raB];
	// Write Section
	always @(negedge clock, negedge reset)	// the register file should be written at 
		begin								// the negative edge of the input clock

			if (reset==1'b0)
		 	begin
		  		for(i=0;i<32;i=i+1)
					data[i] <= 32'b0;		//otan to reset einai 0 mhdenizw olous ts kataxwrhtes
			end

			if ((wen==1'b1) && (reset==1'b1))	// alliws dwse ston ka8e kataxwrhth timh idia me
					data[wa] <= wd;  			// idia me ton au3oda ari8mo tu

		end


endmodule

///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ADDER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					Adds 4 bytes to move to the next instruction				  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////

module Adder (current, next_pc);

	parameter step = 4;
	input [31:0] current;
	output wire [31:0] next_pc;

	assign next_pc = current + step;
	
endmodule

///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CPU ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					A module that describes the function of cpu					  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////
                                                                                  
module CPU (clock, reset);

	input clock, reset;
	input regWrite;
	wire [1:0] alu_op;
	wire [3:0] alu_ctrl;
	wire [5:0] op, func_code;
	
	// enwnoume to FSM me ta upoloipa stoixeia //
	Main_decoder dec_main (op, regWrite, alu_op);
	Alu_decoder dec_alu (alu_op, func_code, alu_ctrl);
	data_path datapass (clock, reset, regWrite, alu_ctrl, op, func_code);

endmodule
