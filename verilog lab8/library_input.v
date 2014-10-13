
`timescale 1ns/1ps


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

  input [3:0] alu_ctrl;
  input [31:0] inA, inB;
  output reg [31:0] out;
  output wire zero;
  
  assign zero = out ? 0 : 1;
  
  always @ (alu_ctrl, inA, inB) begin
    
    case (alu_ctrl)
      4'b0000: out <= inA&inB;
      4'b0001: out <= inA | inB;       	// or
      4'b0010: out <= inA + inB;       	// add
      4'b0110: out <= inA - inB;       	// substract
      4'b0111: out <= inA < inB ? 1:0; 	// slt
      4'b1100: out <= ~(inA | inB);		// nor
      default: out <= 'bx;
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

	always @(posedge clock or negedge reset)
		begin
			if (reset == 1'b0)	// an to reset einai 0 krata 0 kai ton program counter
				PC <= 32'h0;
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

module Memory (clock, ren, wen, addr, din, dout);
  input		ren, wen, clock;
  input		[31:0] addr, din;
  output	[31:0] dout;

  reg [31:0] data[4095:0];
  wire [31:0] dout;

  always @(ren or wen)
    if (ren & wen)
      $display ("\nMemory ERROR (time %0d): ren and wen both active!\n", $time);

  always @(posedge ren or posedge wen) begin
    if (addr[31:10] != 0)
      $display("Memory WARNING (time %0d): address msbs are not zero\n", $time);
  end  

  assign dout = ((wen==1'b0) && (ren==1'b1)) ? data[addr[9:0]] : 32'bx;
  
  always @(negedge clock)
   begin
    if ((wen == 1'b1) && (ren==1'b0))
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
	always @(negedge clock or negedge reset)	// the register file should be written at 
		begin								// the negative edge of the input clock

			if (reset==1'b0)
	  			for(i=0;i<32;i=i+1)
					data[i] <= i;		//otan to reset einai 0 mhdenizw olous ts kataxwrhtes
			else if (wen==1'b1)		// alliws dwse ston ka8e kataxwrhth timh idia me
				data[wa] <= wd;  	// idia me ton au3oda ari8mo tu
		end


endmodule

///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ADDER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					Adds 4 bytes to move to the next instruction				  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////

module Adder (current, next_pc);

	parameter step = 32'h4;
	input [31:0] current;
	output wire [31:0] next_pc;

	assign next_pc = current + step;
	
endmodule

////////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ADDER2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					Adds 4 bytes to shifted instruction							  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////

module Adder2 (shifted_instr, next_pc, next_instr);

	input [31:0] shifted_instr, next_pc;
	output wire [31:0] next_instr;

	assign next_instr = shifted_instr + next_pc;
	
endmodule



///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SignExtend ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					A module that extends the sign and adds zero				  //
//							   at the 2 LSB										  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////

module SignExtend (instr, extended_instr);

	input[15:0] instr;
	output wire [31:0] extended_instr;
	
    assign extended_instr = {{16{instr[15]}}, instr[15:0]};

endmodule


///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Sifter ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					A module that shifts left the LSB bits						  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////

module Sifter (extended_instr, shifted_instr);

	input [31:0] extended_instr;
	output wire [31:0] shifted_instr;
	
	assign shifted_instr = extended_instr << 2;	
	
endmodule

////////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ AND ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 							A simple 'and' gate									  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////


module AND (zero, branch, select_signal);
	input zero, branch;
	output wire select_signal;
	
	assign select_signal = zero & branch;

endmodule


///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Multiplexers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					Four modules for each multiplexers in the circuit			  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////


module MultiPlx_Ctrl_Reg (select, input1, input2, out);

	input [4:0] input1, input2;
	input select;
	output wire [4:0] out;
	
	assign out = (select==1'b1) ? input2 : input1;
	
endmodule


module MultiPlx_Add2_PC (select_signal, new_instr, next_pc, new_program_counter);

	input select_signal;
	input [31:0] new_instr, next_pc;
	output wire [31:0] new_program_counter;
	
	assign new_program_counter = (select_signal==1'b1) ? next_pc : new_instr;
	
endmodule


module MultiPlx_Reg_ALU (AluSrc, rdB, extended_instr, inB);

	input AluSrc;
	input [31:0] rdB, extended_instr;
	output wire [31:0] inB;
	
	assign inB = (AluSrc==1'b1) ? extended_instr : rdB;
	
endmodule


module MultiPlx_Mem_Reg (MemReg, dout, out, wd);
	
	input MemReg;
	input [31:0] dout, out;
	output wire [31:0] wd;
	
	assign wd = (MemReg==1'b1) ? out : dout;
	
endmodule

///////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CPU ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					A module that describes the function of cpu					  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////
                                                                                  
module CPU (clock, reset);

	input clock, reset;
	wire [1:0] alu_op;
	wire RegWrite, RegDist, AluSrc, Branch, MemWrite, MemReg, MemRead;
	wire [3:0] alu_ctrl;
	wire [5:0] op, func_code;
	
	// enwnoume to FSM me ta upoloipa stoixeia //
	Main_decoder	dec_main	(op, RegWrite, alu_op, RegDist, AluSrc, Branch, MemWrite, MemReg, MemRead);
	Alu_decoder		dec_alu		(alu_op, func_code, alu_ctrl);
	data_path		datapass 	(clock, reset, RegWrite, alu_ctrl, op, func_code, RegDist, AluSrc, Branch, MemWrite, MemReg, MemRead);

endmodule
