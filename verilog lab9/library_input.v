
`timescale 1ns/1ps
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

//////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Program Counter ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				//
// 				Sets the new program counter as the current pc					//
//																				//
//////////////////////////////////////////////////////////////////////////////////

module ProgramCounter (clock, reset, PC_new, PCWrite, PC);

	input reset, clock, PCWrite;
	input [31:0] PC_new;
	output reg [31:0] PC;

	always @(posedge clock or negedge reset)
		begin
			if (reset == 1'b0)	// an to reset einai 0 krata 0 kai ton program counter
				PC <= -4;
			else begin
				if (PCWrite)
					PC <= PC_new;	// alliws dwse ston trexon pc timh ish me thn epomenh dieu8unsh
				 /* if PCWrite==0 then keep the old pc and create a bubble */
			end
		end

endmodule

//////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Memory ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				//
// 		Memory (active 1024 words, from 10 address lsbs).						//
//      Read : enable ren, address addr, data dout								//
//      Write: enable wen, address addr, data din.								//
//																				//
//////////////////////////////////////////////////////////////////////////////////

module Memory (ren, wen, addr, din, dout);
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
  
  always @(din or wen or ren or addr)
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

////////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ADDER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					Adds 4 bytes to move to the next instruction				  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////

module Adder (current, step, next_pc);

	input [31:0] current, step;
	output wire [31:0] next_pc;

	assign next_pc = current + step;
	
endmodule

////////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ADDER2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					Adds 4 bytes to shifted instruction							  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////

//module Adder2 (shifted_instr, next_pc, next_instr);

//	input [31:0] shifted_instr, next_pc;
//	output wire [31:0] next_instr;

//	assign next_instr = shifted_instr + next_pc;
	
//endmodule



////////////////////////////////////////////////////////////////////////////////////
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


////////////////////////////////////////////////////////////////////////////////////
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

module MultiPlx3_1 (input1, input2, input3, signal, out);
	
	input [31:0] input1, input2, input3;
	input [1:0] signal;
	output reg [31:0] out;
	
	always @(*) begin
		if (signal == 2'b00) 
			out <= input1;
		else if (signal == 2'b01)
			out <= input2;
		else if (signal == 2'b10)
			out <= input3;
	end
endmodule


////////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~ Hazard Detection Unit~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					A module that describes the hazard detection unit			  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////

module hazard_unit(IFIDrs, IFIDrt, IDEXrt, IDEX_MemRead, PCWrite, IFIDWrite, HazMuxSignal); 
	input  [4:0] IFIDrs, IFIDrt, IDEXrt; 
	input  IDEX_MemRead; 
	output PCWrite, IFIDWrite, HazMuxSignal; 

	reg PCWrite, IFIDWrite, HazMuxSignal; 
	 
	always@(IFIDrs,IFIDrt,IDEXrt,IDEX_MemRead) 
		if(IDEX_MemRead==1'b1 && ((IDEXrt == IFIDrs) || (IDEXrt == IFIDrt)) ) 
			begin//stall 
				PCWrite = 0; 
				IFIDWrite = 0; 
				HazMuxSignal = 0; 
			end 
		else 
			begin//no stall 
				PCWrite = 1; 
				IFIDWrite = 1; 
				HazMuxSignal = 1; 
			end 
 
endmodule


////////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Forward Unit~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					A module that describes the function of forward unit		  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////

module forward_unit(IDEXrt, IDEXrs, EXMEMrd, EXMEM_RegWrite, MEMWBrd, MEMWB_RegWrite, forwardA, forwardB);

	input [4:0] IDEXrt, IDEXrs, EXMEMrd, MEMWBrd;
	input EXMEM_RegWrite, MEMWB_RegWrite;
	output reg [1:0] forwardA, forwardB;
	
	// FORWARD A
	always @(EXMEM_RegWrite or EXMEMrd or IDEXrs or MEMWB_RegWrite or MEMWBrd) begin
	
		if((EXMEM_RegWrite==1'b1)&&(EXMEMrd != 0)&&(EXMEMrd == IDEXrs)) 
 			forwardA = 2'b10; 
 		else if((MEMWB_RegWrite)&&(MEMWBrd != 0)&&(MEMWBrd == IDEXrs)&&((EXMEMrd != IDEXrs) || EXMEM_RegWrite==1'b0)) 
 			forwardA = 2'b01; 
		else 
			forwardA = 2'b00; 
	end 
 
 	//FORWARD B 
	always@(MEMWB_RegWrite or MEMWBrd or IDEXrt or EXMEMrd or EXMEM_RegWrite) begin 
	
		if((MEMWB_RegWrite)&&(MEMWBrd != 0)&&(MEMWBrd == IDEXrt)&&(EXMEMrd != IDEXrt || EXMEM_RegWrite==1'b0)) 
			forwardB = 2'b01; 
		else if((EXMEM_RegWrite)&&(EXMEMrd != 0)&&(EXMEMrd == IDEXrt)) 
			forwardB = 2'b10; 
		else 
			forwardB = 2'b00; 
	end

endmodule


////////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CPU ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//																				  //
// 					A module that describes the function of cpu					  //
//																				  //
////////////////////////////////////////////////////////////////////////////////////
                                                                                  
module CPU (clock, reset);

	input clock, reset;
	reg RegWrite, RegDest, AluSrc, MemWrite, MemReg, MemRead, PCsrc;
	reg [3:0] alu_ctrl;
	wire [5:0] op, func_code;
	output reg [1:0] alu_op;
	
	assign op = IFIDir[31:26];
	assign func_code = IDEXsigex[5:0];

////////////////////////////////////// Data Path /////////////////////////////////////////////////

	
	wire [31:0]	instr, pc_new, pc, rdA, rdB, din, wd, alu_out, adder2_out;
	wire [31:0]	extended_instr, shifted_instr, mem_out, out_multi3, out_multifw1, out_multifw2;
	wire [4:0]	out_multi1;
	wire [1:0]	forwardA, forwardB;	
	wire zero, PCWrite, HazMuxSignal, IFIDWrite;

	//~~~~~~~~~~~~~~~~~~~~~ Pipeline registers ~~~~~~~~~~~~~~~~~~~~~//
	reg [31:0]	IFIDir, IDEXsigex, IDEXa, IDEXb, EXMEMaluout, EXMEMrdB, MEMWBmemout, MEMWBaluout;
	reg [4:0]	IFIDrs, IFIDrt, IFIDrd, IDEXrs, IDEXrt, IDEXrd, EXMEMreg, MEMWBreg;
	reg [3:0]	IDEX_aluctrl;
	reg			IDEX_AluSrc, IDEX_RegDest, IDEX_RegWrite, IDEX_MemWrite, IDEX_MemRead, IDEX_MemToReg;
	reg			EXMEM_MemWrite, EXMEM_MemRead, EXMEM_RegWrite, EXMEM_MemToReg, MEMWB_RegWrite, MEMWB_MemToReg;

	
	always @(negedge clock or negedge reset) begin
		if (!reset) begin
			//IFIDir <= `NOP;
			//IDEXa  <= `NOP;
			//IDEXb  <= `NOP;			
		end
		else begin
			/* InstructionFetch -> InstructionDecode */
			if(IFIDWrite) begin
				IFIDir <= instr;
				IFIDrs <= instr[25:21];
				IFIDrt <= instr[20:16];
				IFIDrd <= instr[15:11];
			end

			/* InstructionDecode -> Execution */
			IDEXsigex <= extended_instr;
			IDEXa  <= rdA;
			IDEXb  <= rdB;
			IDEXrs <= IFIDrs;
			IDEXrt <= IFIDrt;
			IDEXrd <= IFIDrd;
			
			if (HazMuxSignal) begin
				IDEX_AluSrc   <= AluSrc; 
				IDEX_RegDest  <= RegDest;
				IDEX_RegWrite <= RegWrite;
				IDEX_MemWrite <= MemWrite;
				IDEX_MemRead  <= MemRead;
				IDEX_MemToReg <= MemReg;
			end
			else begin
				IDEX_AluSrc   <= 1'b0; 
				IDEX_RegDest  <= 1'b0;
				IDEX_RegWrite <= 1'b0;
				IDEX_MemWrite <= 1'b0;
				IDEX_MemRead  <= 1'b0;
				IDEX_MemToReg <= 1'b0;
			end

			/* Execution -> Memory */
			EXMEMaluout <= alu_out;
			EXMEMreg    <= IDEX_RegDest ? IDEXrd : IDEXrt;
			EXMEMrdB    <= IDEXb;
			
			EXMEM_MemWrite <= IDEX_MemWrite;
			EXMEM_MemRead  <= IDEX_MemRead;
			EXMEM_RegWrite <= IDEX_RegWrite;
			EXMEM_MemToReg <= IDEX_MemToReg;

			/* Memory -> WriteBack */
			MEMWBmemout <= mem_out;
			MEMWBaluout <= EXMEMaluout;
			MEMWBreg    <= EXMEMreg;

			MEMWB_MemToReg <= EXMEM_MemToReg;
			MEMWB_RegWrite <= EXMEM_RegWrite;
		end
	end
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
	

	assign op = IFIDir[31:26];
	assign func_code = IDEXsigex[5:0];
	// enwnoume ola ta stoixeia //
	ProgramCounter		reloaded_pc	(clock, reset, pc_new, PCWrite, pc);
	Adder				additive	(pc, 4, pc_new);
	Memory				mem_INSTR 	(1'b1, 1'b0, pc, , instr);
	hazard_unit			hazardous	(IFIDrs, IFIDrt, IDEXrt, IDEX_MemRead, PCWrite, IFIDWrite, HazMuxSignal);
	
	RegFile				cpu_regs	(clock, reset, IFIDrs, IFIDrt, MEMWBreg, MEMWB_RegWrite, wd, rdA, rdB);
	SignExtend			extended 	(IFIDir[15:0], extended_instr);
	
	
	
	ALU 				my_alu 		(alu_out, zero, out_multifw1, out_multi3, alu_ctrl);
	forward_unit		fu		(IDEXrt, IDEXrs, EXMEMreg, EXMEM_RegWrite, MEMWBreg, MEMWB_RegWrite, forwardA, forwardB);
	MultiPlx_Reg_ALU  	multi2 	    (IDEX_AluSrc, out_multifw2, IDEXsigex, out_multi3);
	MultiPlx3_1			multifw1	(IDEXa, wd, EXMEMaluout, forwardA, out_multifw1);
	MultiPlx3_1			multifw2	(IDEXb, wd, EXMEMaluout, forwardB, out_multifw2);	
	
	
	
	Memory 				mem_DATA 	(EXMEM_MemRead, EXMEM_MemWrite, EXMEMaluout, EXMEMrdB, mem_out);
	MultiPlx_Mem_Reg  	multi3 	    (MEMWB_MemToReg, MEMWBaluout, MEMWBmemout, wd);
	


	
	
///////////////////////////////////////////////////////////////////////////////////////////////////	
//////////////////////////////// Main decoder & ALU decoder ///////////////////////////////////////
		always @(*)
		case (op)
			`R_FORMAT: 
				begin
					alu_op <= 2'b10;

					RegWrite <= 1'b1;
					RegDest <= 1'b1;
					AluSrc <= 1'b0;
					MemWrite <= 1'b0;
					MemReg <= 1'b0;
					MemRead <= 1'b0;
				end
			`LW: 
				begin
					alu_op <= 2'b00;

					RegWrite <= 1'b1;
					RegDest <= 1'b0;
					AluSrc <= 1'b1;
					MemWrite <= 1'b0;
					MemReg <= 1'b1;
					MemRead <= 1'b1;
				end
			`SW: 
				begin
					alu_op <= 2'b00;

					RegWrite <= 1'b0;
					RegDest <= 1'bx;
					AluSrc <= 1'b1;
					MemWrite <= 1'b1;
					MemReg <= 1'bx;
					MemRead <= 1'b0;
				end
			 default: alu_op <= 2'b11;  // ~NOP~ we should never reach this place
		endcase



	always @(func_code or op)
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
///////////////////////////////////////////////////////////////////////////////////////////////////


endmodule
