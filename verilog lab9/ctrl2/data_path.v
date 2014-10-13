`include "constants.h"
`timescale 1ns/1ps

module data_path (clock, reset, RegWrite, alu_ctrl, op, func_code, RegDest, AluSrc, MemWrite, MemReg, MemRead);

	input 		 clock, reset;
	input		 RegWrite, RegDest, AluSrc, MemWrite, MemReg, MemRead;
	input [3:0]	 alu_ctrl;
	output [5:0] op, func_code;
	
	wire [31:0]  instr, added_pc, pc_new, pc, rdA, rdB, din, wd, alu_out, adder2_out;
	wire [31:0]  extended_instr, shifted_instr, mem_out, out_multi3;
	wire [4:0]   out_multi1;
	wire zero;

	//~~~~~~~~~~~~~~~~~~~~~ Pipeline registers ~~~~~~~~~~~~~~~~~~~~~//
	reg [31:0]	IFIDir, IDEXsigex, IDEXa, IDEXb, EXMEMaluout, EXMEMrdB, MEMWBmemout, MEMWBaluout;
	reg [4:0]	IFIDrs, IFIDrt, IFIDrd, IDEXrs, IDEXrt, IDEXrd, EXMEMreg, MEMWBreg;
	reg [3:0]	IDEX_aluctrl;
	reg			IDEX_AluSrc, IDEX_RegDest, IDEX_RegWrite, IDEX_MemWrite, IDEX_MemRead, IDEX_MemToReg;
	reg			EXMEM_MemWrite, EXMEM_MemRead, EXMEM_RegWrite, EXMEM_MemToReg, MEMWB_RegWrite, MEMWB_MemToReg;

	
	always @(negedge clock or negedge reset) begin
		if (1'b0 == reset) begin
			IFIDir <= `NOP;
			IDEXa <= `NOP;
			IDEXb <= `NOP;
		end else begin
			/* IF  */
			IFIDir <= instr;
			IFIDrs <= instr[25:21];
			IFIDrt <= instr[20:16];
			IFIDrd <= instr[15:11];

			/* ID */
			IDEXsigex <= extended_instr;
			IDEXa <= rdA;
			IDEXb <= rdB;
			IDEXrs <= IFIDrs;
			IDEXrt <= IFIDrt;
			IDEXrd <= IFIDrd;

			IDEX_aluctrl <= alu_ctrl;
			IDEX_AluSrc <= AluSrc; 
			IDEX_RegDest <= RegDest;
			IDEX_RegWrite <= RegWrite;
			IDEX_MemWrite <= MemWrite;
			IDEX_MemRead <= MemRead;
			IDEX_MemToReg <= MemReg;


			/* Execution */
			EXMEMaluout <= alu_out;
			EXMEMreg <= IDEX_RegDest ? IDEXrd : IDEXrt;
			EXMEMrdB <= IDEXb;
			
			EXMEM_MemWrite <= IDEX_MemWrite;
			EXMEM_MemRead <= IDEX_MemRead;
			EXMEM_RegWrite <= IDEX_RegWrite;
			EXMEM_MemToReg <= IDEX_MemToReg;

			/* Memory */
			MEMWBmemout <= mem_out;
			MEMWBaluout <= EXMEMaluout;
			MEMWBreg <= EXMEMreg;

			MEMWB_MemToReg <= EXMEM_MemToReg;
			MEMWB_RegWrite <= EXMEM_RegWrite;
		end
	end
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
	


	
	// enwnoume ola ta stoixeia //
	ProgramCounter	reloaded_pc	(clock, reset, pc_new, pc);
	Adder			additive	(pc, pc_new);
	Memory			mem_INSTR 	(1'b1, 1'b0, pc, , instr);
	
	RegFile			cpu_regs	(clock, reset, IFIDrs, IFIDrt, MEMWBreg, MEMWB_RegWrite, wd, rdA, rdB);
	SignExtend		extended 	(IFIDir[15:0], extended_instr);
	
	//Sifter			sifted		(extended_instr, shifted_instr);
	//Adder2			adder2 		(shifted_instr, added_pc, adder2_out);
	
	MultiPlx_Reg_ALU  multi2 	(IDEX_AluSrc, IDEXb, extended_instr, out_multi3);
	ALU 			my_alu 		(alu_out, zero, IDEXa, out_multi3, IDEX_aluctrl);
	
	
	Memory 			mem_DATA 	(EXMEM_MemRead, EXMEM_MemWrite, EXMEMaluout, EXMEMrdB, mem_out);
	//MultiPlx_Ctrl_Reg multi1 	(RegDist, instr[20:16], instr[15:11], out_multi1);
	MultiPlx_Mem_Reg  multi3 	(MEMWB_MemToReg, MEMWBaluout, MEMWBmemout, wd);
	


	
	assign op = IFIDir[31:26];
	assign func_code = IDEXsigex[5:0];

endmodule
