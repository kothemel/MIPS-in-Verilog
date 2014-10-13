
`timescale 1ns/1ps

module data_path (clock, reset, RegWrite, alu_ctrl, op, func_code, RegDist, AluSrc, Branch, MemWrite, MemReg, MemRead);

	input clock, reset;
	input RegWrite, RegDist, AluSrc, Branch, MemWrite, MemReg, MemRead;
	input [3:0] alu_ctrl;
	output [5:0] op, func_code;
	
	wire [31:0] instr, added_pc, pc_new, pc, rdA, rdB, din, wd, alu_out, adder2_out;
	wire [31:0] extended_instr, shifted_instr, mem_out, out_multi3;
	wire [4:0] out_multi1;
	wire and_out, zero;


	// enwnoume ola ta stoixeia //
	ProgramCounter	reloaded_pc	(clock, reset, pc_new, pc);
	Adder			additive	(pc, added_pc);
	
	Memory			mem_INSTR 	(clock, 1'b1, 1'b0, pc, 32'b0, instr);
	RegFile			cpu_regs	(clock, reset, instr[25:21], instr[20:16], out_multi1, RegWrite, wd, rdA, rdB);
	AND				and_gate	(zero, Branch, and_out);
	SignExtend		extended 	(instr[15:0], extended_instr);
	Sifter			sifted		(extended_instr, shifted_instr);
	Adder2			adder2 		(shifted_instr, added_pc, adder2_out);
	
	MultiPlx_Ctrl_Reg multi1 	(RegDist, instr[20:16], instr[15:11], out_multi1);
	MultiPlx_Add2_PC  multi2 	(and_out, added_pc, adder2_out, pc_new);
	MultiPlx_Reg_ALU  multi3 	(AluSrc, rdB, extended_instr, out_multi3);
	MultiPlx_Mem_Reg  multi4 	(MemReg, alu_out, mem_out, wd);
	
	ALU 			my_alu 		(alu_out, zero, rdA, out_multi3, alu_ctrl);
	Memory 			mem_DATA 	(clock, MemRead, MemWrite, alu_out, rdB, mem_out);
	
	assign op = instr[31:26];
	assign func_code = instr [5:0];

endmodule
