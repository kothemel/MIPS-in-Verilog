
module data_path (clock, reset, regWrite, alu_ctrl, op, func_code);

	input clock, reset, regWrite;
	input [3:0] alu_ctrl;
	output [5:0] op, func_code;

	wire [31:0] instr, pc_new, pc, rdA, rdB, din, out;


	// enwnoume ta stoixeia ektos tou FSM //
	ProgramCounter reloaded_pc (clock, reset, pc_new, pc);
	Adder additive (pc, pc_new);
	Memory mem (clock, reset, 1, 0, pc, din, instr);
	RegFile cpu_regs (clock, reset, instr[25:21], instr[20:16], instr[15:11], regWrite, out, rdA, rdB);
	ALU my_alu (out, zero, rdA, rdB, alu_ctrl);
	
	assign op = instr[31:26];
	assign func_code = instr [5:0];

endmodule
