
module CPU (clock, reset);

	input clock, reset;
	input [4:0] regWrite;
	wire [1:0] alu_op;
	wire [3:0] alu_ctrl;
	wire [5:0] op, func_code;

	Main_decoder dec_main (op, regWrite, alu_op);
	Alu_decoder dec_alu (alu_op, func_code, alu_ctrl);
	data_path datpass (clock, reset, regWrite, alu_ctrl, op, func_code);

endmodule
