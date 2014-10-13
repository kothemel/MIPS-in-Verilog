module Adder (current, next_pc);

	parameter step = 4;
	input [31:0] current;
	output wire [31:0] next_pc;

	assign next_pc = current + step;
	
endmodule


