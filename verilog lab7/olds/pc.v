module pc (clock, reset, PC, PC_new);

	input reset, clock;
	input [31:0] PC_new;
	output reg [31:0] PC;

	always @(posedge clock or negedge reset)
		begin
			if (reset == 1'b0)
				PC <= 0;
			else
				PC <= PC_new;
		end

endmodule
