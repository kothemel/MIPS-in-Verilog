# Ioannis Konstadelias, gon1332
CC = iverilog
FLAGS = -Wall -Winfloop

library_input: constants.h testbench_input.v library_input.v
	$(CC) $(FLAGS) -o test_it testbench_input.v library_input.v

	./masmbin -c -f translate_instruction.txt program.masm program.mbin
	vvp test_it
	gtkwave -o lab.vcd config.gtkw
