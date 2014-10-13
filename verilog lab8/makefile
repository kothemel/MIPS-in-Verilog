# Ioannis Konstadelias, gon1332
CC = iverilog
FLAGS = -Wall -Winfloop

library_input: constants.h ctrl_unit.v data_path.v testbench_input.v library_input.v
	$(CC) $(FLAGS) -o test_it ctrl_unit.v data_path.v testbench_input.v library_input.v

	vvp test_it
	gtkwave -o lab8.vcd config.gtkw
