// Define top-level testbench
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Top level has no inputs or outputs
// It only needs to instantiate CPU, Drive the inputs to CPU (clock, reset)
// and monitor the outputs. This is what all testbenches do

`include "constants.h"
`timescale 1ns/1ps
`define clock_period 10

module cpu_tb;
integer   i;
reg       clock, reset;    // Clock and reset signals

// Instantiate CPU here with name cpu0
CPU cpu0 (clock, reset); 

// Initialization and signal generation
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Generate clock and reset signal here
 
  // Initialize Register File with initial values. 
  // cpu0 is the name of the cpu instance
  // cpu_regs is the name of the register file instance in the CPU verilog file 
  // data[] is the register file array 
initial begin 

	$dumpfile("lab8.vcd");
	$dumpvars(0, cpu_tb);
	clock = 1'b0;
	reset = 1'b0;
	 
	#(`clock_period*4) reset = 1'b1;


  // Initialize Data Memory. You have to develop "program.hex" as a text file 
  // which containsthe instruction opcodes as 32-bit hexadecimal values.
	for (i=0; i<4095; i=i+1)
		cpu0.datapass.mem_DATA.data[i]=32'b0;
	$readmemb("code2.mbin", cpu0.datapass.mem_INSTR.data);

  // Edw, to "program.hex" einai ena arxeio pou prepei na brisketai sto 
  // directory pou trexete th Verilog kai na einai ths morfhs:
  
  // @0    00000000
  // @4    20100009
  // @8    00000000
  // @C    00000000
  // ...
  
  // H aristerh sthlh, meta to @, exei th dieythynsh ths mnhmhs (hex),
  // kai h deksia sthlh ta dedomena sth dieythynsh ayth (pali hex).
  // Sto paradeigma pio panw, oi lekseis stis dieythynseis 0, 8 kai 12
  // einai 0, kai sth dieythynsh 4 exei thn timh 32'h20100009. An o PC
  // diabasei thn dieythynsh 4, h timh ekei exei thn entolh
  //   addi $16 <- $0 + 9

  // To deytero orisma ths $readmemh einai pou akribws brisketai h mnhmh
  // pou tha arxikopoihthei. Sto paradeigma, to "dat0" einai to onoma pou
  // dwsame sto instance tou datapath. To "mem" einai to onoma pou exei
  // to instance ths mnhmhs MESA sto datapath, kai to "data" einai to 
  // onoma pou exei to pragmatiko array ths mhnhs mesa sto module ths.
  // An exete dwsei diaforetika onomata, allakste thn $readmemh.

  // Enallaktika, an sas boleyei perissotero, yparxei h entolh $readmemb
  // me thn akribws idia syntaksh. H aristerh sthlh tou arxeiou exei
  // thn idia morfh (dieythynseis se hex), alla h deksia sthlh exei
  // ta dedomena sto dyadiko. Etsi h add mporouse na einai:

  // @4    00100000000100000000000000001001

  // ... h kai akoma kalytera:
  
  // @4    001000_00000_10000_0000000000001001

  // (h Verilog epitrepei diaxwristika underscores).

	
  // Termatismos ekteleshs:
	#(`clock_period * 40) $finish;

end  // initial 

always begin
	#(`clock_period/0.5) clock=~clock;
end
endmodule


