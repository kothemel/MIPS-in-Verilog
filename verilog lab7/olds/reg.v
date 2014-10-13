module RegFile (clock, reset, raA, raB, wa, wen, wd, rdA, rdB);

  // Place your verilog code here. 
  // Remember that the register file should be written at the negative edge of the input clock
  
  output wire [31:0] rdA, rdB;
  input [31:0] wd;
  input [4:0] raA, raB, wa;
  input wen, reset, clock;
  reg [31:0] data [31:0];
  integer i;

  
  assign rdA = data[raA];
  assign rdB = data[raB];
  // Write Section
  always @(negedge clock, negedge reset)
    begin
    
      if (reset==1'b0)
	     begin
	      for(i=0;i<32;i=i+1)
	        data[i] <= 0;
      end
    
      else if (wen==1'b1)
      
        data[wa] <= wd;  

    end

    
endmodule