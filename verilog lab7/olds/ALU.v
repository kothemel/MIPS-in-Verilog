module ALU (out, zero, inA, inB, op);

  parameter N = 8;
  input [3:0] op;
  input [N-1:0] inA, inB;
  output reg [N-1:0] out;
  output wire zero;
  
  assign zero = (out==0);
  
  always @ (op, inA, inB) begin
    
    case (out)
      0: out <= inA&inB;
      1: out <= inA | inB;       	// or
      2: out <= inA + inB;       	// add
      6: out <= inA - inB;       	// substract
      7: out <= inA < inB ? 1:0; 	// slt
      12: out <= ~(inA | inB);		// nor
      default: out <= 0;
    endcase
  end
  
endmodule
