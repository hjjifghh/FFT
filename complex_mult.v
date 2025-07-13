module complex_mult (
    input wire [15:0] a_real, a_imag,
    input wire [15:0] b_real, b_imag,
    output wire [15:0] out_real, out_imag
);

parameter DATA_WIDTH = 16;

wire [31:0] prod1, prod2, prod3, prod4;

assign prod1 = a_real * b_real;
assign prod2 = a_imag * b_imag;
assign prod3 = a_real * b_imag;
assign prod4 = a_imag * b_real;

assign out_real = (prod1 - prod2) >>> (DATA_WIDTH-1);
assign out_imag = (prod3 + prod4) >>> (DATA_WIDTH-1);

endmodule 
