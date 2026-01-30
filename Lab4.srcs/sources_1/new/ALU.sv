`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/23/2025 05:32:46 PM
// Design Name: 
// Module Name: ALU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ALU(
    input [31:0] a, b,
    input [3:0] ALU_FUN,
    output reg [31:0] c
    );
    
    always_comb begin
    
        case(ALU_FUN)
            4'b0000: c = a + b; //add
            4'b1000: c = a - b; //subtract
            4'b0110: c = a | b; //or
            4'b0111: c = a & b; //and
            4'b0100: c = a ^ b; //xor
            4'b0101: c = a >> b[4:0]; //srli
            4'b0001: c = a << b[4:0]; //slli
            4'b1101: c = $signed(a) >>> b[4:0]; //sla
            4'b0010: c = $signed(a) < $signed(b); //set less than
            4'b0011: c = a < b; //sltu
            4'b1001: c = a; //lui
            default: c = 32'hDEADBEEF; //shit
        endcase
    
    end
    
endmodule