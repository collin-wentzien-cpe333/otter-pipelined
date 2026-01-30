`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////// 
// Company: 
// Engineer: 
// 
// Create Date: 04/28/2025 05:23:50 PM
// Design Name: 
// Module Name: branch_addr_gen
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


module branch_addr_gen(
    input [31:0] PC, j, b, i, rs,
    output [31:0] jal, jalr, branch
    );
    
    assign jal = j + PC;
    assign jalr = i + rs;
    assign branch = b + PC;
    
endmodule
