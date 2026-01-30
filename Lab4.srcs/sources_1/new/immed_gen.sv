`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/28/2025 05:04:24 PM
// Design Name: 
// Module Name: immed_gen
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


module immed_gen(
    input [31:0] ir,
    output [31:0] i, s, b, u, j
    );
    
    assign i = {{21{ir[31]}}, ir[30:25], ir[24:20]};
    assign s = {{21{ir[31]}}, ir[30:25], ir[11:7]};
    assign b = {{20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0};
    assign u = {ir[31:12], 12'b0};
    assign j = {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0};
    
endmodule
