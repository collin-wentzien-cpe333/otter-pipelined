`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/14/2025 03:50:50 PM
// Design Name: 
// Module Name: vga_controller
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


module vga_controller(
    input clk,
    input reset,
    output hsync, vsync,
    output [2:0] r,
    output [2:0] g,
    output [1:0] b
    );
    
    reg vclk;
    
    wire[9:0] h;
    wire[8:0] v;
    wire en;
    
    //test
    assign r = en ? 3'b111 : 3'b000;
    assign g = en ? 3'b111 : 3'b000;
    assign b = en ? 2'b11 : 2'b00;
    
    clk_2n_div_test #(.n(2)) vga_clk (
        .clockin(clk),
        .fclk_only(1'b0),
        .clockout(vclk)
    );
    
    vga_sync_gen sync (
        .vclk(vclk),
        .reset(reset),
        .hsync(hsync),
        .vsync(vsync),
        .h(h),
        .v(v),
        .en(en)
    );
        
    
endmodule
