`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/14/2025 04:29:14 PM
// Design Name: 
// Module Name: sync_test
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


module sync_test();

    logic clk = 1'b0;
    logic reset = 1'b0;
    logic hsync, vsync;
    logic[2:0] r, g;
    logic[1:0] b;

    vga_controller vga (
        .clk(clk),
        .reset(reset),
        .hsync(hsync),
        .vsync(vsync),
        .r(r),
        .g(g),
        .b(b)
    );
    
    always begin clk = ~clk; #5; end
    
    initial begin
        reset = 1'b1; #10;
        reset = 1'b0; #10;
    
        
    
    end

endmodule
