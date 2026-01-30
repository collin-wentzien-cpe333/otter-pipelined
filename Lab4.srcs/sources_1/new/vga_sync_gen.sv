`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/14/2025 03:58:00 PM
// Design Name: 
// Module Name: vga_sync_gen
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


module vga_sync_gen(
    input vclk,
    input reset,
    output hsync, vsync,
    output [9:0] h,
    output [8:0] v,
    output en
    );
    
    //640 x 480 @ 25.175MHz
    reg[9:0] x = 0;
    reg[9:0] y = 0;
    
    assign hsync = (x > 655 && x < 752) ? 1'b0 : 1'b1;
    assign vsync = (y > 489 && y < 492) ? 1'b0 : 1'b1;
    assign en = (x < 640 && y < 480) ? 1'b1 : 1'b0;
    
    assign h = (x < 640) ? x[9:1] : 9'b0;
    assign v = (y < 480) ? y[9:1] : 8'b0;
    
    //counter logic
    always @(posedge vclk) begin
        if(reset == 1) begin
            x <= 0;
            y <= 0;
        end
        if(x == 799) begin
            x <= 0;
            
            if(y == 524)
                y <= 0;
            else
                y <= y + 1;
        end
        else begin
            x <= x + 1;
        end
    end
    
endmodule
