`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Ratner Surf Designs
// Engineer:  James Ratner
// 
// Create Date: 01/07/2020 12:59:51 PM
// Design Name: 
// Module Name: tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Testbench file for Exp 4
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
//          (02-20-2024) v1.01 - Changed PC signal names to match schematic
//
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module sim1(); 

   reg reset; 
   reg PC_WE; 
   reg [1:0] PC_SEL; 
   reg CLK; 
   wire [31:0] u, s; 

PC prog(
    .CLK  (CLK),
    .reset  (reset),
    .PC_WE  (PC_WE),
    .PC_SEL  (PC_SEL),
    .u (u),
    .s  (s) ); 
  

   //- Generate periodic clock signal    
   initial    
      begin       
         CLK = 0;   //- init signal        
         forever  #10 CLK = ~CLK;    
      end                        
         
   initial        
   begin           
      reset=1;
      PC_WE = 1; 
      PC_SEL = 0; 
      
      #40
      reset=0; 


    end
        
endmodule