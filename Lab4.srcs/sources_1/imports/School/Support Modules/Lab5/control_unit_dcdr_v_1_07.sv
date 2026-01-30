`timescale 1ns / 1ps
///////////////////////////////////////////////////////////////////////////
// Company: Wentzien Surf Designs
// Engineer: Collin Wentzien
// 
// Create Date: 01/29/2019 04:56:13 PM
// Design Name: MCU!!!!!
// Module Name: CU_DCDR
// Project Name: MCU!!!!!
// Target Devices: Basys3
// Tool Versions: x
// Description: x
// 
// Dependencies: x
// 
// Instantiation Template:
//
// CU_DCDR my_cu_dcdr(
//   .br_eq     (xxxx), 
//   .br_lt     (xxxx), 
//   .br_ltu    (xxxx),
//   .opcode    (xxxx),    
//   .func7     (xxxx),    
//   .func3     (xxxx),    
//   .ALU_FUN   (xxxx),
//   .PC_SEL    (xxxx),
//   .srcA_SEL  (xxxx),
//   .srcB_SEL  (xxxx), 
//   .RF_SEL    (xxxx)   );
//
// 
// Revision:
// Revision 1.00 - Created (02-01-2020) - from Paul, Joseph, & Celina
//          1.01 - (02-08-2020) - removed  else's; fixed assignments
//          1.02 - (02-25-2020) - made all assignments blocking
//          1.03 - (05-12-2020) - reduced func7 to one bit
//          1.04 - (05-31-2020) - removed misleading code
//          1.05 - (12-10-2020) - added comments
//          1.06 - (02-11-2021) - fixed formatting issues
//          1.07 - (12-26-2023) - changed signal names
//
// Additional Comments:
// 
///////////////////////////////////////////////////////////////////////////


module CU_DCDR(
   input br_eq, 
   input br_lt, 
   input br_ltu,
   input [6:0] opcode,   //-  ir[6:0]
   input func7,          //-  ir[30]
   input [2:0] func3,    //-  ir[14:12] 
   input int_taken,
   output logic [3:0] ALU_FUN,
   output logic [2:0] PC_SEL,
   output logic [1:0] srcA_SEL,
   output logic [2:0] srcB_SEL, 
   output logic [1:0] RF_SEL   );
    
   //- datatypes for RISC-V opcode types
   typedef enum logic [6:0] {
        LUI    = 7'b0110111,
        AUIPC  = 7'b0010111,
        JAL    = 7'b1101111,
        JALR   = 7'b1100111,
        BRANCH = 7'b1100011,
        LOAD   = 7'b0000011,
        STORE  = 7'b0100011,
        OP_IMM = 7'b0010011,
        OP_RG3 = 7'b0110011,
        OP_SYS = 7'b1110011
   } opcode_t;
   opcode_t OPCODE; //- define variable of new opcode type
    
   assign OPCODE = opcode_t'(opcode); //- Cast input enum 

   //- datatype for func3Symbols tied to values
   typedef enum logic [2:0] {
        //BRANCH labels
        BEQ = 3'b000,
        BNE = 3'b001,
        BLT = 3'b100,
        BGE = 3'b101,
        BLTU = 3'b110,
        BGEU = 3'b111
   } func3_t;    
   func3_t FUNC3; //- define variable of new opcode type
    
   assign FUNC3 = func3_t'(func3); //- Cast input enum 
       
    always_comb
    begin
        //- schedule all values to avoid latch
        PC_SEL = 3'b000;  srcB_SEL = 3'b000;     RF_SEL = 2'b00; 
        srcA_SEL = 2'b00;   ALU_FUN  = 4'b0000;
        
        case(OPCODE)
            LUI:
            begin
                ALU_FUN = 4'b1001;
                srcA_SEL = 2'b01;
                srcB_SEL = 3'b000;
                RF_SEL = 2'b11;
            end
            
            AUIPC:
            begin
                ALU_FUN = 4'b0000;
                srcA_SEL = 2'b01;
                srcB_SEL = 3'b011;
                RF_SEL = 2'b11;
            end
                
            JAL:
            begin
                PC_SEL = 3'b011;
                RF_SEL = 2'b00;
            end
            
            JALR:
            begin
                ALU_FUN = 4'b0000;
                srcA_SEL = 2'b00;
                srcB_SEL = 3'b001;
                PC_SEL = 3'b001;
                RF_SEL = 2'b00;
            end
                
            LOAD: 
            begin
                ALU_FUN = 4'b0000;
                srcA_SEL = 2'b00;
                srcB_SEL = 3'b001; 
                RF_SEL = 2'b10;
            end
                
            STORE:
            begin
                ALU_FUN = 4'b0000; 
                srcA_SEL = 2'b00;
                srcB_SEL = 3'b010;
            end
            
            BRANCH:
            begin
                case(func3)
                    3'b000: PC_SEL = {{1'b0}, {br_eq}, {1'b0}};
                    3'b001: PC_SEL = {{1'b0}, {~br_eq}, {1'b0}};
                    3'b100: PC_SEL = {{1'b0}, {br_lt}, {1'b0}};
                    3'b101: PC_SEL = {{1'b0}, {~br_lt}, {1'b0}};
                    3'b110: PC_SEL = {{1'b0}, {br_ltu}, {1'b0}};
                    3'b111: PC_SEL = {{1'b0}, {~br_ltu}, {1'b0}};
                    default: PC_SEL = 3'b000;
                endcase
            end
            
            OP_IMM:
            begin
                srcA_SEL = 2'b00; 
                srcB_SEL = 3'b001;
                RF_SEL = 2'b11;
                
                case(FUNC3)
                    3'b000: ALU_FUN = 4'b0000; // instr: ADDI
                    3'b010: ALU_FUN = 4'b0010; // instr: SLTI
                    3'b011: ALU_FUN = 4'b0011; // instr: SLTIU
                    3'b110: ALU_FUN = 4'b0110; // instr: ORI
                    3'b100: ALU_FUN = 4'b0100; // instr: XORI
                    3'b111: ALU_FUN = 4'b0111; // instr: ANDI
                    3'b001: ALU_FUN = 4'b0001; // instr: SLLI
                    3'b101: 
                    begin
                        case(func7)
                            1'b0: ALU_FUN = 4'b0101; // instr: SRLI
                            1'b1: ALU_FUN = 4'b1101; // instr: SRAI
                            default: ALU_FUN = 4'b0101;
                        endcase
                    end
                    default: ALU_FUN = 4'b0000;
                endcase
            end
    
            OP_RG3:
            begin
                srcA_SEL = 2'b00; 
                srcB_SEL = 3'b000;
                RF_SEL = 2'b11;
                
                case(FUNC3)
                    3'b000:
                    begin
                        case(func7)
                            1'b0: ALU_FUN = 4'b0000; // instr: ADD
                            1'b1: ALU_FUN = 4'b1000; // instr: SUB
                            default: ALU_FUN = 4'b0000;
                        endcase
                    end
                    3'b001: ALU_FUN = 4'b0001; // instr: SLL
                    3'b010: ALU_FUN = 4'b0010; // instr: SLT
                    3'b011: ALU_FUN = 4'b0011; // instr: SLTU
                    3'b100: ALU_FUN = 4'b0100; // instr: XOR
                    3'b101:
                    begin
                        case(func7)
                            1'b0: ALU_FUN = 4'b0101; // instr: SRL
                            1'b1: ALU_FUN = 4'b1101; // instr: SRA
                            default: ALU_FUN = 4'b0101;
                        endcase
                    end
                    3'b110: ALU_FUN = 4'b0110; // instr: OR
                    3'b111: ALU_FUN = 4'b0111; // instr: AND
                    default: ALU_FUN = 4'b0000;
                endcase
            end
            
            OP_SYS:
            begin
                case(func3)
                    3'b001: 
                    begin
                        RF_SEL = 2'b01; // instr: CSRRW
                        ALU_FUN = 4'b0000;
                        srcA_SEL = 2'b00;
                        srcB_SEL = 3'b100;
                    end
                    3'b011: 
                    begin
                        RF_SEL = 2'b01; // instr: CSRRC
                        ALU_FUN = 4'b0111;
                        srcA_SEL = 2'b10;
                        srcB_SEL = 3'b100;
                    end
                    3'b010:
                    begin
                        RF_SEL = 2'b01; // instr: CSRRS
                        ALU_FUN = 4'b0110;
                        srcA_SEL = 2'b00;
                        srcB_SEL = 3'b100;
                    end
                    3'b000: PC_SEL = 3'b101; // instr: MRET
                endcase
            end
    
            default: ALU_FUN = 4'b0000;
        endcase
        
        if(int_taken) PC_SEL = 3'b100; // interrupt
    end
endmodule