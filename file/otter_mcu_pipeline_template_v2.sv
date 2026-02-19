`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: CPE333
// Engineer: 
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
} instr_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);

    // IF stage signals
    wire [31:0] pc;
    wire [31:0] next_pc;
    wire [31:0] IR;
    wire [31:0] mem_data;
    wire pcWrite;
    wire memRead1, memRead2;
    wire memWrite;

    // ID stage signals
    wire [31:0] I_immed, S_immed, U_immed, B_immed, J_immed;

    // EX stage signals
    wire [31:0] aluResult;
    logic br_lt, br_eq, br_ltu;
    
    logic [31:0] rf_write_data;
    
    logic [31:0] ex_mem_rs2;
    logic [31:0] ex_mem_aluRes;
    
    // Forwarding select signals
    logic [1:0] forwardA, forwardB;

    //other
    logic branch_taken = 0;
    logic [31:0] branch_pc;
              
//==== Instruction Fetch ===========================================
 
    logic [31:0] if_de_pc;
    logic [31:0] if_de_ir;
    
    always_ff @(posedge CLK) begin
        if(RESET || branch_taken) begin
            if_de_pc <= 0;
            if_de_ir <= 32'h00000013; //nop
        end else begin
            if_de_pc <= pc;
            if_de_ir <= IR;
        end
    end
    
    assign pcWrite = 1'b1; 	//Hardwired high, assuming now hazards
    assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
    
    reg_nb #(.n(32)) PC_REG (
        .data_in(next_pc),
        .clk(CLK),
        .clr(RESET),
        .ld(pcWrite),
        .data_out(pc)
    );
    
  assign next_pc = RESET ? 32'b0 : (branch_taken ? branch_pc : pc + 4);
        
    Memory OTTER_MEMORY (
        .MEM_CLK(CLK),
        .MEM_RDEN1(memRead1),
        .MEM_RDEN2(de_ex_inst.memRead2),
        .MEM_WE2(memWrite),
        .MEM_ADDR1(pc[15:2]),
        .MEM_ADDR2(ex_mem_aluRes),
        .MEM_ADDR2_EARLY(aluResult),
        .MEM_DIN2(ex_mem_rs2),
        .MEM_SIZE(ex_mem_inst.mem_type[1:0]),
        .MEM_SIGN(ex_mem_inst.mem_type[2]),
        .IO_IN(IOBUS_IN),
        .IO_WR(IOBUS_WR),
        .MEM_DOUT1(IR),
        .MEM_DOUT2(mem_data)
    );
        
     
//==== Instruction Decode ===========================================
    logic [31:0] de_ex_opA;
    logic [31:0] de_ex_opB;
    logic [31:0] de_ex_rs2;
    
    logic [31:0] de_ex_jal_target, de_ex_jalr_target, de_ex_branch_target;

    logic[31:0] rs1_data, rs2_data;
    
    instr_t de_ex_inst, de_inst;
    
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(if_de_ir[6:0]); //OPCODE_t, opcode
    
    assign de_inst.rs1_addr = if_de_ir[19:15]; //IR
    assign de_inst.rs2_addr = if_de_ir[24:20]; //IR
    assign de_inst.rd_addr = if_de_ir[11:7]; //IR
    assign de_inst.opcode = OPCODE;
    
    assign de_inst.pc = if_de_pc;
    
    assign de_inst.rs1_used =   de_inst.rs1_addr != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;
                                
    assign de_inst.rs2_used =    de_inst.opcode == OP
                                || de_inst.opcode == STORE
                                || de_inst.opcode == BRANCH;
                                
    assign de_inst.rd_used = de_inst.opcode != STORE && de_inst.opcode != BRANCH;
    assign de_inst.regWrite = de_inst.rd_used && de_inst.rd_addr != 0;
    assign de_inst.memWrite = de_inst.opcode == STORE;
    assign de_inst.memRead2 = de_inst.opcode == LOAD;
    assign de_inst.mem_type = if_de_ir[14:12]; //func3
    
    RegFile REG_FILE (
        .clk(CLK),
        .en(mem_wb_inst.regWrite),
        .adr1(if_de_ir[19:15]), // rs1 address
        .adr2(if_de_ir[24:20]), // rs2 address
        .w_adr(mem_wb_inst.rd_addr),
        .w_data(rf_write_data),
        .rs1(rs1_data),
        .rs2(rs2_data)
    );
    
    immed_gen IMMED_GEN (
        .ir(if_de_ir),
        .i(I_immed),
        .s(S_immed),
        .b(B_immed),
        .u(U_immed),
        .j(J_immed)
    );
    
    logic [31:0] alu_opA;
    always_comb begin
        case (de_inst.opcode)
            LUI:     alu_opA = U_immed; // srcA_SEL = 01
            AUIPC:   alu_opA = U_immed; // srcA_SEL = 01
            default: alu_opA = rs1_data; // srcA_SEL = 00
        endcase
    end
    
    logic [31:0] alu_opB;
    always_comb begin
        case (de_inst.opcode)
            OP:      alu_opB = rs2_data; // srcB_SEL = 000
            STORE:   alu_opB = S_immed; // srcB_SEL = 010
            AUIPC:   alu_opB = de_inst.pc; // srcB_SEL = 011
            default: alu_opB = I_immed; // srcB_SEL = 001
        endcase
    end
    
    logic [31:0] de_jal_target, de_jalr_target, de_branch_target;
    assign de_jal_target    = if_de_pc + J_immed;
    assign de_branch_target = if_de_pc + B_immed;
    assign de_jalr_target   = rs1_data + I_immed;
    
    assign de_inst.rf_wr_sel =   (de_inst.opcode == JAL || de_inst.opcode == JALR) ? 2'b00 :  // PC+4
                                 (de_inst.opcode == LOAD) ? 2'b10 :  // mem_data
                                 2'b11;  // ALU result
    
    logic [3:0] alu_fun_dec;
    always_comb begin
        case (de_inst.opcode)
            LUI:     alu_fun_dec = 4'b1001;
            AUIPC:   alu_fun_dec = 4'b0000;  // ADD
            JAL:     alu_fun_dec = 4'b0000;
            JALR:    alu_fun_dec = 4'b0000;  // ADD
            LOAD:    alu_fun_dec = 4'b0000;  // ADD
            STORE:   alu_fun_dec = 4'b0000;  // ADD
            BRANCH:  alu_fun_dec = 4'b0000;
            OP_IMM: begin
                case (if_de_ir[14:12])  // func3
                    3'b000: alu_fun_dec = 4'b0000;  // ADDI
                    3'b010: alu_fun_dec = 4'b0010;  // SLTI
                    3'b011: alu_fun_dec = 4'b0011;  // SLTIU
                    3'b110: alu_fun_dec = 4'b0110;  // ORI
                    3'b100: alu_fun_dec = 4'b0100;  // XORI
                    3'b111: alu_fun_dec = 4'b0111;  // ANDI
                    3'b001: alu_fun_dec = 4'b0001;  // SLLI
                    3'b101: alu_fun_dec = if_de_ir[30] ? 4'b1101 : 4'b0101;  // SRAI/SRLI
                    default: alu_fun_dec = 4'b0000;
                endcase
            end
            OP: begin  // R-type
                case (if_de_ir[14:12])  // func3
                    3'b000: alu_fun_dec = if_de_ir[30] ? 4'b1000 : 4'b0000;  // SUB/ADD
                    3'b001: alu_fun_dec = 4'b0001;  // SLL
                    3'b010: alu_fun_dec = 4'b0010;  // SLT
                    3'b011: alu_fun_dec = 4'b0011;  // SLTU
                    3'b100: alu_fun_dec = 4'b0100;  // XOR
                    3'b101: alu_fun_dec = if_de_ir[30] ? 4'b1101 : 4'b0101;  // SRA/SRL
                    3'b110: alu_fun_dec = 4'b0110;  // OR
                    3'b111: alu_fun_dec = 4'b0111;  // AND
                    default: alu_fun_dec = 4'b0000;
                endcase
            end
            default: alu_fun_dec = 4'b0000;
        endcase
    end
    
    assign de_inst.alu_fun = alu_fun_dec;
    
    always_ff @(posedge CLK) begin
        if (RESET || branch_taken) begin
            de_ex_inst <= 0;
            de_ex_opA  <= 32'b0;
            de_ex_opB  <= 32'b0;
            de_ex_rs2  <= 32'b0;
            de_ex_jal_target <= 32'b0;
            de_ex_jalr_target <= 32'b0;
            de_ex_branch_target <= 32'b0;
        end else begin
            de_ex_inst <= de_inst;
            de_ex_opA  <= alu_opA;
            de_ex_opB  <= alu_opB;
            de_ex_rs2  <= rs2_data;
            de_ex_jal_target <= de_jal_target;
            de_ex_jalr_target <= de_jalr_target;
            de_ex_branch_target <= de_branch_target;
        end
    end
	
	
//==== Forwarding Detection =========================================

    // ForwardA (rs1)
    always_comb begin
        if (ex_mem_inst.regWrite
            && ex_mem_inst.rd_addr == de_ex_inst.rs1_addr
            && de_ex_inst.rs1_used)
            forwardA = 2'b01;  // forward from EX/MEM
        else if (mem_wb_inst.regWrite
            && mem_wb_inst.rd_addr == de_ex_inst.rs1_addr
            && de_ex_inst.rs1_used)
            forwardA = 2'b10;  // forward from MEM/WB
        else
            forwardA = 2'b00;  // no forward
    end

    // ForwardB (rs2)
    always_comb begin
        if (ex_mem_inst.regWrite
            && ex_mem_inst.rd_addr == de_ex_inst.rs2_addr
            && de_ex_inst.rs2_used)
            forwardB = 2'b01;  // forward from EX/MEM
        else if (mem_wb_inst.regWrite
            && mem_wb_inst.rd_addr == de_ex_inst.rs2_addr
            && de_ex_inst.rs2_used)
            forwardB = 2'b10;  // forward from MEM/WB
        else
            forwardB = 2'b00;  // no forward
    end

//==== Execute ======================================================

    instr_t ex_mem_inst;

    logic [31:0] ex_opA, ex_rs2, ex_aluB;

    // ForwardA mux — selects rs1/opA source
    always_comb begin
        case (forwardA)
            2'b01:   ex_opA = ex_mem_aluRes;   // from EX/MEM (previous instruction)
            2'b10:   ex_opA = rf_write_data;    // from MEM/WB (two instructions back)
            default: ex_opA = de_ex_opA;        // no forward
        endcase
    end

    // ForwardB mux — selects rs2 source
    always_comb begin
        case (forwardB)
            2'b01:   ex_rs2 = ex_mem_aluRes;
            2'b10:   ex_rs2 = rf_write_data;
            default: ex_rs2 = de_ex_rs2;
        endcase
    end

    // ALU B input: use forwarded rs2 for R-type (OP), otherwise keep de_ex_opB (has immediate)
    assign ex_aluB = (de_ex_inst.opcode == OP) ? ex_rs2 : de_ex_opB;

    ALU ALU_UNIT (
        .a(ex_opA),
        .b(ex_aluB),
        .ALU_FUN(de_ex_inst.alu_fun),
        .c(aluResult)
    );

    branch_cond_gen BRANCH_COND (
        .rs1(ex_opA),
        .rs2(ex_rs2),
        .br_eq(br_eq),
        .br_lt(br_lt),
        .br_ltu(br_ltu)
    );
    
    always_comb begin
        case (de_ex_inst.opcode)
            JAL:     branch_pc = de_ex_jal_target;
            JALR:    branch_pc = ex_opA + de_ex_opB; // forwarded rs1 + I_immed
            BRANCH:  branch_pc = de_ex_branch_target;
            default: branch_pc = de_ex_inst.pc + 4;
        endcase
    end
        
    always_comb begin
        case (de_ex_inst.opcode)
            JAL:     branch_taken = 1'b1;
            JALR:    branch_taken = 1'b1;
            BRANCH: begin
                case (de_ex_inst.mem_type)  // func3 stored in mem_type
                    3'b000: branch_taken = br_eq;   // BEQ
                    3'b001: branch_taken = ~br_eq;  // BNE
                    3'b100: branch_taken = br_lt;   // BLT
                    3'b101: branch_taken = ~br_lt;  // BGE
                    3'b110: branch_taken = br_ltu;  // BLTU
                    3'b111: branch_taken = ~br_ltu; // BGEU
                    default: branch_taken = 1'b0;
                endcase
            end
            default: branch_taken = 1'b0;
        endcase
    end
    
    always_ff @(posedge CLK) begin
        if (RESET) begin
            ex_mem_inst   <= '0;
            ex_mem_aluRes <= 32'b0;
            ex_mem_rs2    <= 32'b0;
        end else begin
            ex_mem_inst   <= de_ex_inst;
            ex_mem_aluRes <= aluResult;
            ex_mem_rs2    <= ex_rs2;
        end
    end

//==== Memory ======================================================
 
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    
    assign memRead2 = ex_mem_inst.memRead2;
    assign memWrite = ex_mem_inst.memWrite;
    
    logic [31:0] mem_wb_aluRes;
    logic [31:0] mem_wb_mem_data;
    instr_t mem_wb_inst;
    
    always_ff @(posedge CLK) begin
        if (RESET) begin
            mem_wb_inst     <= '0;
            mem_wb_aluRes   <= 32'b0;
            mem_wb_mem_data <= 32'b0;
        end else begin
            mem_wb_inst     <= ex_mem_inst;
            mem_wb_aluRes   <= ex_mem_aluRes;
            mem_wb_mem_data <= mem_data;
        end
    end
    
//==== Write Back ==================================================
     
    always_comb begin
        case (mem_wb_inst.rf_wr_sel)
            2'b00:   rf_write_data = mem_wb_inst.pc + 4; // JAL/JALR
            2'b10:   rf_write_data = mem_wb_mem_data; // LOAD memory data
            default: rf_write_data = mem_wb_aluRes; // ALU result
        endcase
    end
            
endmodule
