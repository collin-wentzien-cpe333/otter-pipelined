`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: nuh uh
// Engineer: Collin Wentzien
// 
// Create Date: 04/21/2025 04:59:13 PM
// Design Name: Program Counter
// Module Name: PC
// Project Name: PC
// Target Devices: Basys3
// Tool Versions: x
// Description: Program counter with four inputs
// 
// Dependencies: x
// 
// Revision: 1
// Revision 0.01 - File Created
// Additional Comments: no.
// 
//////////////////////////////////////////////////////////////////////////////////



module PC (
    input RST, intr, CLK,
    input [31:0] iobus_in,
    output [31:0] iobus_out, iobus_addr,
    output iobus_wr
    );
    
    //io
    wire [31:0] iobus_in_wire, iobus_out_wire, iobus_addr_wire;
    wire iobus_wr_wire;
    assign iobus_in_wire = iobus_in;
    assign iobus_out = iobus_out_wire;
    assign iobus_addr = iobus_addr_wire;
    assign iobus_wr = iobus_wr_wire;
    
    //pc
    wire [31:0] pc_in, pc_out;
    
    //memory
    wire [31:0] ir, d_out;
    
    //immed & branch gen
    wire [31:0] u, i, s, j, b;
    wire [31:0] jal, jalr, branch;
    
    //reg file
    wire [31:0] rs1, w_data; //rs2 = iobus_out
    
    //alu
    wire [1:0] srcA_SEL;
    wire [2:0] srcB_SEL;
    wire [3:0] ALU_FUN;
    wire [31:0] ALU_srcA, ALU_srcB;
    
    //cu fsm
    wire PC_WE, RF_WE, memWE2, memRDEN1, memRDEN2, reset, int_taken, mret_exec;
    
    //cu_dcdr
    wire [2:0] PC_SEL;
    wire [1:0] RF_SEL;
    
    //branch cond gen
    wire br_eq, br_lt, br_ltu;
    
    //csr
    wire csr_WE, mstatus;
    wire [31:0] mepc, mtvec, csr_RD;
    
    // -- PROGRAM COUNTER -- //
    assign pc_in = (PC_SEL == 3'b000) ? pc_out + 4 :
                   (PC_SEL == 3'b001) ? jalr :
                   (PC_SEL == 3'b010) ? branch :
                   (PC_SEL == 3'b011) ? jal :
                   (PC_SEL == 3'b100) ? mtvec :
                   (PC_SEL == 3'b101) ? mepc :
                                        32'b0;
    reg_nb #(.n(32)) register (
        .data_in  (pc_in), 
        .ld       (PC_WE), 
        .clk      (CLK), 
        .clr      (reset), 
        .data_out (pc_out)
     );  
    
    // -- MEMORY -- //
    Memory OTTER_MEMORY (
        .MEM_CLK (CLK),
        .MEM_RDEN1 (memRDEN1),
        .MEM_RDEN2 (memRDEN2),
        .MEM_WE2 (memWE2),
        .MEM_ADDR1 (pc_out[15:2]),
        .MEM_ADDR2 (iobus_addr_wire),
        .MEM_DIN2 (iobus_out_wire),
        .MEM_SIZE (ir[13:12]),
        .MEM_SIGN (ir[14]),
        .IO_IN (iobus_in_wire),
        .IO_WR (iobus_wr_wire),
        .MEM_DOUT1 (ir),
        .MEM_DOUT2 (d_out)
    ); 
    
    // -- IMMEDIATE GEN -- //
    immed_gen immedgen (
        .ir(ir),
        .i(i),
        .s(s),
        .b(b),
        .u(u),
        .j(j)
    );
    
    // -- BRANCH ADDR GEN -- //
    branch_addr_gen branchgen (
        .PC(pc_out),
        .j(j),
        .b(b),
        .i(i),
        .rs(rs1),
        .jal(jal),
        .branch(branch),
        .jalr(jalr)
    );
    
    // -- REG FILE -- //
    assign w_data = (RF_SEL == 2'b00) ? pc_out + 4 :
                    (RF_SEL == 2'b01) ? 32'b0 :
                    (RF_SEL == 2'b10) ? d_out :
                                        iobus_addr_wire;
    RegFile reg_file (
        .w_data(w_data),
        .clk(CLK),
        .en(RF_WE),
        .adr1(ir[19:15]),
        .adr2(ir[24:20]),
        .w_adr(ir[11:7]),
        .rs1(rs1),
        .rs2(iobus_out_wire)
    );
    
    // -- ALU -- //
    assign ALU_srcA = (srcA_SEL == 2'b00) ? rs1 :
                      (srcA_SEL == 2'b01) ? u :
                      (srcA_SEL == 2'b10) ? ~rs1 :
                                            32'b0;
    assign ALU_srcB = (srcB_SEL == 3'b000) ? iobus_out_wire :
                      (srcB_SEL == 3'b001) ? i :
                      (srcB_SEL == 3'b010) ? s :
                      (srcB_SEL == 3'b011) ? pc_out :
                      (srcB_SEL == 3'b100) ? csr_RD :
                                             32'b0;
    ALU alu (
        .a(ALU_srcA),
        .b(ALU_srcB),
        .ALU_FUN(ALU_FUN),
        .c(iobus_addr_wire)
    );
    
    // -- BRANCH COND GEN -- //
    branch_cond_gen br_cond (
        .rs1(rs1),
        .rs2(iobus_out_wire),
        .br_eq(br_eq),
        .br_lt(br_lt),
        .br_ltu(br_ltu) 
   );
    
    // -- CU_FSM -- //
    assign intr_in = intr && mstatus;
    
    CU_FSM cu_fsm (
        .intr(intr_in),
        .clk(CLK),
        .RST(RST),
        .opcode(ir[6:0]),
        .PC_WE(PC_WE),
        .RF_WE(RF_WE),
        .memWE2(memWE2),
        .memRDEN1(memRDEN1),
        .memRDEN2(memRDEN2),
        .csr_WE(csr_WE),
        .int_taken(int_taken),
        .mret_exec(mret_exec),
        .reset(reset)
    );
    
    // -- CU_DCDR -- //
    CU_DCDR cu_dec (
        .br_eq(br_eq),
        .br_lt(br_lt),
        .br_ltu(br_ltu),
        .opcode(ir[6:0]),
        .func7(ir[30]),
        .func3(ir[14:12]),
        .int_taken(int_taken),
        .ALU_FUN(ALU_FUN),
        .PC_SEL(PC_SEL),
        .srcA_SEL(srcA_SEL),
        .srcB_SEL(srcB_SEL),
        .RF_SEL(RF_SEL)
    );
    
    // -- CSR -- //
    CSR csr (
        .CLK(CLK),
        .RST(reset),
        .MRET_EXEC(mret_exec),
        .INT_TAKEN(int_taken),
        .ADDR(ir[31:20]),
        .PC(pc_out),
        .WD(iobus_addr_wire),
        .WR_EN(csr_WE),
        .RD(csr_RD),
        .CSR_MEPC(mepc),
        .CSR_MTVEC(mtvec),
        .CSR_MSTATUS_MIE(mstatus)
    );
    
endmodule
