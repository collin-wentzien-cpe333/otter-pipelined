`timescale 1ns / 1ps

module tb_pipeline();

    reg CLK;
    reg RESET;
    reg INTR;
    reg [31:0] IOBUS_IN;
    wire [31:0] IOBUS_OUT;
    wire [31:0] IOBUS_ADDR;
    wire IOBUS_WR;

    OTTER_MCU uut (
        .CLK(CLK),
        .RESET(RESET),
        .INTR(INTR),
        .IOBUS_IN(IOBUS_IN),
        .IOBUS_OUT(IOBUS_OUT),
        .IOBUS_ADDR(IOBUS_ADDR),
        .IOBUS_WR(IOBUS_WR)
    );

    initial begin
        CLK = 0;
        forever #10 CLK = ~CLK;
    end

    initial begin
        RESET = 1;
        INTR = 0;
        IOBUS_IN = 32'b0;

        #50;
        RESET = 0;
        
    end

endmodule
