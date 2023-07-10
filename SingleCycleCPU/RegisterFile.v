///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: RegisterFile.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: RF module under DP from "Datapath.v"

// INCLUDE
`include "opcodes.v"

`define NUM_REG (1<<`reg_addr_length)

// MODULE DECLARATION
module RF(
    input clk,
    input reset_n,

    input [`reg_addr_length-1 : 0] addr1,
    output reg [`WORD_SIZE-1 : 0] data1,

    input [`reg_addr_length-1 : 0] addr2,
    output reg [`WORD_SIZE-1 : 0] data2,

    input write,
    input [`reg_addr_length-1 : 0] addr3,
    input [`WORD_SIZE-1 : 0] data3
);

    reg [`WORD_SIZE-1 : 0] memory[`NUM_REG-1 : 0];

    always @(*) begin
        data1 = memory[addr1];
        data2 = memory[addr2];
    end

    integer addr_idx;
    always @(negedge reset_n or posedge clk) begin

        if(reset_n == 1'b0) begin
            for(addr_idx = 0; addr_idx < `NUM_REG; addr_idx = addr_idx + 1) begin
                memory[addr_idx] <= `WORD_SIZE'b0;
            end
        end

        else begin
            if(write) begin
                memory[addr3] <= data3;
            end
        end
    end
endmodule