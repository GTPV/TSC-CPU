///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: RegisterFile.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: RF module under DP from "Datapath.v"

// INCLUDE
`include "opcodes.v"

// DEFINE
`define NUM_REG (1<<`reg_addr_bitlen)

// MODULE DECLARATION
module RF(
    input clk,
    input reset_n,

    input[`reg_addr_bitlen-1 : 0] addr1,
    output reg [`WORD_SIZE-1 : 0] data1,

    input [`reg_addr_bitlen-1 : 0] addr2,
    output reg [`WORD_SIZE-1 : 0] data2,

    input write,
    input [`reg_addr_bitlen-1 : 0] addr3,
    input [`WORD_SIZE-1 : 0] data3
);
    reg [`WORD_SIZE-1 : 0] p_rf[`NUM_REG-1 : 0];

    integer addr_idx;

    always @(*) begin
        data1 = p_rf[addr1];
        data2 = p_rf[addr2];
    end

    wire CLK;
    assign CLK = (`DATA_FORWARDING_ENABLED == 1'b1 || `RF_SELF_FORWARDING == 1'b1) ? ~clk : clk;

    always @(posedge CLK) begin
        if(reset_n == 1'b0) begin
            // if reset -> reset all values
            for(addr_idx = 0; addr_idx < `NUM_REG; addr_idx = addr_idx+1) begin
                p_rf[addr_idx] = `WORD_SIZE'b0;
            end
        end else begin
            if(write) begin
                // write Register value
                p_rf[addr3] = data3;
            end
        end
    end
endmodule