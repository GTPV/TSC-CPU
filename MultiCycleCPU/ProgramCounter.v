///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: ProgramCounter.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: PC module under cpu from "cpu.v"

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION
module PC(
    input clk,
    input reset_n,
    input [`WORD_SIZE - 1 : 0] next_target,

    input PVSWrite,
    input PCWrite,
    input [1:0] PCSource,

    output [`WORD_SIZE - 1 : 0] next_Seq_addr,
    output reg [`WORD_SIZE - 1 : 0] inst_addr
);

    reg reset_cycle;

    reg [`WORD_SIZE - 1 : 0] next_Seq_addr; // next sequential PC
    reg [`WORD_SIZE - 1 : 0] next_Branch_addr; // next branch PC : only used when Branch taken

    always @(posedge clk) begin
        if(reset_n == 1'b0) begin
            reset_cycle = 1'b1;
            inst_addr = `WORD_SIZE'b0;
        end else if(reset_cycle) begin
            reset_cycle = 1'b0;
            inst_addr = `WORD_SIZE'b0;
        end else if(PVSWrite) begin // change PVS
            if(PCSource == `PCSrc_Seq) inst_addr = next_Seq_addr;
            else if(PCSource == `PCSrc_Offset) inst_addr = next_Branch_addr;
            else if(PCSource == `PCSrc_Uncond_Jump) inst_addr = next_target;
        end else if(PCWrite) begin // save next PC candidate
            if(PCSource == `PCSrc_Seq) next_Seq_addr = next_target; // save seq
            else if(PCSource == `PCSrc_Offset) next_Branch_addr = next_target; // save branch dest
        end
    end
endmodule