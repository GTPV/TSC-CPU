///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: ProgramCounter.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: PC module under cpu from "cpu.v"

// INCLUDES
`include "opcodes.v"

// MODULE DECLARATION
module PC(
    input clk,
    input reset_n,
    input stall,

    input Jump_Failed,
    input [`WORD_SIZE-1 : 0] Jump_Correct_PC,
    input Branch_Failed,
    input [`WORD_SIZE-1 : 0] Branch_Correct_PC,

    input [`WORD_SIZE-1 : 0] EX_PC,
    input [`inst_type_bitlen-1 : 0] EX_inst_type,
    input Branch_Taken_Result,

    input [`WORD_SIZE-1 : 0] ID_PC,
    input [`WORD_SIZE-1 : 0] ID_PC_next_seq,
    input [`WORD_SIZE-1 : 0] ID_Jump_Target,

    output reg [`WORD_SIZE-1 : 0] pc,
    output [`WORD_SIZE-1 : 0] pc_next_seq
);
    wire [`WORD_SIZE-1 : 0] new_pc;

    assign pc_next_seq = pc + `WORD_SIZE'b1;

    BranchPredictor BP_UUT(
        .clk (clk),
        .reset_n (reset_n),

        .pc (pc),
        .pc_next_seq (pc_next_seq),

        .ID_PC (ID_PC),
        .ID_PC_next_seq (ID_PC_next_seq),
        .ID_Jump_Target (ID_Jump_Target),

        .EX_PC (EX_PC),
        .EX_inst_type (EX_inst_type),
        .Branch_Taken_Result (Branch_Taken_Result),

        .new_pc (new_pc)
    );

    always @(posedge clk) begin
        if(reset_n == 1'b0) begin
            pc = 0;
        end else if(Branch_Failed) begin             // Branch_Failed -> override stall(must update to correct PC)
            pc = Branch_Correct_PC;
        end else if(Jump_Failed) begin      // Jump_Failed -> override stall(must update to correct PC)
            pc = Jump_Correct_PC;
        end else if(stall == 1'b0) begin    // no fail, no stall
            pc = new_pc;
        end else begin                      // no fail, stall
            pc = pc;
        end
    end
endmodule