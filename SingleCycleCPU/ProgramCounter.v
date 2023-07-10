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

    input Jump, // indicate if current instruction is Jump
    input [`Jump_target_length - 1 : 0] J_target, 

    output reg [`WORD_SIZE - 1 : 0] instruction_addr // memory address for next instruction
);

    always @(posedge clk) begin
        if(reset_n == 1'b0) begin // if reset_n is low -> reset
            instruction_addr = 0;
        end
        else if(Jump) begin // if current instruction is Jump -> make next addr by concat
            instruction_addr = {instruction_addr[`WORD_SIZE - 1 : `WORD_SIZE - 4], J_target};
        end
        else begin // next addr
            instruction_addr = instruction_addr + `WORD_SIZE'b1;
        end
    end

endmodule
///////////////////////////////////////////////////////////////////////////