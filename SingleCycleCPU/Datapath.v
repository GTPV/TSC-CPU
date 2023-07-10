///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: Datapath.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: DP module under cpu from "cpu.v"
// Submodule :  "RegisterFile.v" RF RF_UUT,
//              "ALUcombinational.v" ALU ALU_UUT

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION
module DP(
    input clk,
    input reset_n,

    output [`WORD_SIZE - 1 : 0] output_data,

    // input from Control
    input RegDst,
    input RegWrite,
    input ALUSrc,
    input [3:0] ALUOp,
    input Jump,
    input isWWD,

    input [`WORD_SIZE - 1 : 0] instruction
);
    // parsed instructions
    wire [`opcode_length - 1:0] opcode;
    wire [`reg_addr_length - 1 : 0] rs, rt, rd;
    wire [`WORD_SIZE - 1 : 0] I_immediate;
    wire [`Jump_target_length - 1 : 0] J_target;
    wire [`func_code_length - 1 : 0] func_code;

    // RegisterFile module ports
    wire [`reg_addr_length - 1 : 0] write_addr;
    wire [`WORD_SIZE - 1 : 0] RF_rs;
    wire [`WORD_SIZE - 1 : 0] RF_rd;
    wire [`WORD_SIZE - 1 : 0] RF_rt;
    wire [`WORD_SIZE - 1 : 0] RF_write;

    // ALU module ports
    wire [`WORD_SIZE - 1 : 0] ALU_B;
    wire [`WORD_SIZE - 1 : 0] ALU_C;
    wire ALU_Cout;

    // parse instruction
    assign opcode = instruction[`opcode_left : `opcode_right];
    assign func_code = instruction[`func_code_left : `func_code_right];
    assign rs = instruction[`rs_left : `rs_right];
    assign rt = instruction[`rt_left : `rt_right];
    assign rd = instruction[`rd_left : `rd_right];
    assign I_immediate = { {8{instruction[`immediate_left]}}, instruction[`immediate_left : `immediate_right]}; //Sign-extend
    assign J_target = instruction[`target_left : `target_right];

    // mux selection by control signal
    assign write_addr = (RegDst) ? rd : rt;
    assign ALU_B = (ALUSrc) ? I_immediate : RF_rt;
    assign RF_write = ALU_C;

    // output data
    assign output_data = ALU_C;

    // RegisterFile module
    RF RF_UUT(
        .clk(clk),
        .reset_n(reset_n),

        .addr1(rs),
        .data1(RF_rs),

        .addr2(rt),
        .data2(RF_rt),

        .write(RegWrite),
        .addr3(write_addr),
        .data3(RF_write)
    );

    // ALU module
    ALU ALU_UUT(
        .A(RF_rs),
        .B(ALU_B),
        .Cin(1'b0),

        .OP(ALUOp),

        .C(ALU_C),
        .Cout(ALU_Cout)
    );
endmodule
///////////////////////////////////////////////////////////////////////////