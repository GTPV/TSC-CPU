///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: Datapath.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: Datapath module under cpu from "cpu.v"

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION
module DP(
    input clk,
    input reset_n,
    input [`WORD_SIZE - 1 : 0] inst_addr,
    input [`WORD_SIZE - 1 : 0] inst_seq_addr,
    input [`WORD_SIZE - 1 : 0] instruction,
    input IRWrite,
    input [`WORD_SIZE - 1 : 0] MemData,

    input [1:0] RegWriteSrc,
    input RegWrite,
    input [1:0] RegDst,
    input [3:0] ALUOp,
    input [1:0] ALUSrcA,
    input [1:0] ALUSrcB,

    output [`WORD_SIZE - 1 : 0] output_data,
    output [`WORD_SIZE - 1 : 0] RF_addr,
    output [`WORD_SIZE - 1 : 0] RF_data,
    output [`WORD_SIZE - 1 : 0] ALU_out_C,
    output [`WORD_SIZE - 1 : 0] ALUOut,
    output [1:0] ALU_Cmp
);

    // reg to latch data
    reg [`WORD_SIZE - 1 : 0] IR, MDR, ALUOut, A, B;
    // latch data when posedge clk
    always @(posedge clk) begin
        MDR = MemData;
        ALUOut = ALU_out_C;
        A = RF_rs;
        B = RF_rt;
        if(IRWrite) begin
            IR = instruction;
        end
    end

    // parsed instructions
    wire [`opcode_bitlen - 1 : 0] opcode;
    wire [`reg_addr_bitlen - 1 : 0] rs, rt, rd;
    wire [`R_func_bitlen - 1 : 0] R_func_code;
    wire [`WORD_SIZE - 1 : 0] I_imm_off;
    wire [`J_target_addr_bitlen - 1 : 0] J_target_addr;
    // parse instructions
    assign opcode = IR[`opcode_idx_left : `opcode_idx_right];
    assign rs = IR[`rs_idx_left : `rs_idx_right];
    assign rt = IR[`rt_idx_left : `rt_idx_right];
    assign rd = IR[`rd_idx_left : `rd_idx_right];
    assign R_func_code = IR[`R_func_idx_left : `R_func_idx_right];
    assign I_imm_off = {{8{IR[`imm_off_idx_left]}}, IR[`imm_off_idx_left : `imm_off_idx_right]}; // sign-extend
    assign J_target_addr = IR[`target_addr_idx_left : `target_addr_idx_right];


    // RF ports
    // input ports
    wire [`reg_addr_bitlen - 1 : 0] write_addr;
    wire [`WORD_SIZE - 1 : 0] RF_write;
    // output ports
    wire [`WORD_SIZE - 1 : 0] RF_rs, RF_rt;
    // RF inout selection
    assign write_addr = (RegDst == `RegDst_rd) ? rd :
                        (RegDst == `RegDst_rt) ? rt : 
                        (RegDst == `RegDst_2) ? 2'b10 : 2'bx;
    assign RF_write = ( (RegWriteSrc == `RegWriteSrc_ALU) ? ALUOut :
                        (RegWriteSrc == `RegWriteSrc_MEM) ? MDR :
                        (RegWriteSrc == `RegWriteSrc_PC)  ? inst_seq_addr : `WORD_SIZE'bx);

    // ALU ports
    wire [`WORD_SIZE - 1 : 0] ALU_in_A, ALU_in_B; // input
    // ALU input selection
    assign ALU_in_A = ( (ALUSrcA == `ALUSrcA_RF_rs) ? A :
                        (ALUSrcA == `ALUSrcA_PC) ? inst_addr :
                        (ALUSrcA == `ALUSrcA_Seq_PC) ? inst_seq_addr : `WORD_SIZE'bz);
    assign ALU_in_B = ( (ALUSrcB == `ALUSrcB_RF_rt) ? B :
                        (ALUSrcB == `ALUSrcB_Seq) ? `WORD_SIZE'd1 : 
                        (ALUSrcB == `ALUSrcB_I_OFFSET) ? I_imm_off: `WORD_SIZE'b0);
    wire ALU_Cout;
    wire [1:0] ALU_Cmp; // output

    // output assignments
    assign output_data = RF_rs;
    assign RF_addr = RF_rs; // for JRL, JPR
    assign RF_data = RF_rt; // for SWD


    // RF unit
    RF RF_UUT(
        .clk (clk),
        .reset_n (reset_n),

        .addr1 (rs),
        .data1 (RF_rs),

        .addr2 (rt),
        .data2 (RF_rt),

        .write (RegWrite),
        .addr3 (write_addr),
        .data3 (RF_write)
    );

    // ALU unit
    ALU ALU_UUT(
        .A (ALU_in_A),
        .B (ALU_in_B),
        .Cin (1'b0),
        .OP (ALUOp),
        .C (ALU_out_C),
        .Cout (ALU_Cout),
        .Cmp (ALU_Cmp)
    );
endmodule