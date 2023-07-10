///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: PipelineDataRegister.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: 

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pipeline Data Registers - IFID, IDEX, EXMEM, MEMWB

// IF-ID Latching Register---------------------------------------------------------------------------------------------//
module IFID(
    input clk,
    input reset_n,

    // input signals of PC address and inst
    input [`WORD_SIZE-1 : 0] PC_in, output reg [`WORD_SIZE-1 : 0] PC_out,
    input [`WORD_SIZE-1 : 0] PC_next_seq_in, output reg [`WORD_SIZE-1 : 0] PC_next_seq_out,
    input [`WORD_SIZE-1 : 0] inst_in, output reg [`WORD_SIZE-1 : 0] inst_out,
    //input [`WORD_SIZE-1 : 0] inst_num_in, output reg [`WORD_SIZE-1 : 0] inst_num_out,

    // stall, flush signals
    input stall,
    input flush
);
    always @(posedge clk) begin
        if(reset_n == 1'b0 || flush == 1'b1) begin // if reset or flushed
            PC_out = 0;
            PC_next_seq_out = 0;
            inst_out = `INST_FLUSHED;
            //inst_num_out = 0;
        end else if(stall == 1'b0) begin
            PC_out = PC_in;
            PC_next_seq_out = PC_next_seq_in;
            inst_out = inst_in;
            //inst_num_out = inst_num_in;
        end
    end
endmodule
// ID-EX Latching Register---------------------------------------------------------------------------------------------//
module IDEX(
    input clk,
    input reset_n,

    // input signals of PC address and inst
    input [`WORD_SIZE-1 : 0] PC_in, output reg [`WORD_SIZE-1 : 0] PC_out,
    input [`WORD_SIZE-1 : 0] PC_next_seq_in, output reg [`WORD_SIZE-1 : 0] PC_next_seq_out,
    input [`WORD_SIZE-1 : 0] inst_in, output reg [`WORD_SIZE-1 : 0] inst_out,
    input [`inst_type_bitlen-1 : 0] inst_type_in, output reg [`inst_type_bitlen-1 : 0] inst_type_out,
    //input [`WORD_SIZE-1 : 0] inst_num_in, output reg [`WORD_SIZE-1 : 0] inst_num_out,

    // RegisterFile values read(or forwarded)
    input [`WORD_SIZE-1 : 0] rs_value_in, output reg [`WORD_SIZE-1 : 0] rs_value_out,
    input [`WORD_SIZE-1 : 0] rt_value_in, output reg [`WORD_SIZE-1 : 0] rt_value_out,

    // Immediate value : sign-extended
    input [`WORD_SIZE-1 : 0] I_imm_in, output reg [`WORD_SIZE-1 : 0] I_imm_out,

    // Possible not sequential address
    // I_Branch - Branch Taken addr, J_Jump - concat addr, R_JumpReg - RF value
    input [`WORD_SIZE-1 : 0] Jumped_PC_in, output reg [`WORD_SIZE-1 : 0] Jumped_PC_out,

    // stall, flush signals
    input stall,
    input flush
);
    always @(posedge clk) begin
        if(reset_n == 1'b0 || flush == 1'b1) begin // if reset or flushed
            PC_out = 0;
            PC_next_seq_out = 0;
            inst_out = `INST_FLUSHED;
            inst_type_out = 0;
            //inst_num_out = 0;
            Jumped_PC_out = 0;
            rs_value_out = 0;
            rt_value_out = 0;
            I_imm_out = 0;
        end else if(stall == 1'b0) begin
            PC_out = PC_in;
            PC_next_seq_out = PC_next_seq_in;
            inst_out = inst_in;
            inst_type_out = inst_type_in;
            //inst_num_out = inst_num_in;
            Jumped_PC_out = Jumped_PC_in;
            rs_value_out = rs_value_in;
            rt_value_out = rt_value_in;
            I_imm_out = I_imm_in;
        end
    end
endmodule
// EX-MEM Latching Register---------------------------------------------------------------------------------------------//
module EXMEM(
    input clk,
    input reset_n,

    // input signals of PC address and inst
    input [`WORD_SIZE-1 : 0] PC_in, output reg [`WORD_SIZE-1 : 0] PC_out,
    input [`WORD_SIZE-1 : 0] PC_next_seq_in, output reg [`WORD_SIZE-1 : 0] PC_next_seq_out,
    input [`WORD_SIZE-1 : 0] inst_in, output reg [`WORD_SIZE-1 : 0] inst_out,
    input [`inst_type_bitlen-1 : 0] inst_type_in, output reg [`inst_type_bitlen-1 : 0] inst_type_out,
    //input [`WORD_SIZE-1 : 0] inst_num_in, output reg [`WORD_SIZE-1 : 0] inst_num_out,

    // RegisterFile values read(or forwarded)
    input [`WORD_SIZE-1 : 0] rt_value_in, output reg [`WORD_SIZE-1 : 0] rt_value_out,

    // Result of ALU from EX stage
    input [`WORD_SIZE-1 : 0] ALU_result_in, output reg [`WORD_SIZE-1 : 0] ALU_result_out,

    // stall, flush signals
    input stall,
    input flush
);
    always @(posedge clk) begin
        if(reset_n == 1'b0 || flush == 1'b1) begin // if reset or flushed
            PC_out = 0;
            PC_next_seq_out = 0;
            inst_out = `INST_FLUSHED;
            inst_type_out = 0;
            //inst_num_out = 0;
            rt_value_out = 0;
            ALU_result_out = 0;
        end else if(stall == 1'b0) begin
            PC_out = PC_in;
            PC_next_seq_out = PC_next_seq_in;
            inst_out = inst_in;
            inst_type_out = inst_type_in;
            //inst_num_out = inst_num_in;
            rt_value_out = rt_value_in;
            ALU_result_out = ALU_result_in;
        end
    end
endmodule
// MEM-WB Latching Register---------------------------------------------------------------------------------------------//
module MEMWB(
    input clk,
    input reset_n,

    // input signals of PC address and inst
    input [`WORD_SIZE-1 : 0] PC_in, output reg [`WORD_SIZE-1 : 0] PC_out,
    input [`WORD_SIZE-1 : 0] PC_next_seq_in, output reg [`WORD_SIZE-1 : 0] PC_next_seq_out,
    //input [`WORD_SIZE-1 : 0] inst_num_in, output reg [`WORD_SIZE-1 : 0] inst_num_out,
    input [`WORD_SIZE-1 : 0] inst_in, output reg [`WORD_SIZE-1 : 0] inst_out,
    input [`inst_type_bitlen-1 : 0] inst_type_in, output reg [`inst_type_bitlen-1 : 0] inst_type_out,

    input [`WORD_SIZE-1 : 0] ALU_result_in, output reg [`WORD_SIZE-1 : 0] ALU_result_out,

    input [`WORD_SIZE-1 : 0] MemData_in, output reg [`WORD_SIZE-1 : 0] MemData_out,

    // stall, flush signals
    input stall,
    input flush
);
    always @(posedge clk) begin
        if(reset_n == 1'b0 || flush == 1'b1) begin // if reset or flushed
            PC_out = 0;
            PC_next_seq_out = 0;
            inst_out = `INST_FLUSHED;
            inst_type_out = 0;
            //inst_num_out = 0;
            ALU_result_out = 0;
            MemData_out = 0;
        end else if(stall == 1'b0) begin
            PC_out = PC_in;
            PC_next_seq_out = PC_next_seq_in;
            inst_out = inst_in;
            inst_type_out = inst_type_in;
            //inst_num_out = inst_num_in;
            ALU_result_out = ALU_result_in;
            MemData_out = MemData_in;
        end
    end
endmodule

