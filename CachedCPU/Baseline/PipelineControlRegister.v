///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: PipelineControlRegister.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: 

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control Signal Registers - EX_Control, MEM_Control, WB_Control - Decoded & Generated once at ID, used later
// IDEX :   EX, MEM, WB - EX signals are consumed
// EXMEM :      MEM, WB - MEM signals are consumed
// MEMWB :           WB - WB signals are consumed

// EX stage Control signals--------------------------------------------------------------------------------------------
// EX needs ALUSrcB, ALUOp
module EX_Control(
    input clk,
    input reset_n,
    input stall,
    input flush,

    input [1:0] ALUSrcB_in, output reg [1:0] ALUSrcB_out,
    input [3:0] ALUOp_in, output reg [3:0] ALUOp_out
);
    always @(posedge clk) begin
        if(reset_n == 1'b0 || flush == 1'b1) begin
            ALUSrcB_out = 0;
            ALUOp_out = 0;
        end else if(stall == 1'b0) begin
            ALUSrcB_out = ALUSrcB_in;
            ALUOp_out = ALUOp_in;
        end
    end
endmodule
// MEM stage Control signals-------------------------------------------------------------------------------------------
// MEM needs MemRead, MemWrite
module MEM_Control(
    input clk,
    input reset_n,
    input stall,
    input flush,

    input MemRead_in, output reg MemRead_out,
    input MemWrite_in, output reg MemWrite_out
);
    always @(posedge clk) begin
        if(reset_n == 1'b0 || flush == 1'b1) begin
            MemRead_out = 0;
            MemWrite_out = 0;
        end else if(stall == 1'b0) begin
            MemRead_out = MemRead_in;
            MemWrite_out = MemWrite_in;
        end
    end
endmodule
// WB stage Control signals---------------------------------------------------------------------------------------------
// WB needs RegWriteSrc, RegWrite, is_halted
module WB_Control(
    input clk,
    input reset_n,
    input stall,
    input flush,

    input RegWrite_in, output reg RegWrite_out,
    input [1:0] RegWriteSrc_in, output reg [1:0] RegWriteSrc_out,
    input [`reg_addr_bitlen-1 : 0] RegWrite_addr_in, output reg [`reg_addr_bitlen-1 : 0] RegWrite_addr_out,
    input is_halted_in, output reg is_halted_out
);
    always @(posedge clk) begin
        if(reset_n == 1'b0 || flush == 1'b1) begin
            RegWrite_out = 0;
            RegWriteSrc_out = 0;
            RegWrite_addr_out = 0;
            is_halted_out = 0;
        end else if(stall == 1'b0) begin
            RegWrite_out = RegWrite_in;
            RegWriteSrc_out = RegWriteSrc_in;
            RegWrite_addr_out = RegWrite_addr_in;
            is_halted_out = is_halted_in;
        end
    end
endmodule


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Output port Control Register
module Output_Control(
    input clk,
    input reset_n,
    input stall,
    input flush,

    input output_active_in, output reg output_active_out,
    input [`WORD_SIZE-1 : 0] num_inst_in, output reg [`WORD_SIZE-1 : 0] num_inst_out,
    input [`WORD_SIZE-1 : 0] output_value_in, output reg[`WORD_SIZE-1 : 0] output_value_out
);
    always @(posedge clk) begin
        if(reset_n == 1'b0 || flush == 1'b1) begin
            output_active_out = 0;
            num_inst_out = 0;
            output_value_out = 0;
        end else if(stall == 1'b0) begin
            output_active_out = output_active_in;
            num_inst_out = num_inst_in;
            output_value_out = output_value_in;
        end
    end
endmodule