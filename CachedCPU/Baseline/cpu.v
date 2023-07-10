///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: cpu.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: top module for testbench
// Submodule : PC(ProgramCounter.v), DP(Datapath.v), Control(Control.v)
// Connected module : memory(memory.v)
`timescale 1ns/1ns
`define WORD_SIZE 16    // data and address word size

// INCLUDES
`include "opcodes.v"

// MODULE DECLARATION
module cpu(
        input Clk, 
        input Reset_N, 

	// Instruction memory interface
        output i_readM, 
        output i_writeM, 
        output [`WORD_SIZE-1:0] i_address, 
        inout [`WORD_SIZE-1:0] i_data, 

	// Data memory interface
        output d_readM, 
        output d_writeM, 
        output [`WORD_SIZE-1:0] d_address, 
        inout [`WORD_SIZE-1:0] d_data, 

        output [`WORD_SIZE-1:0] num_inst, 
        output [`WORD_SIZE-1:0] output_port, 
        output is_halted
);

    wire clk, reset_n;
    assign clk = Clk;
    assign reset_n = Reset_N;

    wire IF_stall, IFID_stall, IDEX_stall, EXMEM_stall, MEMWB_stall;
    wire IFID_flush, IDEX_flush, EXMEM_flush, MEMWB_flush;
    wire Jump_Failed, Branch_Failed;

    // to ProgramCounter
    wire [`WORD_SIZE-1 : 0] EX_PC;
    wire Branch_Taken_Result;

    // instructions
    wire [`WORD_SIZE-1 : 0] IF_inst, ID_inst, WB_inst;
    wire [`inst_type_bitlen-1 : 0] WB_inst_type;

    // Control signals that are decoded once : Control_UUT -> DP_UUT
    wire [`inst_type_bitlen-1 : 0] ID_inst_type, EX_inst_type;
    wire ID_RegWrite, ID_MemRead, ID_MemWrite, ID_output_active, ID_is_halted, WB_is_halted, ID_rs_used, ID_rt_used, MEM_MemRead, MEM_MemWrite;
    wire [1:0] ID_RegWriteSrc, ID_ALUSrcB;
    wire [3:0] ID_ALUOp;
    wire [`reg_addr_bitlen-1 : 0] ID_RegWrite_addr, ID_rs, ID_rt;
    // address wires
    wire [`WORD_SIZE-1 : 0] ID_Jump_Target, EX_Branch_Correct_PC, ID_PC, ID_PC_next_seq, IF_PC, IF_PC_next_seq;

    // cache wires
    wire i_cache_ready;
    wire d_cache_ready;

    // memory data
    wire [`WORD_SIZE-1 : 0] MEM_addr;
    wire [`WORD_SIZE-1 : 0] MEM_Read_Data, MEM_Write_Data;
    wire [`WORD_SIZE-1 : 0] d_cache_data;

    // output port
    wire WB_output_active;
    wire [`WORD_SIZE-1 : 0] WB_output_value;

    // HD
    wire [`WORD_SIZE-1 : 0] EX_inst, MEM_inst;
    wire [`inst_type_bitlen-1 : 0] MEM_inst_type;
    wire EX_RegWrite, MEM_RegWrite, WB_RegWrite;
    wire [`reg_addr_bitlen-1 : 0] EX_RegWrite_addr, MEM_RegWrite_addr, WB_RegWrite_addr;

    // HH
    wire IF_data_hazard;
    wire ID_data_hazard;
    wire MEM_data_hazard;

    // d_address is Z when not using memory, MEM_addr when using
    /*assign d_address = (MEM_MemRead || MEM_MemWrite) ? MEM_addr : `WORD_SIZE'bz;*/
    // d_data is Z when reading, Mem_Write_Data when writing.
    assign d_cache_data = MEM_MemWrite ? MEM_Write_Data : `WORD_SIZE'hz;
    // MEM_Read_Data is connected to d_data when MemRead
/*  assign MEM_Read_Data = MEM_MemRead ? d_data : `WORD_SIZE'bx; */
    assign MEM_Read_Data = d_cache_data;

    // memory access through CACHE
    Cache i_cache_UUT(
        .clk (clk),
        .reset_n (reset_n),
        .stall (IF_stall),
        .CPU_readM (1'b1),
        .CPU_writeM (1'b0),
        .CPU_address (IF_PC),
        .CPU_data (IF_inst),

        .MEMORY_readM (i_readM),
        .MEMORY_writeM (i_writeM),
        .MEMORY_address (i_address),
        .MEMORY_data (i_data),

        .CPU_ready (i_cache_ready)
    );
    Cache d_cache_UUT(
        .clk (clk),
        .reset_n (reset_n),
        .stall (EXMEM_stall),
        .CPU_readM (MEM_MemRead),
        .CPU_writeM (MEM_MemWrite),
        .CPU_address (MEM_addr),
        .CPU_data (d_cache_data),

        .MEMORY_readM (d_readM),
        .MEMORY_writeM (d_writeM),
        .MEMORY_address (d_address),
        .MEMORY_data (d_data),

        .CPU_ready (d_cache_ready)
    );

    // throw away reset cycle
    reg reset_cycle;
    reg [`WORD_SIZE-1 : 0] num_inst, output_port;
    reg is_halted;
    integer Branch_Instruction_Count;
    always @(posedge clk) begin
        if(reset_n == 1'b0) begin
            reset_cycle = 1'b0;
            num_inst = 0;
            output_port = `WORD_SIZE'hz;
            is_halted = 0;
            Branch_Instruction_Count = 0;
        end else if(reset_cycle == 1'b0) begin
            reset_cycle = 1'b1;
            num_inst = 0;
            output_port = `WORD_SIZE'hz;
            is_halted = 0;
            Branch_Instruction_Count = 0;
        end else if(WB_inst != `INST_FLUSHED) begin
            num_inst = num_inst+1;
            output_port = WB_output_active ? WB_output_value : `WORD_SIZE'hz;
            is_halted = WB_is_halted;
            if(WB_inst_type[`I_Branch_idx] == 1'b1 || WB_inst_type[`R_JumpReg_idx] == 1'b1 || WB_inst_type[`J_Jump_idx] == 1'b1 || WB_inst_type[`Jump_Link_idx] == 1'b1) begin
                Branch_Instruction_Count = Branch_Instruction_Count + 1;
            end
        end
    end

    // used for debugging, report
    // RAW hazard stall counters
    integer RAW_count, Fail_Count;
    always @(negedge clk) begin
        if(reset_n == 1'b0) begin
            RAW_count = 0;
            Fail_Count = 0;
        end else if(ID_data_hazard == 1'b1) begin
            RAW_count = RAW_count + 1;
        end else if(Branch_Failed == 1'b1 || Jump_Failed == 1'b1) begin
            Fail_Count = Fail_Count + 1;
        end
    end

    PC PC_UUT(
        .clk (clk), .reset_n (reset_n),
        .stall (IF_stall),

        .Jump_Failed (Jump_Failed),
        .Jump_Correct_PC (ID_Jump_Target),
        .Branch_Failed (Branch_Failed),
        .Branch_Correct_PC (EX_Branch_Correct_PC),
        .EX_PC (EX_PC),
        .EX_inst_type (EX_inst_type),
        .Branch_Taken_Result (Branch_Taken_Result),

        .ID_PC (ID_PC),
        .ID_PC_next_seq (ID_PC_next_seq),
        .ID_Jump_Target (ID_Jump_Target),

        .pc (IF_PC),
        .pc_next_seq (IF_PC_next_seq)
    );
    Control Control_UUT(
        .inst (ID_inst),

        .inst_type (ID_inst_type),
        .RegWrite (ID_RegWrite),
        .RegWrite_addr (ID_RegWrite_addr),
        .RegWriteSrc (ID_RegWriteSrc),
        .MemRead (ID_MemRead),
        .MemWrite (ID_MemWrite),
        //.ALUSrcA (),
        .ALUSrcB (ID_ALUSrcB),
        .ALUOp (ID_ALUOp),
        .output_active (ID_output_active),
        .is_halted (ID_is_halted),

        .rs_used (ID_rs_used),
        //.rs_used_stage (),
        .rt_used (ID_rt_used)
        //.rt_used_stage (),
        //.write_value_produce_stage (),
    );
    HazardDetector HD_UUT(
        .IF_data_ready (i_cache_ready),
        .IF_data_hazard (IF_data_hazard),

        .ID_rs_used (ID_rs_used),

        .ID_rs (ID_rs),
        .ID_rt_used (ID_rt_used),
        .ID_rt (ID_rt),

        .EX_inst (EX_inst),
        .EX_RegWrite (EX_RegWrite),
        .EX_RegWrite_addr (EX_RegWrite_addr),

        .MEM_RegWrite (MEM_RegWrite),
        .MEM_RegWrite_addr (MEM_RegWrite_addr),

        .WB_RegWrite (WB_RegWrite),
        .WB_RegWrite_addr (WB_RegWrite_addr),

        .ID_data_hazard (ID_data_hazard),

        .MEM_inst (MEM_inst),
        .MEM_inst_type (MEM_inst_type),
        .MEM_data_ready (d_cache_ready),
        
        .MEM_data_hazard (MEM_data_hazard),

        .EX_inst_type (EX_inst_type),
        .EX_Correct_PC (EX_Branch_Correct_PC),

        .ID_inst_type (ID_inst_type),
        .ID_Correct_PC (ID_Jump_Target),

        .IF_PC (IF_PC),
        .ID_PC (ID_PC),

        .Branch_Failed (Branch_Failed),
        .Jump_Failed (Jump_Failed)
    );
    HazardHandler HH_UUT(
        .IF_data_hazard (IF_data_hazard),
        .ID_data_hazard (ID_data_hazard),
        .MEM_data_hazard (MEM_data_hazard),
        
        .Jump_Failed (Jump_Failed),
        .Branch_Failed (Branch_Failed),

        .WB_is_halted (WB_is_halted),

        .IF_stall (IF_stall),
        .IFID_stall (IFID_stall),
        .IDEX_stall (IDEX_stall),
        .EXMEM_stall (EXMEM_stall),
        .MEMWB_stall (MEMWB_stall),

        .IFID_flush (IFID_flush),
        .IDEX_flush (IDEX_flush),
        .EXMEM_flush (EXMEM_flush),
        .MEMWB_flush (MEMWB_flush)
    );
    DP DP_UUT(
        .clk (clk),
        .reset_n (reset_n),

        .IF_PC (IF_PC),
        .IF_PC_next_seq (IF_PC_next_seq),
        .IF_inst (IF_inst),

        .ID_inst (ID_inst),

        .ID_inst_type (ID_inst_type),
        .ID_RegWrite (ID_RegWrite),
        .ID_RegWrite_addr (ID_RegWrite_addr),
        .ID_RegWriteSrc (ID_RegWriteSrc),
        .ID_MemRead (ID_MemRead),
        .ID_MemWrite (ID_MemWrite),
        .ID_ALUSrcB (ID_ALUSrcB),
        .ID_ALUOp (ID_ALUOp),
        .ID_output_active (ID_output_active),
        .ID_is_halted (ID_is_halted),
        .ID_rs_used (ID_rs_used),
        .ID_rt_used (ID_rt_used),

        .ID_PC(ID_PC),
        .ID_PC_next_seq (ID_PC_next_seq),
        .ID_Jump_Target (ID_Jump_Target),

        .EX_Branch_Correct_PC (EX_Branch_Correct_PC),
        .EX_PC (EX_PC),
        .Branch_Taken (Branch_Taken_Result),

        .MEM_MemRead (MEM_MemRead),
        .MEM_MemWrite (MEM_MemWrite),
        .MEM_addr (MEM_addr),
        .MEM_Read_Data (MEM_Read_Data),
        .MEM_Write_Data (MEM_Write_Data),

        .WB_output_active (WB_output_active),
        .WB_output_value (WB_output_value),
        .WB_is_halted (WB_is_halted),
        .WB_inst (WB_inst),
        .WB_inst_type (WB_inst_type),

        .ID_rs (ID_rs),
        .ID_rt (ID_rt),
        .EX_inst(EX_inst),
        .MEM_inst(MEM_inst),
        .EX_RegWrite (EX_RegWrite),
        .EX_RegWrite_addr (EX_RegWrite_addr),
        .MEM_RegWrite (MEM_RegWrite),
        .MEM_RegWrite_addr (MEM_RegWrite_addr),
        .WB_RegWrite (WB_RegWrite),
        .WB_RegWrite_addr (WB_RegWrite_addr),

        .EX_inst_type (EX_inst_type),
        .MEM_inst_type (MEM_inst_type),

        .IFID_stall (IFID_stall),
        .IDEX_stall (IDEX_stall),
        .EXMEM_stall (EXMEM_stall),
        .MEMWB_stall (MEMWB_stall),

        .IFID_flush (IFID_flush),
        .IDEX_flush (IDEX_flush),
        .EXMEM_flush (EXMEM_flush),
        .MEMWB_flush (MEMWB_flush)
    );

endmodule