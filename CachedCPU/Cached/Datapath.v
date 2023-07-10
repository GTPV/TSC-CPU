///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: Datapath.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: Datapath module under cpu from "cpu.v"

// INCLUDES
`include "opcodes.v"

// MODULE DECLARATION

module DP(
    input clk,
    input reset_n,

    input [`WORD_SIZE-1 : 0] IF_PC,
    input [`WORD_SIZE-1 : 0] IF_PC_next_seq,
    input [`WORD_SIZE-1 : 0] IF_inst,

    // ID stage ----------------------------------------------------------//

    // Instruction data for Control Unit to generate Control signals
    output [`WORD_SIZE-1 : 0] ID_inst,

    // Control signals from Control Unit
    input [`inst_type_bitlen-1 : 0] ID_inst_type,
    input ID_RegWrite,
    input [`reg_addr_bitlen-1 : 0] ID_RegWrite_addr,
    input [1:0] ID_RegWriteSrc,
    input ID_MemRead,
    input ID_MemWrite,
    input [1:0] ID_ALUSrcB,
    input [3:0] ID_ALUOp,
    input ID_output_active,
    input ID_is_halted,
    input ID_rs_used,
    input ID_rt_used,
    
    // Instruction addresses of decoded instruction
    output [`WORD_SIZE-1 : 0] ID_PC,
    output [`WORD_SIZE-1 : 0] ID_PC_next_seq,
    output reg [`WORD_SIZE-1 : 0] ID_Jump_Target,
    output [`WORD_SIZE-1 : 0] EX_Branch_Correct_PC,
    output [`WORD_SIZE-1 : 0] EX_PC,
    output reg Branch_Taken,

    // MEM stage : To memory Unit ----------------------------------------//
    output MEM_MemRead,
    output MEM_MemWrite,
    output [`WORD_SIZE-1 : 0] MEM_addr,
    input [`WORD_SIZE-1 : 0] MEM_Read_Data,
    output [`WORD_SIZE-1 : 0] MEM_Write_Data,

    // WB stage : output port control ------------------------------------//
    output WB_output_active,
    output [`WORD_SIZE-1 : 0] WB_output_value,
    output WB_is_halted,
    output [`WORD_SIZE-1 : 0] WB_inst,
    output [`inst_type_bitlen-1 : 0] WB_inst_type,

    // To HazardDetector Unit --------------------------------------------//
    output [`reg_addr_bitlen-1 : 0] ID_rs,
    output [`reg_addr_bitlen-1 : 0] ID_rt,
    output [`WORD_SIZE-1 : 0] EX_inst,
    output [`WORD_SIZE-1 : 0] MEM_inst,
    output EX_RegWrite,
    output [`reg_addr_bitlen-1 : 0] EX_RegWrite_addr,
    output MEM_RegWrite,
    output [`reg_addr_bitlen-1 : 0] MEM_RegWrite_addr,
    output WB_RegWrite,
    output [`reg_addr_bitlen-1 : 0] WB_RegWrite_addr,

    output [`inst_type_bitlen-1 : 0] EX_inst_type,
    output [`inst_type_bitlen-1 : 0] MEM_inst_type,

    // From HazardHander Unit
    input IFID_stall,
    input IDEX_stall,
    input EXMEM_stall,
    input MEMWB_stall,

    input IFID_flush,
    input IDEX_flush,
    input EXMEM_flush,
    input MEMWB_flush
);
    // wires
    wire [`WORD_SIZE-1 : 0] MEM_PC, WB_PC;
    wire [`WORD_SIZE-1 : 0] EX_PC_next_seq, MEM_PC_next_seq, WB_PC_next_seq;
    wire [`WORD_SIZE-1 : 0] RF_rs_value, RF_rt_value, ID_rs_value, ID_rt_value, EX_rs_value, EX_rt_value, MEM_rs_value, MEM_rt_value;
    wire [`WORD_SIZE-1 : 0] EX_RegWrite_value, MEM_RegWrite_value, WB_RegWrite_value;
    wire [`WORD_SIZE-1 : 0] ID_I_imm, EX_I_imm;
    wire [`WORD_SIZE-1 : 0] ID_Branch_Target, ID_Concat_Target;
    wire [`WORD_SIZE-1 : 0] EX_Jump_Target;
    wire [1:0] EX_ALUSrcB;
    wire [3:0] EX_ALUOp;
    wire EX_MemRead /*, MEM_MemRead */;
    wire EX_MemWrite /*, MEM_MemWrite */;
/*  wire EX_RegWrite, MEM_RegWrite, WB_RegWrite; */
    wire [1:0] EX_RegWriteSrc, MEM_RegWriteSrc, WB_RegWriteSrc;
/*  wire [`reg_addr_bitlen-1 : 0] EX_RegWrite_addr, MEM_RegWrite_addr, WB_RegWrite_addr; */
    wire EX_is_halted, MEM_is_halted;
    wire EX_output_active, MEM_output_active;
    wire [`WORD_SIZE-1 : 0] EX_output_value, MEM_output_value;
    wire [`WORD_SIZE-1 : 0] EX_ALU_result, MEM_ALU_result, WB_ALU_result;
    wire [`WORD_SIZE-1 : 0] WB_MemData;
    // IF stage
    IFID IFID_UUT(
        .clk (clk), .reset_n (reset_n),
        .PC_in (IF_PC), .PC_out (ID_PC),
        .PC_next_seq_in (IF_PC_next_seq), .PC_next_seq_out (ID_PC_next_seq),
        .inst_in (IF_inst), .inst_out (ID_inst),

        .stall (IFID_stall),
        .flush (IFID_flush)
    );
    // ID stage -------------------------------------------------------------------------------------------------------------------//
    assign ID_I_imm = {{8{ID_inst[`imm_off_idx_left]}}, ID_inst[`imm_off_idx_left : `imm_off_idx_right]};
    assign ID_rs = ID_inst[`rs_idx_left : `rs_idx_right];
    assign ID_rt = ID_inst[`rt_idx_left : `rt_idx_right];

    // Data forwarding ---------------------------------------------------------------------------------//
    assign MEM_RegWrite_value = (MEM_RegWriteSrc == `RegWriteSrc_ALU) ? MEM_ALU_result :
                                (MEM_RegWriteSrc == `RegWriteSrc_MEM) ? MEM_Read_Data : MEM_PC_next_seq;
    assign EX_RegWrite_value =  (EX_RegWriteSrc == `RegWriteSrc_ALU) ? EX_ALU_result :
                                (EX_RegWriteSrc == `RegWriteSrc_MEM)? `WORD_SIZE'bx : EX_PC_next_seq;
    // rs forwarding
    assign ID_rs_value = (`DATA_FORWARDING_ENABLED == 1'b0) ?               RF_rs_value : 
                        (EX_RegWrite && (ID_rs == EX_RegWrite_addr)) ?      EX_RegWrite_value : 
                        (MEM_RegWrite && (ID_rs == MEM_RegWrite_addr)) ?    MEM_RegWrite_value :
                                                                            RF_rs_value;
    // rt forwarding
    assign ID_rt_value = (`DATA_FORWARDING_ENABLED == 1'b0) ?               RF_rt_value : 
                        (EX_RegWrite && (ID_rt == EX_RegWrite_addr)) ?      EX_RegWrite_value : 
                        (MEM_RegWrite && (ID_rt == MEM_RegWrite_addr)) ?    MEM_RegWrite_value :
                                                                            RF_rt_value;
    // End of Data forwarding --------------------------------------------------------------------------//

    RF RF_UUT(
        .clk (clk), .reset_n (reset_n),

        .addr1 (ID_rs),
        .data1 (RF_rs_value),

        .addr2 (ID_rt),
        .data2 (RF_rt_value),

        .write (WB_RegWrite),
        .addr3 (WB_RegWrite_addr),
        .data3 (WB_RegWrite_value)
    );
    ALU ALU_Branch_Target(
        .A (ID_PC_next_seq),
        .B (ID_I_imm),
        .Cin (1'b0),

        .OP (`OP_ADD),

        .C (ID_Branch_Target),
        .Cout (),
        .Cmp ()
    );
    assign ID_Concat_Target = {ID_PC[`WORD_SIZE-1 : `WORD_SIZE-4], ID_inst[`target_addr_idx_left : `target_addr_idx_right]};
    // Choose Taken Address(ID_Jump_Target) according to instruction type
    always @(*) begin
        if(ID_inst_type[`I_Branch_idx]) begin
            ID_Jump_Target = ID_Branch_Target;
        end else if(ID_inst_type[`J_Jump_idx]) begin
            ID_Jump_Target = ID_Concat_Target;
        end else if(ID_inst_type[`R_JumpReg_idx]) begin
            ID_Jump_Target = ID_rs_value;
        end else begin
            ID_Jump_Target = ID_PC_next_seq;
        end
    end

    EX_Control IDEX_EX_Control(
        .clk (clk), .reset_n (reset_n),
        .stall (IDEX_stall), .flush (IDEX_flush),
        .ALUSrcB_in (ID_ALUSrcB), .ALUSrcB_out (EX_ALUSrcB),
        .ALUOp_in (ID_ALUOp), .ALUOp_out (EX_ALUOp)
    );
    MEM_Control IDEX_MEM_Control(
        .clk (clk), .reset_n (reset_n),
        .stall (IDEX_stall), .flush (IDEX_flush),
        .MemRead_in (ID_MemRead), .MemRead_out (EX_MemRead),
        .MemWrite_in (ID_MemWrite), .MemWrite_out (EX_MemWrite)
    );
    WB_Control IDEX_WB_Control(
        .clk (clk), .reset_n (reset_n),
        .stall (IDEX_stall), .flush (IDEX_flush),
        .RegWrite_in (ID_RegWrite), .RegWrite_out (EX_RegWrite),
        .RegWriteSrc_in (ID_RegWriteSrc), .RegWriteSrc_out (EX_RegWriteSrc),
        .RegWrite_addr_in (ID_RegWrite_addr), .RegWrite_addr_out (EX_RegWrite_addr),
        .is_halted_in (ID_is_halted), .is_halted_out (EX_is_halted)
    );
    Output_Control IDEX_Output_Control(
        .clk (clk), .reset_n (reset_n),
        .stall (IDEX_stall), .flush (IDEX_flush),
        .output_active_in (ID_output_active), .output_active_out (EX_output_active),
        .output_value_in (ID_rs_value), .output_value_out (EX_output_value)
    );
    IDEX IDEX_UUT(
        .clk (clk), .reset_n (reset_n),
        .PC_in (ID_PC), .PC_out (EX_PC),
        .PC_next_seq_in (ID_PC_next_seq), .PC_next_seq_out (EX_PC_next_seq),
        .inst_in (ID_inst), .inst_out (EX_inst),
        .inst_type_in (ID_inst_type), .inst_type_out (EX_inst_type),

        .rs_value_in (ID_rs_value), .rs_value_out (EX_rs_value),
        .rt_value_in (ID_rt_value), .rt_value_out (EX_rt_value),
        .I_imm_in (ID_I_imm), .I_imm_out (EX_I_imm),

        .Jumped_PC_in (ID_Jump_Target), .Jumped_PC_out (EX_Jump_Target),

        .stall (IDEX_stall),
        .flush (IDEX_flush)
    );
    // EX stage -------------------------------------------------------------------------------------------------------------------//
    // Mux selection by Control signals
    wire [`WORD_SIZE-1 : 0] EX_ALUSrcB_value;
    assign EX_ALUSrcB_value = (EX_ALUSrcB == `ALUSrcB_I_OFFSET) ? EX_I_imm :
                                (EX_ALUSrcB == `ALUSrcB_RF_rt)  ? EX_rt_value : `WORD_SIZE'b0;
    wire [1:0] EX_Branch_Check;
    wire [`opcode_bitlen-1 : 0] EX_opcode;
    assign EX_opcode = EX_inst[`opcode_idx_left : `opcode_idx_right];
    always @(*) begin
        if(EX_inst_type[`I_Branch_idx]) begin
            case(EX_opcode)
                `OPCODE_BEQ : Branch_Taken = (EX_Branch_Check == 2'b00);
                `OPCODE_BNE : Branch_Taken = (EX_Branch_Check != 2'b00);
                `OPCODE_BGZ : Branch_Taken = (EX_Branch_Check == 2'b10);
                `OPCODE_BLZ : Branch_Taken = (EX_Branch_Check == 2'b11);
            endcase
        end else begin
            Branch_Taken = 1'b0;
        end
    end
    assign EX_Branch_Correct_PC = (Branch_Taken) ? EX_Jump_Target : EX_PC_next_seq;
    ALU ALU_main(
        .A (EX_rs_value),
        .B (EX_ALUSrcB_value),
        .Cin (1'b0),

        .OP (EX_ALUOp),

        .C (EX_ALU_result),
        .Cout (),
        .Cmp (EX_Branch_Check)
    );
    MEM_Control EXMEM_MEM_Control(
        .clk (clk), .reset_n (reset_n),
        .stall (EXMEM_stall), .flush (EXMEM_flush),
        .MemRead_in (EX_MemRead), .MemRead_out (MEM_MemRead),
        .MemWrite_in (EX_MemWrite), .MemWrite_out (MEM_MemWrite)
    );
    WB_Control EXMEM_WB_Control(
        .clk (clk), .reset_n (reset_n),
        .stall (EXMEM_stall), .flush (EXMEM_flush),
        .RegWrite_in (EX_RegWrite), .RegWrite_out (MEM_RegWrite),
        .RegWriteSrc_in (EX_RegWriteSrc), .RegWriteSrc_out (MEM_RegWriteSrc),
        .RegWrite_addr_in (EX_RegWrite_addr), .RegWrite_addr_out (MEM_RegWrite_addr),
        .is_halted_in (EX_is_halted), .is_halted_out (MEM_is_halted)
    );
    Output_Control EXMEM_Output_Control(
        .clk (clk), .reset_n (reset_n),
        .stall (EXMEM_stall), .flush (EXMEM_flush),
        .output_active_in (EX_output_active), .output_active_out (MEM_output_active),
        .output_value_in (EX_rs_value), .output_value_out (MEM_output_value)
    );
    EXMEM EXMEM_UUT(
        .clk (clk), .reset_n (reset_n),
        .PC_in (EX_PC), .PC_out (MEM_PC),
        .PC_next_seq_in (EX_PC_next_seq), .PC_next_seq_out (MEM_PC_next_seq),
        .inst_in (EX_inst), .inst_out (MEM_inst),
        .inst_type_in (EX_inst_type), .inst_type_out (MEM_inst_type),

        .rt_value_in (EX_rt_value), .rt_value_out (MEM_rt_value),

        .ALU_result_in (EX_ALU_result), .ALU_result_out (MEM_ALU_result),

        .stall (EXMEM_stall),
        .flush (EXMEM_flush)
    );

    // MEM stage ------------------------------------------------------------------------------------------------------------------//
    assign MEM_addr = MEM_ALU_result;
    assign MEM_Write_Data = MEM_rt_value;
    WB_Control MEMWB_WB_Control(
        .clk (clk), .reset_n (reset_n),
        .stall (MEMWB_stall), .flush (MEMWB_flush),
        .RegWrite_in (MEM_RegWrite), .RegWrite_out (WB_RegWrite),
        .RegWriteSrc_in (MEM_RegWriteSrc), .RegWriteSrc_out (WB_RegWriteSrc),
        .RegWrite_addr_in (MEM_RegWrite_addr), .RegWrite_addr_out (WB_RegWrite_addr),
        .is_halted_in (MEM_is_halted), .is_halted_out (WB_is_halted)
    );
    Output_Control MEMWB_Output_Control(
        .clk (clk), .reset_n (reset_n),
        .stall (MEMWB_stall), .flush (MEMWB_flush),
        .output_active_in (MEM_output_active), .output_active_out (WB_output_active),
        .output_value_in (MEM_output_value), .output_value_out (WB_output_value)
    );
    MEMWB MEMWB_UUT(
        .clk (clk), .reset_n (reset_n),
        .PC_in (MEM_PC), .PC_out (WB_PC),
        .PC_next_seq_in (MEM_PC_next_seq), .PC_next_seq_out (WB_PC_next_seq),
        .inst_in (MEM_inst), .inst_out (WB_inst),
        .inst_type_in (MEM_inst_type), .inst_type_out (WB_inst_type),

        .ALU_result_in (MEM_ALU_result), .ALU_result_out (WB_ALU_result),

        .MemData_in (MEM_Read_Data), .MemData_out (WB_MemData),

        .stall (MEMWB_stall),
        .flush (MEMWB_flush)
    );
    // WB stage -------------------------------------------------------------------------------------------------------------------//
    assign WB_RegWrite_value = (WB_RegWriteSrc == `RegWriteSrc_ALU) ? WB_ALU_result :
                            (WB_RegWriteSrc == `RegWriteSrc_MEM)? WB_MemData : WB_PC_next_seq;
endmodule