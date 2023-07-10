///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: HazardDetector.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: Hazard detection module under cpu from "cpu.v" - detects RAW, Branch misprediction, Cache miss

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION
module HazardDetector(
    // Data Hazard Detection
    input IF_data_ready,

    output IF_data_hazard,

    input ID_rs_used,   // Does instruction in ID stage(IFID.inst_out) uses rs?
    input [`reg_addr_bitlen-1 : 0] ID_rs,
    input ID_rt_used,   // Does instruction in ID stage(IFID.inst_out) uses rt?
    input [`reg_addr_bitlen-1 : 0] ID_rt,

    input [`WORD_SIZE-1 : 0] EX_inst,
    input EX_RegWrite,
    input [`reg_addr_bitlen-1 : 0] EX_RegWrite_addr,

    input MEM_RegWrite,
    input [`reg_addr_bitlen-1 : 0] MEM_RegWrite_addr,

    input WB_RegWrite,
    input [`reg_addr_bitlen-1 : 0] WB_RegWrite_addr,

    output ID_data_hazard,

    input [`WORD_SIZE-1 : 0] MEM_inst,
    input [`inst_type_bitlen-1 : 0] MEM_inst_type,
    input MEM_data_ready,

    output MEM_data_hazard,

    // Control Hazard Detection
    input [`inst_type_bitlen-1 : 0] EX_inst_type,
    input [`WORD_SIZE-1 : 0] EX_Correct_PC,

    input [`inst_type_bitlen-1 : 0] ID_inst_type,
    input [`WORD_SIZE-1 : 0] ID_Correct_PC,

    input [`WORD_SIZE-1 : 0] IF_PC,
    input [`WORD_SIZE-1 : 0] ID_PC,
    
    output reg Branch_Failed,   // Branch Prediction Failed
    output reg Jump_Failed      // Unconditional Jump Failed
);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // IF fetch Data Hazard Detection
    // IF data hazard : instruction not fetched yet ---------------------------------------------------------------------//
    reg IF_data_hazard;
    always @(*) begin
        if(~IF_data_ready) IF_data_hazard = 1'b1;
        else IF_data_hazard = 1'b0;
    end
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ID Register Data Hazard Detection
    reg ID_rs_data_hazard;
    reg ID_rt_data_hazard;
    // rs_data_hazard ---------------------------------------------------------------------------------------------------//
    always @(*) begin
        if(ID_rs_used) begin
            if(`DATA_FORWARDING_ENABLED == 1'b0) begin // Data forwarding disabled
                if(EX_RegWrite && (EX_RegWrite_addr == ID_rs)) ID_rs_data_hazard = 1'b1;
                else if(MEM_RegWrite && (MEM_RegWrite_addr == ID_rs)) ID_rs_data_hazard = 1'b1;
                else if((`RF_SELF_FORWARDING == 1'b0) && WB_RegWrite && (WB_RegWrite_addr == ID_rs)) ID_rs_data_hazard = 1'b1;
                else ID_rs_data_hazard = 1'b0;
            end else begin // full data forwarding enabled : only LWD of dist=1 causes hazard
                if(EX_RegWrite && (EX_RegWrite_addr == ID_rs) && (EX_inst[`opcode_idx_left:`opcode_idx_right] == `OPCODE_LWD)) ID_rs_data_hazard = 1'b1;
                else ID_rs_data_hazard = 1'b0;
            end
        end else begin
            ID_rs_data_hazard = 1'b0;
        end
    end
    // rt_data_hazard ---------------------------------------------------------------------------------------------------//
    always @(*) begin
        if(ID_rt_used) begin
            if(`DATA_FORWARDING_ENABLED == 1'b0) begin // Data forwarding disabled
                if(EX_RegWrite && (EX_RegWrite_addr == ID_rt)) ID_rt_data_hazard = 1'b1;
                else if(MEM_RegWrite && (MEM_RegWrite_addr == ID_rt)) ID_rt_data_hazard = 1'b1;
                else if((`RF_SELF_FORWARDING == 1'b0) && WB_RegWrite && (WB_RegWrite_addr == ID_rt)) ID_rt_data_hazard = 1'b1;
                else ID_rt_data_hazard = 1'b0;
            end else begin // full data forwarding enabled : only LWD of dist=1 causes hazard
                if(EX_RegWrite && (EX_RegWrite_addr == ID_rt) && (EX_inst[`opcode_idx_left:`opcode_idx_right] == `OPCODE_LWD)) ID_rt_data_hazard = 1'b1;
                else ID_rt_data_hazard = 1'b0;
            end
        end else begin
            ID_rt_data_hazard = 1'b0;
        end
    end

    assign ID_data_hazard = (ID_rs_data_hazard) || (ID_rt_data_hazard);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MEM memory Data Hazard Detection
    reg MEM_data_hazard;
    always @(*) begin
        if(MEM_inst_type[`I_memory_idx] == 1'b1) begin
            if(MEM_data_ready) MEM_data_hazard = 1'b0;
            else MEM_data_hazard = 1'b1;
        end else begin
            MEM_data_hazard = 1'b0;
        end
    end

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Control Hazard Detection
    // Branch Prediction fail(after EX) has higher priority than Unconditional Jump(after ID) since it's older instruction

    // Branch Prediction failure check ----------------------------------------------------------------------------------//
    always @(*) begin
        if(EX_inst_type[`I_Branch_idx] == 1'b0) begin
            Branch_Failed = 1'b0;
        end else begin
            Branch_Failed = (EX_Correct_PC == ID_PC) ? 1'b0 : 1'b1;
        end
    end

    // Jump Prediction failure check ------------------------------------------------------------------------------------//
    always @(*) begin
        if(ID_inst_type[`J_Jump_idx] || ID_inst_type[`R_JumpReg_idx] || ID_inst_type[`Jump_Link_idx]) begin
            Jump_Failed = (ID_Correct_PC == IF_PC) ? 1'b0 : 1'b1;
        end else begin
            Jump_Failed = 1'b0;
        end
    end

endmodule