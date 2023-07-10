///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: HazardHandler.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: Stalls or flushes pipeline according to detected hazard

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION
module HazardHandler(
    input IF_data_hazard, // memory delay
    input ID_data_hazard, // RAW hazard occurred
    input MEM_data_hazard, // memory delay

    input Jump_Failed,      // Unconditional Jump Prediction Failed
    input Branch_Failed,    // Branch Prediction Failed

    input WB_is_halted,

    output reg IF_stall,    // Do not update PC
    output reg IFID_stall,  // IFID do not latch new instruction -> Flush IDEX next posedge
    output reg IDEX_stall,
    output reg EXMEM_stall,
    output reg MEMWB_stall,
    output reg IFID_flush,  // Flush IFID register
    output reg IDEX_flush,  // Flush IDEX register
    output reg EXMEM_flush, // Flush EXMEM register
    output reg MEMWB_flush // Flush MEMWB register
);

    // if pipeline ahead is stalled -> stall (blocked)
    // if pipeline following is stalled -> flush (can't latch)
    always @(*) begin
        // default values - nothing stall or flush
        IF_stall = 1'b0;
        IFID_stall = 1'b0;
        IDEX_stall = 1'b0;
        EXMEM_stall = 1'b0;
        MEMWB_stall = 1'b0;
        IFID_flush = 1'b0;
        IDEX_flush = 1'b0;
        EXMEM_flush = 1'b0;
        MEMWB_flush = 1'b0;

        if(WB_is_halted) begin
            IF_stall = 1'b1;
            IFID_stall = 1'b1;
            IDEX_stall = 1'b1;
            EXMEM_stall = 1'b1;
            MEMWB_stall = 1'b1;

        // MEM stage hazard is oldest instruction among hazard - Should be handled first
        end else if(MEM_data_hazard) begin
            IF_stall = 1'b1;
            IFID_stall = 1'b1;
            IDEX_stall = 1'b1;
            EXMEM_stall = 1'b1;
            MEMWB_flush = 1'b1; // If EXMEM is stalled -> MEMWB will be flushed next posedge

        // Branch is next oldest instruction among hazard. Branch_Failed : flush IFID, IDEX ----------//
        end else if(Branch_Failed) begin    
            IFID_flush = 1'b1;
            IDEX_flush = 1'b1;

        // RAW should be resolved before Jump_Failed detection ----------------------------------//
        end else if(ID_data_hazard) begin   
            IF_stall = 1'b1;
            IFID_stall = 1'b1;
            IDEX_flush = 1'b1; // If IFID is stalled -> IDEX will be flushed next posedge

        // Jump_Faild : flush IFID --------------------------------------------------------------//
        end else if(Jump_Failed) begin      
            IFID_flush = 1'b1;

        end else if(IF_data_hazard) begin
            IF_stall = 1'b1;
            IFID_flush = 1'b1; // If IF is stalled -> IFID will be flushed next posedge

        // No hazard : don't stall, don't flush -------------------------------------------------//
        end else begin                      
            IF_stall = 1'b0;
            IFID_stall = 1'b0;
            IDEX_stall = 1'b0;
            EXMEM_stall = 1'b0;
            MEMWB_stall = 1'b0;
            IFID_flush = 1'b0;
            IDEX_flush = 1'b0;
            EXMEM_flush = 1'b0;
            MEMWB_flush = 1'b0;
        end
    end
endmodule