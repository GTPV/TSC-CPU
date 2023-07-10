///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: Cache.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description : DMA controller module under cpu from "cpu.v" - Connect/disconnect bus from D-cache, generate BUS_Grant signal.

// INCLUDES
`include "opcodes.v"

module DMAController(
    input clk,
    input reset_n,

    // DMA interface
    input DMA_begin,
    input DMA_end,
    input BUS_Request,
    output reg DMA_cmd,
    output BUS_Grant,

    // CPU interface : Dcache interface
    input DCACHE_readM,
    input DCACHE_writeM,
    input [`WORD_SIZE-1 : 0] DCACHE_address,
    inout [`MEMORY_BANDWIDTH-1 : 0] DCACHE_data,

    // MEMORY interface
    output reg MEMORY_readM,
    output reg MEMORY_writeM,
    output reg [`WORD_SIZE-1 : 0] MEMORY_address,
    inout [`MEMORY_BANDWIDTH-1 : 0] MEMORY_data,

    output DCACHE_ready
);

    reg BUS_Grant_Prev;

    // DCACHE can use memory when ~BUS_Grant
    assign DCACHE_ready = ~BUS_Grant;

    // Memory interface - if BUS_Grant : disconnect DCACHE<->MEMORY, if ~BUS_Grant : connect DCACHE<->MEMORY
    assign DCACHE_data = BUS_Grant ? `WORD_SIZE'bz : DCACHE_readM ? MEMORY_data : `MEMORY_BANDWIDTH'hz;
    assign MEMORY_data = BUS_Grant ? `WORD_SIZE'bz : DCACHE_writeM ? DCACHE_data : `MEMORY_BANDWIDTH'hz;
    always @(*) begin
        if(BUS_Grant) begin
            MEMORY_readM = 1'b0;
            MEMORY_writeM = 1'bz;
            MEMORY_address = `WORD_SIZE'bz;
        end else begin
            MEMORY_readM = DCACHE_readM;
            MEMORY_writeM = DCACHE_writeM;
            MEMORY_address = DCACHE_address;
        end
    end

    // BUS grant : Only when BR - if bus was granted last cycle or cpu is not using memory
    assign BUS_Grant =  (`DMA_ENABLED == 1'b1) && 
                        (BUS_Request == 1'b1) &&
                        (BUS_Grant_Prev == 1'b1 || 
                            (DCACHE_readM == 1'b0 && DCACHE_writeM == 1'b0)
                        );

    always @(posedge clk) begin
        if(reset_n == 1'b0) begin
            DMA_cmd = 1'b0;
            BUS_Grant_Prev = 1'b0;
        end else begin
            BUS_Grant_Prev = BUS_Grant;
            DMA_cmd = DMA_begin;
        end
    end

endmodule