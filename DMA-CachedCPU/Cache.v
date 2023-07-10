///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: Cache.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description : Cache module under cpu from "cpu.v" - L1 cache with write-back + write-allocate

// INCLUDES
`include "opcodes.v"

// DEFINES
`define CACHE_BLOCK_SIZE_WORD (1<<`cache_block_offset_bitlen) // block size as word
`define CACHE_HEIGHT (1<<`cache_idx_bitlen) // cache table height : 2^idx
//`define cache_tag_bitlen (`WORD_SIZE - `cache_idx_bitlen - `cache_block_offset_bitlen) // cache tag length
`define cache_tag_bitlen 12

module Cache(
    input clk,
    input reset_n,
    input stall,

    // ports from/to CPU module
    input CPU_readM,
    input CPU_writeM,
    input [`WORD_SIZE-1 : 0] CPU_address,
    inout [`WORD_SIZE-1 : 0] CPU_data,

    input MEMORY_available,

    // ports from/to MEMORY module
    output reg MEMORY_readM,
    output reg MEMORY_writeM,
    output reg [`WORD_SIZE-1 : 0] MEMORY_address,
    inout [`MEMORY_BANDWIDTH-1 : 0] MEMORY_data,

    output reg CPU_ready,

    output Busy,
    output reg Hit
);

    reg [`MEMORY_BANDWIDTH-1 : 0] MEMORY_write_data;
    reg Busy; // CACHE is reading/writing data from/to memory
    reg [2:0] MEMORY_delay_count;

    // Cache Tag Bank : height = `CACHE_HEIGHT, width = `cache_tag_bitlen
    reg [`cache_tag_bitlen-1 : 0] Cache_Tag_Bank[`CACHE_HEIGHT-1 : 0];
    // Cache Valid bits(single bit for each row(line)) : height = `CACHE_HEIGHT, width = 1
    reg Cache_Valid[`CACHE_HEIGHT-1 : 0];
    reg Cache_Dirty[`CACHE_HEIGHT-1 : 0];
    // Data Bank : height = `CACHE_HEIGHT, width = `CACHE_BLOCK_SIZE_WORD * `WORD_SIZE
    reg [`WORD_SIZE-1 : 0] Cache_Data_Bank[`CACHE_HEIGHT-1 : 0][`CACHE_BLOCK_SIZE_WORD-1 : 0];

/*     assign CPU_data = CPU_readM ? (`CACHE_ENABLED ? cache_table[][] : MEMORY_data) : `WORD_SIZE'hz; */


    //######################################################################################################################//
    // parse address to access cache table ---------------------------------------------------------------------------------//
    /* CPU_address : 16-bit
    ------------------------------------------------------
    |  Ongoing_tag  | Ongoing_idx | Ongoing_block_offset |                                        
    |    12-bit     |    2-bit    |        2-bit         |                                        
    ------------------------------------------------------
    */
    reg [`WORD_SIZE-1 : 0] Ongoing_address;
    reg [1:0] Ongoing_state;
    reg [`cache_tag_bitlen-1 : 0] Ongoing_tag;
    reg [`cache_idx_bitlen-1 : 0] Ongoing_idx;
    reg [`cache_block_offset_bitlen-1 : 0] Ongoing_block_offset;
    always @(*) begin
        {Ongoing_tag, Ongoing_idx, Ongoing_block_offset} = Ongoing_address;
    end
    reg [`cache_tag_bitlen-1 : 0] CPU_tag;
    reg [`cache_idx_bitlen-1 : 0] CPU_idx;
    reg [`cache_block_offset_bitlen-1 : 0] CPU_block_offset;
    always @(*) begin
        {CPU_tag, CPU_idx, CPU_block_offset} = CPU_address;
    end

    //######################################################################################################################//
    // check CACHE table, determine Hit/Miss -------------------------------------------------------------------------------//
    always @(*) begin
        if(CPU_tag != Cache_Tag_Bank[CPU_idx]) begin // Different Tag : Collision -> Miss
            Hit = 1'b0;
        end else if(Cache_Valid[CPU_idx] == 1'b0) begin // Invalidated -> Miss
            Hit = 1'b0;
        end else begin // Cache hit
            Hit = 1'b1;
        end
    end

    //######################################################################################################################//
    // assign CPU_ready ----------------------------------------------------------------------------------------------------//
    always @(*) begin
        if(Busy == 1'b1) begin // CACHE is busy : reading/writing data from/to memory ----//
            CPU_ready = 1'b0;
        end else begin // CACHE is not busy ----------------------------------------------//
            if(Hit) CPU_ready = 1'b1;
            else if(CPU_readM == 1'b0 && CPU_writeM == 1'b0) CPU_ready = 1'b1;
            else CPU_ready = 1'b0;
        end
    end

    //######################################################################################################################//
    // assign CPU/MEMORY interface -----------------------------------------------------------------------------------------//
    assign CPU_data = CPU_readM ? 
           /* CPU_readM = 1 */   Cache_Data_Bank[CPU_idx][CPU_block_offset] : 
           /* CPU_readM = 0 */   `WORD_SIZE'hz;
    assign MEMORY_data = MEMORY_writeM ? MEMORY_write_data : `MEMORY_BANDWIDTH'hz;
    always @(*) begin
        // if Busy : read/write from/to memory module ---------------------------------------------------------//
        if(Busy == 1'b1) begin
            // If cache line is dirty, line should be written back to memory before getting erased ---//
            if(Cache_Dirty[Ongoing_idx]) begin
                MEMORY_readM = 1'b0;
                MEMORY_writeM = 1'b1;
                MEMORY_address = {Cache_Tag_Bank[Ongoing_tag], Ongoing_idx, 2'b00}; // use address of data that will be erased
                MEMORY_write_data = {Cache_Data_Bank[Ongoing_idx][0], Cache_Data_Bank[Ongoing_idx][1], Cache_Data_Bank[Ongoing_idx][2], Cache_Data_Bank[Ongoing_idx][3]};
            end else begin
                MEMORY_readM = 1'b1;
                MEMORY_writeM = 1'b0;
                MEMORY_address = {Ongoing_tag, Ongoing_idx, 2'b00};
            end
        // if not busy : nothing going on with memory<->cache -------------------------------------------------//
        end else begin
            MEMORY_readM = 1'b0;
            MEMORY_writeM = 1'b0;
        end
    end

    //######################################################################################################################//
    integer idx, widx;
    always @(posedge clk) begin
        // reset values ---------------------------------------------------------------------------------------//
        if(reset_n == 1'b0) begin
            Ongoing_address =  `WORD_SIZE'hffff;
            Ongoing_state = `CACHE_IDLE;
            MEMORY_delay_count = `MEMORY_DELAY_READY;
            // initialize CACHE Banks
            for(idx = 0; idx < `CACHE_HEIGHT; idx = idx + 1) begin
                Cache_Tag_Bank[idx] = `cache_tag_bitlen'b0;
                Cache_Valid[idx] = 1'b0;
                Cache_Dirty[idx] = 1'b0;
                for(widx = 0; widx < `CACHE_BLOCK_SIZE_WORD; widx = widx + 1) begin
                    Cache_Data_Bank[idx][widx] = `WORD_SIZE'b0;
                end
            end
            Busy = 1'b0;
        // Not reset ------------------------------------------------------------------------------------------//
        end else begin
            if(MEMORY_delay_count != `MEMORY_DELAY_READY && MEMORY_available == 1'b1) MEMORY_delay_count = MEMORY_delay_count + 1;

            if(Busy == 1'b1 && MEMORY_available == 1'b1) begin
                if(MEMORY_delay_count == `MEMORY_DELAY_READY) begin
                    if(Cache_Dirty[Ongoing_idx] == 1'b1) begin
                        Cache_Dirty[Ongoing_idx] = 1'b0;
                    end else begin
                        Cache_Tag_Bank[Ongoing_idx] = Ongoing_tag;
                        Cache_Valid[Ongoing_idx] = 1'b1;
                        Busy = 1'b0;
                    end
                    {Cache_Data_Bank[Ongoing_idx][0], Cache_Data_Bank[Ongoing_idx][1], Cache_Data_Bank[Ongoing_idx][2], Cache_Data_Bank[Ongoing_idx][3]} = MEMORY_data;
                end
            end else begin
                // write hit -> write back : just write on cache -------------------------------//
                if(CPU_writeM == 1'b1 && Hit == 1'b1) begin
                    Cache_Data_Bank[CPU_idx][CPU_block_offset] = CPU_data;
                    Cache_Dirty[CPU_idx] = 1'b1;
                // Handle Miss -----------------------------------------------------------------//
                end else if(Hit == 1'b0 && (CPU_readM == 1'b1 || CPU_writeM == 1'b1)) begin
                    if(MEMORY_available == 1'b1) begin
                        Ongoing_address = CPU_address;
                        MEMORY_delay_count = 0;
                        Busy = 1'b1;
                    end
                end
            end
        end
    end
endmodule