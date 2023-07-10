///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: Cache.v
// Author: gangtaeng_parangvo@snu.ac.kr

// INCLUDES
`include "opcodes.v"

module Cache(
    input clk,
    input reset_n,
    input stall,

    // ports from/to CPU module
    input CPU_readM,
    input CPU_writeM,
    input [`WORD_SIZE-1 : 0] CPU_address,
    inout [`WORD_SIZE-1 : 0] CPU_data,

    // ports from/to MEMORY module
    output reg MEMORY_readM,
    output reg MEMORY_writeM,
    output reg [`WORD_SIZE-1 : 0] MEMORY_address,
    inout [`WORD_SIZE-1 : 0] MEMORY_data,

    output reg CPU_ready
);

    reg [`WORD_SIZE-1 : 0] MEMORY_write_data;

    reg [`WORD_SIZE-1 : 0] Ongoing_address;
    reg [1:0] Ongoing_state;

    /* assign CPU_data = CPU_readM ? (`CACHE_ENABLED ? cache_table[][] : MEMORY_data) : `WORD_SIZE'hz; */
    assign CPU_data = CPU_readM ? MEMORY_data : `WORD_SIZE'hz;
    // assign CPU_ready
    always @(*) begin
        if(`CACHE_ENABLED == 1'b0) begin // CACHE disabled(baseline)
            if(CPU_readM == 1'b0 && CPU_writeM == 1'b0) CPU_ready = 1'b1;
            else if(Ongoing_address == CPU_address) begin
                if((Ongoing_state == `CACHE_READING && CPU_readM) || (Ongoing_state == `CACHE_WRITING && CPU_writeM)) CPU_ready = 1'b1;
                else CPU_ready = 1'b0;
            end else begin
                CPU_ready = 1'b0;
            end
        end else begin // CACHE enabled
        end
    end

    assign MEMORY_data = MEMORY_writeM ? MEMORY_write_data : `WORD_SIZE'hz;

    always @(*) begin
        if(`CACHE_ENABLED == 1'b0) begin
            MEMORY_readM = CPU_readM;
            MEMORY_writeM = CPU_writeM;
            MEMORY_address = CPU_address;
            MEMORY_write_data = CPU_data;
        end else begin
        end
    end

    always @(posedge clk) begin
        if(`CACHE_ENABLED == 1'b0) begin
            Ongoing_address = (CPU_readM || CPU_writeM) ? CPU_address : `WORD_SIZE'hffff;
            Ongoing_state = CPU_readM ? `CACHE_READING :
                            CPU_writeM ? `CACHE_WRITING : `CACHE_IDLE;
        end else begin
        end
    end
endmodule