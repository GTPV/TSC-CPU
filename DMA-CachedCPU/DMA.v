/*************************************************
* DMA module (DMA.v)
* input: clock (CLK), bus grant (BG) signal, 
*        data from the device (edata), and DMA command (cmd)
* output: bus request (BR) signal 
*         WRITE signal
*         memory address (addr) to be written by the device, 
*         offset device offset (0 - 2)
*         data that will be written to the memory
*         interrupt to notify DMA is end
* You should NOT change the name of the I/O ports and the module name
* You can (or may have to) change the type and length of I/O ports 
* (e.g., wire -> reg) if you want 
* Do not add more ports! 
*************************************************/

// INCLUDES
`include "opcodes.v"

module DMA (
    input CLK, BG,
    input [`MEMORY_BANDWIDTH- 1 : 0] edata,
    input cmd,
    output BR, WRITE,
    output [`WORD_SIZE - 1 : 0] addr, 
    output [`MEMORY_BANDWIDTH-1 : 0] data,
    output reg [1:0] offset,
    output interrupt
);

    /* Implement your own logic */

    // assign signal aliases
    wire clk, BUS_Grant;
    reg BUS_Request;
    reg WRITE_complete;
    assign clk = CLK;
    assign BUS_Grant = BG;
    assign BR = BUS_Request;
    assign interrupt = WRITE_complete;

    reg BUS_Request_Waiting;
    reg [2:0] MEMORY_delay_count;

    // address block offset of 4 words - first 4 words = 0, second 4 words = 4, last 4 words = 8
    reg [`WORD_SIZE-1 : 0] addr_block_offset;
    assign addr = `DMA_WRITE_ADDRESS + addr_block_offset; // 0x01f4 + offset

    // Data is connected only when bus granted
    assign data = BUS_Grant ? edata : `MEMORY_BANDWIDTH'hz;
    assign WRITE = BUS_Grant ? 1'b1 : 1'bz;

    always @(posedge clk) begin
        if(MEMORY_delay_count != `MEMORY_DELAY_READY && BUS_Grant) MEMORY_delay_count = MEMORY_delay_count + 1;

        // BUS is requested -------------------------------------------------------------------------------------------//
        if(BUS_Request == 1'b1) begin
            if(BUS_Grant == 1'b1) begin
                // writing last block && writing finished(memory delay end) : end of dma operation --------//
                if(addr_block_offset == `WORD_SIZE'h8 && MEMORY_delay_count == `MEMORY_DELAY_READY) begin
                    BUS_Request = 1'b0;
                    WRITE_complete = 1'b1;
                // Cycle stealing : when a block(4 words) written, wait 1 cycle ---------------------------//
                end else if(`CYCLE_STEAL_ENABLED && MEMORY_delay_count == `MEMORY_DELAY_READY) begin
                    BUS_Request = 1'b0;
                    BUS_Request_Waiting = 1'b1;
                    addr_block_offset = addr_block_offset + `WORD_SIZE'h4;
                    offset = offset + 2'b01;
                    MEMORY_delay_count = 3'b000;
                // Block write finish : write next block --------------------------------------------------//
                end else if(MEMORY_delay_count == `MEMORY_DELAY_READY) begin
                    addr_block_offset = addr_block_offset + `WORD_SIZE'h4;
                    offset = offset + 2'b01;
                    MEMORY_delay_count = 3'b000;
                end
            end
        // Cycle stealing, data remaining : wait 1 cycle and request again --------------------------------------------//
        end else if(`CYCLE_STEAL_ENABLED && BUS_Request_Waiting == 1'b1) begin
            BUS_Request_Waiting = 1'b0;
            BUS_Request = 1'b1;
            MEMORY_delay_count = 3'b000;
        // write command from external_device -------------------------------------------------------------------------//
        end else if(cmd == 1'b1) begin
            BUS_Request = 1'b1;
            BUS_Request_Waiting = 1'b0;
            addr_block_offset = `WORD_SIZE'h0;
            offset = 2'b00;
            WRITE_complete = 1'b0;
            MEMORY_delay_count = 3'b000;
        // IDLE state : set everything to 0 ---------------------------------------------------------------------------//
        end else begin
            BUS_Request = 1'b0;
            BUS_Request_Waiting = 1'b0;
            addr_block_offset = `WORD_SIZE'h0;
            offset = 2'b00;
            WRITE_complete = 1'b0;
            MEMORY_delay_count = `MEMORY_DELAY_READY;
        end
    end

endmodule


