`timescale 1ns/100ps

`include "opcodes.v"
`include "constants.v"

module cpu (
    output readM, // read from memory
    output writeM, // write to memory
    output [`WORD_SIZE-1:0] address, // current address for data
    inout [`WORD_SIZE-1:0] data, // data being input or output
    input inputReady, // indicates that data is ready from the input port
    input reset_n, // active-low RESET signal
    input clk, // clock signal
    
    // for debuging/testing purpose
    output [`WORD_SIZE-1:0] num_inst, // number of instruction during execution
    output [`WORD_SIZE-1:0] output_port, // this will be used for a "WWD" instruction
    output is_halted // 1 if the cpu is halted
);
    reg [`WORD_SIZE-1:0] num_inst;
    reg reset_cycle;

    // instruction
    wire [`WORD_SIZE - 1 : 0] inst_addr; // PC value
    reg [`WORD_SIZE - 1 : 0] instruction; // IR
    wire [`WORD_SIZE - 1 : 0] inst_seq_addr; // PC value for next seq
    wire [`WORD_SIZE - 1 : 0] next_target;
    // parsed instruction
    wire [`opcode_bitlen - 1 : 0] opcode;
    wire [`R_func_bitlen - 1 : 0] R_func_code;
    wire [`WORD_SIZE - 1 : 0] I_imm_off;
    wire [`J_target_addr_bitlen - 1 : 0] J_target_addr;
    // parse instruction
    assign opcode = instruction[`opcode_idx_left : `opcode_idx_right];
    assign R_func_code = instruction[`R_func_idx_left : `R_func_idx_right];
    assign I_imm_off = { {8{instruction[`imm_off_idx_left]}}, instruction[`imm_off_idx_left : `imm_off_idx_right]};
    assign J_target_addr = instruction[`target_addr_idx_left : `target_addr_idx_right];

    // control signal wires
    wire PVSWrite; // PC update enable
    wire PCWrite; // PC store new value
    wire [1:0] PCSource; // PC source select MUX
    
    wire Jump_Register; // Jump to address written in register value : JRL, JPR

    wire IorD;
    wire IRWrite;

    wire [1:0] RegWriteSrc;
    wire RegWrite;
    wire [1:0] RegDst;

    wire [3:0] ALUOp;
    wire [1:0] ALUSrcA;
    wire [1:0] ALUSrcB;

    wire output_active;

    // datapath wires
    reg [`WORD_SIZE - 1 : 0] MemData;
    wire [`WORD_SIZE - 1 : 0] ALUOut_C;
    wire [`WORD_SIZE - 1 : 0] ALUOut_data;
    wire [`WORD_SIZE - 1 : 0] ALU_Cmp;
    wire [`WORD_SIZE - 1 : 0] output_data; // for WWD
    wire [`WORD_SIZE - 1 : 0] RF_addr;
    wire [`WORD_SIZE - 1 : 0] RF_data;

    assign next_target = (  (PCSource == `PCSrc_Seq) ? ALUOut_C:
                            (PCSource == `PCSrc_Offset) ? ALUOut_C:
                            (Jump_Register) ? RF_addr : {inst_addr[`WORD_SIZE - 1 : `target_addr_idx_left + 1], J_target_addr});

    assign output_port = output_active ? output_data : `WORD_SIZE'bz; // for WWD
    assign data = writeM ? RF_data : `WORD_SIZE'bz;
    assign address = IorD ? ALUOut_data : inst_addr;

    always @(posedge clk) begin
        if(reset_n == 1'b0) begin
            num_inst = 1;
            reset_cycle = 1'b1;
        end else if(reset_cycle) begin
            reset_cycle = 1'b0;
        end else if(PVSWrite) begin
            num_inst = num_inst + 1;
        end
    end

    always @(posedge inputReady) begin
        if(IorD == 1'b0) begin // IorD=1'b0 : memory->IR
            instruction = data;
        end else begin
            MemData = data;
        end
    end

    // ProgramCounter
    PC PC_UUT(
        .clk (clk),
        .reset_n (reset_n),
        .next_target (next_target),
        .PVSWrite (PVSWrite),
        .PCWrite (PCWrite),
        .PCSource (PCSource),
        .next_Seq_addr (inst_seq_addr),
        .inst_addr (inst_addr)
    );

    // Control unit
    Control CT_UUT(
        .clk (clk),
        .reset_n (reset_n),
        .opcode (opcode),
        .R_func_code (R_func_code),
        .ALU_Cmp (ALU_Cmp),
        .PVSWrite (PVSWrite),
        .PCWrite (PCWrite),
        .PCSource (PCSource),
        .Jump_Register (Jump_Register),
        .IorD (IorD),
        .readM (readM),
        .writeM (writeM),
        .IRWrite (IRWrite),
        .RegWriteSrc (RegWriteSrc),
        .RegWrite (RegWrite),
        .RegDst (RegDst),
        .ALUOp (ALUOp),
        .ALUSrcA (ALUSrcA),
        .ALUSrcB (ALUSrcB),
        .is_halted (is_halted),
        .output_active (output_active)
    );

    // Datapath unit
    DP DP_UUT(
        .clk (clk),
        .reset_n (reset_n),
        .inst_addr (inst_addr),
        .inst_seq_addr (inst_seq_addr),
        .instruction (instruction),
        .IRWrite (IRWrite),
        .MemData (MemData),
        .RegWriteSrc (RegWriteSrc),
        .ALUOp (ALUOp),
        .ALUSrcA (ALUSrcA),
        .ALUSrcB (ALUSrcB),
        .RegWrite (RegWrite),
        .RegDst (RegDst),
        .output_data(output_data),
        .RF_addr (RF_addr),
        .RF_data (RF_data),
        .ALU_out_C (ALUOut_C),
        .ALUOut (ALUOut_data),
        .ALU_Cmp (ALU_Cmp)
    );
endmodule
