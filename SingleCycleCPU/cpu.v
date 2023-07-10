///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: cpu.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: Top module for SingleCycleCPU
// Submodule :  "Control.v" Control Control_UUT,
//              "Datapath.v" DP DP_UUT,
//              "ProgramCounter.v" PC PC_UUT,

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION
module cpu (

    output readM,                           // read from memory
    output [`WORD_SIZE - 1:0] address,      // current address for data

    inout [`WORD_SIZE - 1:0] data,          // data being input or output
    input inputReady,                       // indicates that data is ready from the input port
    input reset_n,                          // active-low RESET signal
    input clk,                              // clock signal
  
    // for debuging/testing purpose
    output [`WORD_SIZE - 1:0] num_inst,     // number of instruction during execution
    output [`WORD_SIZE - 1:0] output_port   // this will be used for a "WWD" instruction
);

    reg [`WORD_SIZE - 1 : 0] num_instruction;

    // instruction fetch
    wire [`WORD_SIZE - 1 : 0] instruction_addr; // address of instruction to read(Program Counter)
    reg [`WORD_SIZE - 1 : 0] instruction;       // fetched Instruction
    reg [63:0] counter;         // counter for clk cycle
    reg [63:0] fetched_cycle;   // save counter at which current instruction was fetched
    wire fetch_completed;       // if instruction was fetched this cycle
    reg readM_sync;             // request from cpu for new instruction(synced with clk)

    // parsed instruction
    wire [`opcode_length - 1 : 0] opcode;
    wire [`func_code_length - 1 : 0] func_code;
    wire [`Immediate_length - 1 : 0] I_immediate;
    wire [`Jump_target_length - 1 : 0] J_target;
    // parse instruction
    assign opcode = instruction[`opcode_left : `opcode_right];
    assign func_code = instruction[`func_code_left : `func_code_right];
    assign I_immediate = { {8{instruction[`immediate_left]}}, instruction[`immediate_left : `immediate_right]}; //Sign-extend
    assign J_target = instruction[`target_left : `target_right];
  
    // port : control_unit -> PC
    wire Jump;

    // ports : Control -> DP
    wire RegDst, ALUSrc, RegWrite, isWWD;
    wire [3:0] ALUOp;

    // DP module ports
    wire [`WORD_SIZE - 1 : 0] output_data;

    // output signals
    reg [`WORD_SIZE - 1 : 0] output_save; // save output_data from DP
    always @(posedge clk) begin
        output_save = output_data;
    end
    assign output_port = output_save; // only used for WWD
    assign address = instruction_addr; // PC to memory
    assign num_inst = num_instruction;

    // check if fetch was completed on this cycle
    // if already fetched -> readM = 0
    assign fetch_completed = (counter == fetched_cycle);
    assign readM = fetch_completed ? 0 : readM_sync;

    always @(posedge clk) begin
        if(reset_n == 1'b0) begin // reset
            num_instruction = 0;
            counter = 63'b0;
            readM_sync = 0;
        end
        else begin
            num_instruction = num_instruction + 1;
            counter = counter + 1; // increment cycle counter
            readM_sync = 1; // request new instruction
        end
    end

    //if inputReady goes high -> "fetch" (receive new instruction from memory)
    always @(posedge inputReady or negedge reset_n) begin
        if(reset_n == 1'b0) begin // reset
            fetched_cycle = -1;
        end
        else begin
            instruction = data; // fetch
            fetched_cycle = counter; // remember that cpu fetched this cycle
        end
    end

    // ProgramCounter module
    PC PC_UUT(
        .clk (clk),
        .reset_n (reset_n),
        .Jump (Jump),
        .J_target (J_target),
        .instruction_addr (instruction_addr)
    );

    // Control module
    Control Control_UUT (
        .reset_n (reset_n),
        .opcode (opcode),
        .func_code (func_code),
        .RegDst (RegDst),
        .Jump (Jump),
        .ALUOperation (ALUOp),
        .ALUSrc (ALUSrc),
        .RegWrite (RegWrite),
        .isWWD (isWWD)
        ); 
        
    // Datapath module
    DP DP_UUT (
        .clk(clk),
        .reset_n (reset_n),
        .instruction (instruction),
        .output_data (output_data),
        
        .RegDst (RegDst),
        .RegWrite (RegWrite),
        .ALUSrc (ALUSrc),
        .ALUOp (ALUOp),    
        .Jump (Jump),
        .isWWD (isWWD)
        
        );    

endmodule
//////////////////////////////////////////////////////////////////////////
