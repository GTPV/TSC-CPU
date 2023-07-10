///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: control.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: Control module under cpu from "cpu.v"

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION
module Control(
    input [`WORD_SIZE-1 : 0] inst,

    // Control signals : Generated once, flow through pipeline, used at appropriate stages
    output reg [`inst_type_bitlen-1 : 0] inst_type,
    output reg RegWrite,
    output reg [`reg_addr_bitlen-1 : 0] RegWrite_addr,
    output reg [1:0] RegWriteSrc,
    output reg MemRead,
    output reg MemWrite,
    //output reg ALUSrcA,
    output reg [1:0] ALUSrcB,
    output reg [3:0] ALUOp,
    output output_active,
    output is_halted,

    // Used for hazard detection : Generated once, flow through pipeline
    output reg rs_used,
    //output reg [1:0] rs_used_stage,
    output reg rt_used
    //output reg [1:0] rt_used_stage,
    //output reg [1:0] write_value_produce_stage,
);
    //////////////////////////////////////////////////////////////////////////////
    // Generating control signals once

    // parsed instructions
    wire [`opcode_bitlen - 1 : 0] opcode;
    wire [`reg_addr_bitlen - 1 : 0] rs;
    wire [`reg_addr_bitlen - 1 : 0] rt;
    wire [`reg_addr_bitlen - 1 : 0] rd;
    wire [`R_func_bitlen - 1 : 0] R_func_code;
    wire [`WORD_SIZE - 1 : 0] I_imm_off;
    wire [`J_target_addr_bitlen - 1 : 0] J_target_addr;
    // parse instructions
    assign opcode = inst[`opcode_idx_left : `opcode_idx_right];
    assign rs = inst[`rs_idx_left : `rs_idx_right];
    assign rt = inst[`rt_idx_left : `rt_idx_right];
    assign rd = inst[`rd_idx_left : `rd_idx_right];
    assign R_func_code = inst[`R_func_idx_left : `R_func_idx_right];
    assign I_imm_off = {{8{inst[`imm_off_idx_left]}}, inst[`imm_off_idx_left : `imm_off_idx_right]}; // sign-extend
    assign J_target_addr = inst[`target_addr_idx_left : `target_addr_idx_right];
    // instruction type - parsed from inst_type
    reg R_Arithmetic;       // R-type Arithmetic instructions : IF-ID-EX-WB
    reg R_Halt;    // R-type Halt instruction : IF-ID-halt(stay ID)
    reg R_Output;     // R-type other instructions : IF-ID
    reg I_Arithmetic;       // I-type Arithmetic instructions : IF-ID-EX-WB
    reg I_Memory;     // I-type Memory instructions : IF-ID-EX-MEM or IF-ID-EX-MEM-WB
    reg I_Branch;  // I-type Branch instructions : IF-ID-EX
    reg J_Jump;    // J-type Jump instructions : IF-ID
    reg R_JumpReg; // Jump using register value
    reg Jump_Link; // Jump with link
    // identify instruction type
    always @(*) begin
        R_Arithmetic = (opcode == `OPCODE_Rtype) 
                        && 
                        (   (R_func_code == `FUNC_ADD)
                        ||  (R_func_code == `FUNC_SUB)
                        ||  (R_func_code == `FUNC_AND)
                        ||  (R_func_code == `FUNC_ORR)
                        ||  (R_func_code == `FUNC_NOT)
                        ||  (R_func_code == `FUNC_TCP)
                        ||  (R_func_code == `FUNC_SHL)
                        ||  (R_func_code == `FUNC_SHR)
                        );
        R_Halt = (opcode == `OPCODE_Rtype) && (R_func_code == `FUNC_HLT);
        R_Output = (opcode == `OPCODE_Rtype) && ((R_func_code == `FUNC_WWD));
        R_JumpReg = (opcode == `OPCODE_Rtype) 
                    && 
                    ((R_func_code == `FUNC_JPR) || (R_func_code == `FUNC_JRL));
        I_Arithmetic = (  (opcode == `OPCODE_ADI)
                    || (opcode == `OPCODE_ORI)
                    || (opcode == `OPCODE_LHI)
                    );
        I_Memory = (opcode == `OPCODE_LWD) || (opcode == `OPCODE_SWD);
        I_Branch = ( (opcode == `OPCODE_BNE)
                    ||  (opcode == `OPCODE_BEQ)
                    ||  (opcode == `OPCODE_BGZ)
                    ||  (opcode == `OPCODE_BLZ)
                    );
        J_Jump = (opcode == `OPCODE_JMP) || (opcode == `OPCODE_JAL);
        Jump_Link = (opcode == `OPCODE_JAL)
                    ||
                    ((opcode == `OPCODE_Rtype)&&(R_func_code == `FUNC_JRL));
        inst_type = {R_Arithmetic, R_Halt, R_Output, R_JumpReg, I_Arithmetic, I_Memory, I_Branch, J_Jump, Jump_Link};

/*         if(R_Arithmetic)        inst_type = `INST_TYPE_R_ALU;
        else if(R_Halt)         inst_type = `INST_TYPE_HALT;
        else if(R_Output)       inst_type = `INST_TYPE_OUTPUT;
        else if(R_JumpReg)      inst_type = `INST_TYPE_JR;
        else if(I_Arithmetic)   inst_type = `INST_TYPE_I_ALU;
        else if(I_Memory) begin
            case(opcode)
                `OPCODE_LWD :   inst_type = `INST_TYPE_LWD;
                `OPCODE_SWD :   inst_type = `INST_TYPE_SWD;
            endcase
        end else if(I_Branch)   inst_type = `INST_TYPE_BRANCH;
        else if(J_Jump)         inst_type = `INST_TYPE_JUMP;
        else                    inst_type = `INST_TYPE_NOP; */
    end

    // RegWrite_addr assignment
    always @(*) begin
        if(R_Arithmetic)                RegWrite_addr = rd;
        else if(I_Arithmetic)           RegWrite_addr = rt;
        else if(opcode == `OPCODE_LWD)  RegWrite_addr = rt;
        else if(R_Output)               RegWrite_addr = rt;
        else if(Jump_Link)              RegWrite_addr = 2'b10;
        else                            RegWrite_addr = 2'bx;
    end

    // RegWrite assignment : assert for instructions those who takes WB stage
    always @(*) begin
        RegWrite = (    (R_Arithmetic)
                    ||  (I_Arithmetic)
                    ||  (opcode == `OPCODE_LWD)
                    ||  (Jump_Link)
                    );
    end

    // RegWriteSrc assignment
    always @(*) begin
        if(opcode == `OPCODE_LWD)   RegWriteSrc = `RegWriteSrc_MEM;
        else if(Jump_Link)          RegWriteSrc = `RegWriteSrc_PC;
        else                        RegWriteSrc = `RegWriteSrc_ALU;
    end

    // MemRead assignment
    always @(*) begin
        if(opcode == `OPCODE_LWD) MemRead = 1'b1;
        else MemRead = 1'b0;
    end

    // MemWrite assignment
    always @(*) begin
        if(opcode == `OPCODE_SWD) MemWrite = 1'b1;
        else MemWrite = 1'b0;
    end

    // ALU_main always get RF_rs_value for SrcA : branch target calculation using Seq_PC is done by another instance of ALU at ID stage
    // ALUSrcA assignment
/*  always @(*) begin
        if(Jump_Link) ALUSrcA = `ALUSrcA_Seq_PC;
        else ALUSrcA = `ALUSrcA_RF_rs;
    end */

    // ALUSrcB assignment
    always @(*) begin
        if(R_Arithmetic) ALUSrcB = `ALUSrcB_RF_rt;
        else if(I_Arithmetic) ALUSrcB = `ALUSrcB_I_OFFSET;
        else if(I_Memory) ALUSrcB = `ALUSrcB_I_OFFSET;
        else if(I_Branch) begin
            if(opcode == `OPCODE_BEQ || opcode == `OPCODE_BNE) ALUSrcB = `ALUSrcB_RF_rt;
            else ALUSrcB = `ALUSrcB_ZERO;
        end
    end

    // ALUOp assignment
    always @(*) begin
        if(R_Arithmetic) begin
            case(R_func_code)
                `FUNC_ADD : ALUOp = `OP_ADD;
                `FUNC_SUB : ALUOp = `OP_SUB;
                `FUNC_AND : ALUOp = `OP_AND;
                `FUNC_ORR : ALUOp = `OP_ORR;
                `FUNC_NOT : ALUOp = `OP_NOT;
                `FUNC_TCP : ALUOp = `OP_TCP;
                `FUNC_SHL : ALUOp = `OP_SHL;
                `FUNC_SHR : ALUOp = `OP_SHR;
            endcase
        end else if(I_Arithmetic) begin
            case(opcode)
                `OPCODE_ADI : ALUOp = `OP_ADD;
                `OPCODE_ORI : ALUOp = `OP_ORR;
                `OPCODE_LHI : ALUOp = `OP_LHI;
            endcase
        end else if(I_Memory) begin
            ALUOp = `OP_ADD;
        end else if(I_Branch) begin
            ALUOp = `OP_SUB;
        end else begin
            ALUOp = 4'bx;
        end
    end

    // output_active assignment
    assign output_active = R_Output;

    // is_halted assignment
    assign is_halted = R_Halt;



    //////////////////////////////////////////////////////////////////////////////
    // Signals for hazard detection (Data use stage, Data produce stage, RF addr)

    // rs
    // rs_used : if rs is used
    always @(*) begin
        rs_used = ( R_Arithmetic
                ||  R_JumpReg
                ||  R_Output
                ||  I_Arithmetic
                ||  I_Branch
                ||  I_Memory
                );
    end
/*     // rs_used_stage : when rs is used - without forwarding, ALL ID stage
    always @(*) begin
        if(~rs_used) begin
            rs_used_stage = 2'bx;
        end else if(`DATA_FORWARDING_ENABLED) begin // Data Forwarding enabled
            rs_used_stage = R_Arithmetic    ?   `STAGE_EX :
                            R_JumpReg       ?   `STAGE_ID :
                            R_Output        ?   `STAGE_WB :
                            I_Branch        ?   `STAGE_EX :
                            I_Memory        ?   `STAGE_EX : `STAGE_ID;
        end else begin // Without data forwarding : rs is used at ID stage always
            rs_used_stage = `STAGE_ID;
        end
    end
 */
    // rt
    // rt_used : if rt is used
    always @(*) begin
        rt_used = ( (R_Arithmetic)
                ||  (I_Memory)
                ||  (opcode == `OPCODE_BEQ)
                ||  (opcode == `OPCODE_BNE)
                );
    end
/*     // rt_used : when rt is used - without forwarding, ALL ID stage
    always @(*) begin
        if(~rt_used) begin
            rt_used_stage = 2'bx;
        end else if(`DATA_FORWARDING_ENABLED) begin
            rt_used_stage = (opcode == `OPCODE_SWD) ? `STAGE_MEM : `STAGE_EX;
        end else begin // without data forwarding : rt is used at ID stage always
            rt_used_stage = `STAGE_ID;
        end
    end
 */
/*     // stage of write data produce(next instruction can read) - without forwarding, ALL WB stage
    always @(*) begin
        if(~write_register) begin
            write_value_produce_stage = 2'bx;
        end else if(`DATA_FORWARDING_ENABLED) begin
            write_value_produce_stage = (R_Arithmetic || I_Arithmetic) ? `STAGE_EX :
                                        Jump_Link ? `STAGE_ID : 
                                                    `STAGE_MEM;
        end else begin // without forwarding : always WB stage
            write_value_produce_stage = `STAGE_WB;
        end
    end
 */
endmodule