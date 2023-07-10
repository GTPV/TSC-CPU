///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: control.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: Control module under cpu from "cpu.v"

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION
module Control(
    input clk,
    input reset_n,
    input [`opcode_bitlen - 1 : 0] opcode,
    input [`R_func_bitlen - 1 : 0] R_func_code,
    input [1:0] ALU_Cmp,

    output reg PVSWrite, // PC update enable
    output reg PCWrite, // save next PC candidate
    output reg [1:0] PCSource,
    output reg Jump_Register,
    output reg IorD, // instruction or data (IR or MDR)
    output reg readM, // read from memory
    output writeM, // write to memory
    output reg IRWrite, // instruction register enable
    output reg [1:0] RegWriteSrc, // which source to write on RegisterFile
    output reg RegWrite, // RegisterFile write enable
    output reg [1:0] RegDst, // which address of RegisterFile to write on
    output reg [3:0] ALUOp, // ALU operation code
    output reg [1:0] ALUSrcA, // ALU source A
    output reg [1:0] ALUSrcB, // ALU source B
    output is_halted, // CPU halt
    output output_active // output active
);

    reg reset_cycle;

    reg [2:0] stage, next_stage;

    // instruction types
    wire R_Arithmetic;       // R-type Arithmetic instructions : IF-ID-EX-WB
    wire R_Halt;    // R-type Halt instruction : IF-ID-halt(stay ID)
    wire R_etc;     // R-type other instructions : IF-ID
    wire I_Arithmetic;       // I-type Arithmetic instructions : IF-ID-EX-WB
    wire I_Memory;     // I-type Memory instructions : IF-ID-EX-MEM or IF-ID-EX-MEM-WB
    wire I_Branch;  // I-type Branch instructions : IF-ID-EX
    wire J_Jump;    // J-type Jump instructions : IF-ID

    // identify instruction type
    assign R_Arithmetic = (opcode == `OPCODE_Rtype) 
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
    assign R_Halt = (opcode == `OPCODE_Rtype) && (R_func_code == `FUNC_HLT);
    assign R_etc = (opcode == `OPCODE_Rtype)
                    &&
                    (   (R_func_code == `FUNC_WWD)
                    ||  (R_func_code == `FUNC_JPR)
                    ||  (R_func_code == `FUNC_JRL)
                    );
    assign I_Arithmetic = (  (opcode == `OPCODE_ADI)
                 || (opcode == `OPCODE_ORI)
                 || (opcode == `OPCODE_LHI)
                );
    assign I_Memory = (opcode == `OPCODE_LWD) || (opcode == `OPCODE_SWD);
    assign I_Branch = ( (opcode == `OPCODE_BNE)
                    ||  (opcode == `OPCODE_BEQ)
                    ||  (opcode == `OPCODE_BGZ)
                    ||  (opcode == `OPCODE_BLZ)
                    );
    assign J_Jump = (opcode == `OPCODE_JMP) || (opcode == `OPCODE_JAL);

    // output assignment
    assign output_active = ((stage == `STAGE_ID)
                            &&
                            (opcode == `OPCODE_Rtype)
                            &&
                            (R_func_code == `FUNC_WWD)
                        );
    assign is_halted = ((stage == `STAGE_ID) && R_Halt) ? 1'b1 : 1'b0;
    assign writeM = (stage == `STAGE_MEM) && (opcode == `OPCODE_SWD);

    // PVSWrite - when a instruction is finished (next stage == IF), (write signal : @negedge, is_halted->0)
    always @(negedge clk) begin
        PVSWrite = (next_stage == `STAGE_IF) ? 1'b1 : 1'b0;

        if(is_halted) PVSWrite = 1'b0;
    end

    // PCWrite - write signals : @ negedge, is_halted -> 0
    always @(negedge clk) begin
        if(stage == `STAGE_IF || (stage == `STAGE_ID && I_Branch)) begin
            PCWrite = 1'b1;
        end else begin
            PCWrite = 1'b0;
        end

        if(is_halted) PCWrite = 1'b0;
    end

    // PCSource
    always @(*) begin
        if(stage == `STAGE_IF) begin
            PCSource = `PCSrc_Seq;
        end else if(stage == `STAGE_ID) begin
            if(I_Branch) PCSource = `PCSrc_Offset;
            else if(J_Jump) PCSource = `PCSrc_Uncond_Jump;
            else if(opcode == `OPCODE_Rtype && (R_func_code == `FUNC_JPR || R_func_code == `FUNC_JRL)) PCSource = `PCSrc_Uncond_Jump;
            else if(opcode == `OPCODE_Rtype && R_func_code == `FUNC_WWD) PCSource = `PCSrc_Seq;
            else PCSource = `PCSrc_Offset;
        end else if(stage == `STAGE_EX) begin
            case(opcode)
                `OPCODE_BNE : PCSource = {1'b0, ALU_Cmp != 2'b00};
                `OPCODE_BEQ : PCSource = {1'b0, ALU_Cmp == 2'b00};
                `OPCODE_BGZ : PCSource = {1'b0, ALU_Cmp == 2'b10};
                `OPCODE_BLZ : PCSource = {1'b0, ALU_Cmp == 2'b11};
                default : PCSource = `PCSrc_Seq;
            endcase
        end else PCSource = `PCSrc_Seq;
    end

    // Jump_Register
    always @(*) begin
        if(opcode == `OPCODE_Rtype && (R_func_code == `FUNC_JPR || R_func_code == `FUNC_JRL)) begin
            Jump_Register = 1'b1;
        end else begin
            Jump_Register = 1'b0;
        end
    end

    // IorD
    always @(*) begin
        IorD = (stage == `STAGE_IF) ? 1'b0 : 1'b1;
    end

    // readM
    always @(*) begin
        if(stage == `STAGE_IF) begin
            readM = 1'b1;
        end else if(stage == `STAGE_MEM && opcode == `OPCODE_LWD) begin
            readM = 1'b1;
        end else readM = 1'b0;
    end

    // IRWrite
    always @(*) begin
        IRWrite = (stage == `STAGE_IF) ? 1'b1 : 1'b0;
    end

    // RegWriteSrc
    always @(*) begin
        if(opcode == `OPCODE_LWD) begin
            RegWriteSrc = `RegWriteSrc_MEM;
        end else if(opcode == `OPCODE_JAL || (opcode == `OPCODE_Rtype && R_func_code == `FUNC_JRL)) begin
            RegWriteSrc = `RegWriteSrc_PC;
        end else begin
            RegWriteSrc = `RegWriteSrc_ALU;
        end
    end

    // RegWrite - write signals are @ negedge
    always @(negedge clk) begin
        if(     (stage == `STAGE_WB)
            ||  (opcode == `OPCODE_JAL && stage == `STAGE_ID)
            ||  (opcode == `OPCODE_Rtype && R_func_code == `FUNC_JRL && stage == `STAGE_ID)
            ) begin
            RegWrite = 1'b1;
        end else begin
            RegWrite = 1'b0;
        end
    end

    // RegDst : 0->rt, 1->rd
    always @(*) begin
        if(opcode == `OPCODE_JAL && stage == `STAGE_ID) begin
            RegDst = `RegDst_2;
        end else if(opcode == `OPCODE_Rtype) begin
            if(R_func_code == `FUNC_JRL && stage == `STAGE_ID) begin
                RegDst = `RegDst_2;
            end else begin
                RegDst = `RegDst_rd;
            end
        end else begin
            RegDst = `RegDst_rt;
        end
    end

    // ALUOp
    always @(*) begin
        if(stage == `STAGE_IF) ALUOp = `OP_ADD;
        else if(stage == `STAGE_ID) ALUOp = `OP_ADD;
        else begin
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
                    default : ALUOp = `OP_ADD;
                endcase
            end else if(I_Arithmetic) begin
                case(opcode)
                    `OPCODE_ADI : ALUOp = `OP_ADD;
                    `OPCODE_ORI : ALUOp = `OP_ORR;
                    `OPCODE_LHI : ALUOp = `OP_LHI;
                    default : ALUOp = `OP_ADD;
                endcase
            end else if(I_Memory) begin
                ALUOp = `OP_ADD;
            end else if(I_Branch) begin
                ALUOp = `OP_SUB;
            end else begin
                ALUOp = 4'bx;
            end
        end
    end

    // ALUSrcA
    always @(*) begin
        case(stage)
            `STAGE_IF : ALUSrcA = `ALUSrcA_PC;
            `STAGE_ID : ALUSrcA = `ALUSrcA_Seq_PC;
            default :   ALUSrcA = `ALUSrcA_RF_rs;
        endcase
    end

    // ALUSrcB
    always @(*) begin
        case(stage)
            `STAGE_IF : ALUSrcB = `ALUSrcB_Seq;
            `STAGE_ID : ALUSrcB = `ALUSrcB_I_OFFSET;
            `STAGE_EX : begin
                if(R_Arithmetic) begin
                    ALUSrcB = `ALUSrcB_RF_rt;
                end else if(I_Arithmetic || I_Memory) begin
                    ALUSrcB = `ALUSrcB_I_OFFSET;
                end else if(I_Branch) begin
                    case(opcode)
                        `OPCODE_BNE : ALUSrcB = `ALUSrcB_RF_rt;
                        `OPCODE_BEQ : ALUSrcB = `ALUSrcB_RF_rt;
                        `OPCODE_BGZ : ALUSrcB = `ALUSrcB_ZERO;
                        `OPCODE_BLZ : ALUSrcB = `ALUSrcB_ZERO;
                        default : ALUSrcB = `ALUSrcB_RF_rt;
                    endcase
                end else begin
                    ALUSrcB = `ALUSrcB_RF_rt;
                end
            end
            default : ALUSrcB = `ALUSrcB_RF_rt;
        endcase
    end

    // next stage decision
    always @(*) begin
        case(stage)
            `STAGE_IF : begin
                next_stage = `STAGE_ID;
            end
            `STAGE_ID : begin
                if(R_Halt) begin
                    next_stage = `STAGE_ID;
                end else if(R_etc || J_Jump) begin
                    next_stage = `STAGE_IF;
                end else begin
                    next_stage = `STAGE_EX;
                end
            end
            `STAGE_EX : begin
                if(I_Branch) begin
                    next_stage = `STAGE_IF;
                end else if(I_Memory) begin
                    next_stage = `STAGE_MEM;
                end else begin
                    next_stage = `STAGE_WB;
                end
            end
            `STAGE_MEM : begin
                next_stage = (opcode == `OPCODE_LWD) ? `STAGE_WB : `STAGE_IF;
            end
            `STAGE_WB : begin
                next_stage = `STAGE_IF; // WB is always the last stage of instruction
            end
        endcase
    end

    // stage transition
    always @(posedge clk) begin
        if(reset_n == 1'b0) begin
            reset_cycle = 1'b1;
        end else if(reset_cycle) begin
            reset_cycle = 1'b0;
            stage = `STAGE_IF;
        end else begin
            stage = next_stage;
        end
    end
endmodule