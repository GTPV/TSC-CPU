///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: control.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: Control module under cpu from "cpu.v"

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION
module Control(
    input reset_n,
    input [`opcode_length-1:0] opcode,
    input [`func_code_length-1:0] func_code,

    output reg RegDst,
    output reg Jump,
    output reg [3:0] ALUOperation,
    output reg ALUSrc,
    output reg RegWrite,
    output reg isWWD
);

    always @(*) begin
        case(opcode)
            `OPCODE_Rtype : begin // R-type instruction
                RegDst = 1'b1;
                Jump = 1'b0;
                ALUSrc = 1'b0;
                RegWrite = 1'b1;
                case(func_code)
                    `FUNC_ADD : begin
                        ALUOperation = `OP_ADD;
                        isWWD = 1'b0;
                    end
                    `FUNC_WWD : begin
                        ALUOperation = `OP_ID;
                        RegWrite = 1'b0;
                        isWWD = 1'b1;
                    end
                endcase
            end
            `OPCODE_ADI : begin // ADI instruction
                RegDst = 1'b0;
                Jump = 1'b0;
                ALUOperation =  `OP_ADD;
                ALUSrc = 1'b1;
                RegWrite = 1'b1;
                isWWD = 1'b0;
            end
            `OPCODE_LHI : begin // LHI instruction
                RegDst = 1'b0;
                Jump = 1'b0;
                ALUOperation = `OP_LHI;
                ALUSrc = 1'b1;
                RegWrite = 1'b1;
                isWWD = 1'b0;
            end
            `OPCODE_JMP : begin // JMP instruction
                RegDst = 1'b0;
                Jump = 1'b1;
                ALUOperation = `OP_ADD;
                ALUSrc = 1'b0;
                RegWrite = 1'b0;
                isWWD = 1'b0;
            end
        endcase
    end
endmodule
///////////////////////////////////////////////////////////////////////////