///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: ALUcombinational.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: ALU module with combinational circuit only

// INCLUDE
`include "opcodes.v"

// MODULE DECLARATION
module ALU(
    input [15:0] A,
    input [15:0] B,
    input Cin,
    
    input [3:0] OP,
    
    output reg [15:0] C,
    output reg Cout,
    output reg [1:0] Cmp
);
    reg Overflow;
    reg [`WORD_SIZE - 1 : 0] AddSub;

    always @(*) begin
        if(OP == `OP_ADD) {Overflow, AddSub} = {1'b0, A[15:0]} + {1'b0, B[15:0]} + Cin;
        else if(OP == `OP_SUB) {Overflow, AddSub} = {1'b0, A[15:0]} - {1'b0, B[15:0]} - Cin;
    end

    // AddSub[`WORD_SIZE - 1] : Sign-bit -> 0 : bigger, 1 : smaller
    // Cmp -> 00 : same(A==B), 10 : bigger(A>B), 11 : smaller(A<B)
    always @(*) begin
        if(OP == `OP_SUB) begin
            if(AddSub == `WORD_SIZE'b0) Cmp = 2'b00; // AddSub=0 -> same
            else Cmp = {1'b1, AddSub[`WORD_SIZE - 1]}; // 1 | (AddSub Sign-bit)
        end
        else Cmp = 2'b00;
    end

    //Combinational logic
    always @(*) begin
        Cout = Overflow;
        case(OP)
            `OP_ADD :    C = AddSub;
            `OP_SUB :    C = AddSub;
            `OP_AND :    C = (A&B);
            `OP_ORR :    C = (A|B);
            `OP_NOT :    C = ~A;
            `OP_TCP :    C = -A;
            `OP_SHL :    C = {A[14:0], 1'b0};
            `OP_SHR :    C = {A[15], A[15:1]};
            `OP_LHI :    C = {B[7:0], 8'b00000000};
            default :   C = {16'b0000000000000000};
        endcase
    end
endmodule
///////////////////////////////////////////////////////////////////////////