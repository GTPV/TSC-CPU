///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: BranchPredictor.v
// Author: gangtaeng_parangvo@snu.ac.kr
// Description: Branch prediction module under PC from "ProgramCounter.v" - 2bit Saturation/Hystersis

// INCLUDE
`include "opcodes.v"

// MACROS
`define btb_tag_bitlen (`WORD_SIZE - `btb_idx_bitlen)
`define btb_size (1 << (`btb_idx_bitlen))
`define predictor_bitlen 2

`define StrongTaken `predictor_bitlen'b11
`define WeakTaken `predictor_bitlen'b10
`define WeakNotTaken `predictor_bitlen'b01
`define StrongNotTaken `predictor_bitlen'b00

// MODULE DECLARATION

module BranchPredictor(
    input clk,
    input reset_n,

    input [`WORD_SIZE-1 : 0] pc,
    input [`WORD_SIZE-1 : 0] pc_next_seq,
    // Decoded instruction can update BTB : if instruction may jump(not seq) -> save target address
    input [`WORD_SIZE-1 : 0] ID_PC,
    input [`WORD_SIZE-1 : 0] ID_PC_next_seq,
    input [`WORD_SIZE-1 : 0] ID_Jump_Target,

    // update BHT
    input [`WORD_SIZE-1 : 0] EX_PC,
    input [`inst_type_bitlen-1 : 0] EX_inst_type,
    input Branch_Taken_Result,

    output reg [`WORD_SIZE-1 : 0] new_pc
);

    // parse pc to tag and idx
    wire [`btb_tag_bitlen-1 : 0] pc_btb_tag;
    wire [`btb_idx_bitlen-1 : 0] pc_btb_idx;
    assign pc_btb_tag = pc[`WORD_SIZE-1 : `btb_idx_bitlen];
    assign pc_btb_idx = pc[`btb_idx_bitlen-1 : 0];

    // parse Decoded PC
    wire [`btb_tag_bitlen-1 : 0] ID_PC_btb_tag;
    wire [`btb_idx_bitlen-1 : 0] ID_PC_btb_idx;
    assign ID_PC_btb_tag = ID_PC[`WORD_SIZE-1 : `btb_idx_bitlen];
    assign ID_PC_btb_idx = ID_PC[`btb_idx_bitlen-1 : 0];

    reg [`WORD_SIZE-1 : 0] BTB[`btb_size-1 : 0];
    reg [`btb_tag_bitlen-1 : 0] TagTable[`btb_size-1 : 0];
    reg [`predictor_bitlen-1 : 0] BHT[`btb_size-1 : 0];

    wire Prediction;
    wire Tag_Same;
    reg Taken;
    assign Tag_Same = (TagTable[pc_btb_idx] == pc_btb_tag);

    // for BHT update
    wire EX_Branch;
    wire EX_Jump;
    wire [`btb_tag_bitlen-1 : 0] EX_pc_btb_tag;
    wire [`btb_idx_bitlen-1 : 0] EX_pc_btb_idx;
    assign EX_pc_btb_tag = EX_PC[`WORD_SIZE-1 : `btb_idx_bitlen];
    assign EX_pc_btb_idx = EX_PC[`btb_idx_bitlen-1 : 0];
    assign EX_Branch = EX_inst_type[`I_Branch_idx];
    assign EX_Jump = (EX_inst_type[`R_JumpReg_idx] || EX_inst_type[`J_Jump_idx] || EX_inst_type[`Jump_Link_idx]) ? 1'b1 : 1'b0;

    always @(*) begin
        if(`BRANCH_PREDICTOR_ALWAYS_TAKEN) begin
            Taken = 1'b1;
        end else begin
            case(BHT[pc_btb_idx])
                `StrongTaken : Taken = 1'b1;
                `WeakTaken : Taken = 1'b1;
                `WeakNotTaken : Taken = 1'b0;
                `StrongNotTaken : Taken = 1'b0;
            endcase
        end
    end
    assign Prediction = Taken && Tag_Same;

    always @(*) begin
        if(`BRANCH_PREDICTOR_ENABLED) begin
            new_pc = (Prediction) ? BTB[pc_btb_idx] : pc_next_seq;
        end else begin
            new_pc = pc_next_seq;
        end
    end

    integer it;
    always @(posedge clk) begin
        if(reset_n == 1'b0) begin
            for(it = 0; it < `btb_size; it = it+1) begin
                BTB[it] = 0;
                TagTable[it] = {`btb_tag_bitlen{1'b1}};
                BHT[it] = `StrongTaken; // Initialize as strong taken
            end
        end else begin
            // if decoded instruction is jump or branch -> update BTB
            if(ID_Jump_Target != ID_PC_next_seq) begin
                BTB[ID_PC_btb_idx] = ID_Jump_Target;
                TagTable[ID_PC_btb_idx] = ID_PC_btb_tag;
            end

            // update BHT
            if(EX_Branch) begin
                if(`BRANCH_PREDICTOR_COUNTER) begin
                    if(Branch_Taken_Result == 1'b1) begin
                        if(BHT[EX_pc_btb_idx] != `StrongTaken) BHT[EX_pc_btb_idx] = BHT[EX_pc_btb_idx] + 1;
                    end else begin
                        if(BHT[EX_pc_btb_idx] != `StrongNotTaken) BHT[EX_pc_btb_idx] = BHT[EX_pc_btb_idx] - 1;
                    end
                end else if (`BRANCH_PREDICTOR_HYSTERSIS) begin
                    if(Branch_Taken_Result == 1'b1) begin
                        case(BHT[EX_pc_btb_idx])
                            `StrongTaken : BHT[EX_pc_btb_idx] = `StrongTaken;
                            `WeakTaken : BHT[EX_pc_btb_idx] = `StrongTaken;
                            `WeakNotTaken : BHT[EX_pc_btb_idx] = `StrongTaken;
                            `StrongNotTaken : BHT[EX_pc_btb_idx] = `WeakNotTaken;
                        endcase
                    end else begin
                        case(BHT[EX_pc_btb_idx])
                            `StrongTaken : BHT[EX_pc_btb_idx] = `WeakTaken;
                            `WeakTaken : BHT[EX_pc_btb_idx] = `StrongNotTaken;
                            `WeakNotTaken : BHT[EX_pc_btb_idx] = `StrongNotTaken;
                            `StrongNotTaken : BHT[EX_pc_btb_idx] = `StrongNotTaken;
                        endcase
                    end
                end
            end else if(EX_Jump) begin
                BHT[EX_pc_btb_idx] = `StrongTaken;
            end
        end
    end
endmodule