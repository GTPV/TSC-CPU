`define WORD_SIZE 16
`define MEM_DELAY 2'b10

///////////////////////////////////////////////////////////////////////////
// Cache status
`define CACHE_IDLE 2'b00
`define CACHE_READING 2'b01
`define CACHE_WRITING 2'b10
`define CACHE_READY 2'b11

///////////////////////////////////////////////////////////////////////////
// Cache enable/disable
`define CACHE_ENABLED 1'b0

///////////////////////////////////////////////////////////////////////////
// Values for enable/disable dataforwarding
`define DATA_FORWARDING_ENABLED 1'b1
`define RF_SELF_FORWARDING (1'b1 || `DATA_FORWARDING_ENABLED)
`define BRANCH_PREDICTOR_ENABLED 1'b1
`define BRANCH_PREDICTOR_ALWAYS_TAKEN 1'b1
`define BRANCH_PREDICTOR_HYSTERSIS (1'b0 && `BRANCH_PREDICTOR_ENABLED)
`define BRANCH_PREDICTOR_COUNTER (1'b1 && `BRANCH_PREDICTOR_ENABLED)

///////////////////////////////////////////////////////////////////////////
// TSC ISA

// TSC ISA Instruction Function code
`define FUNC_ADD 6'd0
`define FUNC_SUB 6'd1
`define FUNC_AND 6'd2
`define FUNC_ORR 6'd3
`define FUNC_NOT 6'd4
`define FUNC_TCP 6'd5
`define FUNC_SHL 6'd6
`define FUNC_SHR 6'd7
`define FUNC_WWD 6'd28
`define FUNC_JPR 6'd25
`define FUNC_JRL 6'd26
`define FUNC_HLT 6'd29

// TSC ISA Instruction Opcodes
`define OPCODE_ADI 4'd4
`define OPCODE_ORI 4'd5
`define OPCODE_LHI 4'd6
`define OPCODE_LWD 4'd7
`define OPCODE_SWD 4'd8
`define OPCODE_BNE 4'd0
`define OPCODE_BEQ 4'd1
`define OPCODE_BGZ 4'd2
`define OPCODE_BLZ 4'd3
`define OPCODE_JMP 4'd9
`define OPCODE_JAL 4'd10
`define OPCODE_Rtype 4'd15
`define OPCODE_NOP 4'd11

// Flushed instruction
`define INST_FLUSHED ({`OPCODE_NOP, `J_target_addr_bitlen'b0})

///////////////////////////////////////////////////////////////////////////
// PC Parsing

`define btb_idx_bitlen 6

///////////////////////////////////////////////////////////////////////////
// Instruction Parsing

// bit-length of each instruction format type
`define opcode_bitlen 4
`define reg_addr_bitlen 2
`define R_func_bitlen 6
`define I_imm_off_bitlen 8
`define J_target_addr_bitlen 12
`define inst_type_bitlen 9

// idx for parsing instruction
`define opcode_idx_left (`WORD_SIZE - 1)
`define opcode_idx_right (`WORD_SIZE - `opcode_bitlen)
`define rs_idx_left (`opcode_idx_right - 1)
`define rs_idx_right (`opcode_idx_right - `reg_addr_bitlen)
`define rt_idx_left (`rs_idx_right - 1)
`define rt_idx_right (`rs_idx_right - `reg_addr_bitlen)
`define rd_idx_left (`rt_idx_right - 1)
`define rd_idx_right (`rt_idx_right - `reg_addr_bitlen)
`define R_func_idx_left (`R_func_bitlen - 1)
`define R_func_idx_right (0)
`define imm_off_idx_left (`I_imm_off_bitlen - 1)
`define imm_off_idx_right (0)
`define target_addr_idx_left (`J_target_addr_bitlen - 1)
`define target_addr_idx_right (0)

///////////////////////////////////////////////////////////////////////////
// ALU OPCODES

`define	OP_ADD	4'b0000
`define	OP_SUB	4'b0001
`define	OP_AND	4'b0010
`define	OP_ORR	4'b0011
`define	OP_NOT	4'b0100
`define	OP_TCP	4'b0101
`define	OP_SHL	4'b0110
`define	OP_SHR	4'b0111
`define	OP_LHI	4'b1000

///////////////////////////////////////////////////////////////////////////
// Instruction type indexes

`define R_Arithmetic_idx 8
`define R_Halt_idx 7
`define R_Output_idx 6
`define R_JumpReg_idx 5
`define I_Arithmetic_idx 4
`define I_memory_idx 3
`define I_Branch_idx 2
`define J_Jump_idx 1
`define Jump_Link_idx 0

///////////////////////////////////////////////////////////////////////////
// Stages

`define STAGE_IF 3'b000
`define STAGE_ID 3'b001
`define STAGE_EX 3'b010
`define STAGE_MEM 3'b011
`define STAGE_WB 3'b100

///////////////////////////////////////////////////////////////////////////
// Control Signals

/* // ALUSrcA
`define ALUSrcA_RF_rs 1'b0
`define ALUSrcA_Seq_PC 1'b1 */

// ALUSrcB
`define ALUSrcB_RF_rt 2'b00
`define ALUSrcB_Seq 2'b01
`define ALUSrcB_I_OFFSET 2'b10
`define ALUSrcB_ZERO 2'b11

// RegWriteSrc
`define RegWriteSrc_ALU 2'b00
`define RegWriteSrc_MEM 2'b01
`define RegWriteSrc_PC 2'b10
/* 
// RegDst
`define RegDst_rd 2'b00
`define RegDst_rt 2'b01
`define RegDst_2 2'b10
 */