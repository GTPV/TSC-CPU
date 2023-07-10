# TSC CPU
Simple TSC-ISA cpu implementation with Verilog HDL.

---
## 1. Single Cycle CPU
Single Cycle CPU implementation with Verilog HDL.

* Only 5 instructions from TSC-ISA is implemented : **`ADD`, `ADI`, `LHI`, `JMP`, `WWD`**

---
## 2. Multi Cycle CPU
Multi Cycle CPU implementation with Verilog HDL.

* All instructions from TSC-ISA except **`RWD`** is implemented.
* Maximum 5 cycle for each instruction.

    | Number of cycle | Stage detail            | Instruction                                           |
    |-----------------|-------------------------|-------------------------------------------------------|
    | 2 cycles | IF-ID | **`WWD`**, **`JMP`**, **`JAL`**, **`JPR`**, **`JRL`**, **`HLT`** |
    | 3 cycles | IF-ID-EX | **`BNE`**, **`BEQ`**, **`BGZ`**, **`BLZ`** |
    | 4 cycles - A | IF-ID-EX-WB | **`ADD`**, **`SUB`**, **`AND`**, **`ORR`**, **`NOT`**, **`TCP`**, **`SHL`**, **`SHP`**, **`ADI`**, **`ORI`**, **`LHI`** |
    | 4 cycles - B | IF-ID-EX-MEM | **`SWD`** |
    | 5 cycles | IF-ID-EX-MEM-WB | **`LWD`** |

---
## 3. Pipelined CPU
MIPS 5 stage pipelined cpu.(**`IF-ID-EX-MEM-WB`**)

* All instructions from TSC-ISA except **`RWD`** is implemented.

* Same testbench as Multi Cycle CPU is used.

* Branch prediction can be enabled/disabled by setting `BRANCH_PREDICTION_ENABLED` in `opcodes.v`.
    * Always-taken(Default)
    * 2-bit Hystersis
    * 2-bit Saturation Counter

* Data hazards handled by pipeline stall.
    * Data forwarding can be enabled/disabled by setting `DATA_FORWARDING_ENABLED` in `opcodes.v`.
    * Self-forwarding of RegisterFile by using `negedge clk` can be enabled/disabled by setting `RF_SELF_FORWARDING` in `opcodes.v` (cannot be disabled when data forwarding is enabled).

* `num_inst` is incremented when valid instruction commits(retires)

* Assuming ideal memory : 1-cycle read, 1-cycle write.

---
## 4. Cached CPU
Cache added on **[3. Pipelined CPU]** above. Baseline(only memory access delay applied) design also included.

* Baseline
    * Memory delay applied to **[3. Pipelined CPU]** : 2-cycle read, 2-cycle write.

    * Single-level, direct-mapped cache.
        * Block size : 4 words
        * Height : 4 lines
    * Instruction and data cache separated(**`i-cache`**, **`d-cache`**).
    * Hit policy : write-through.
    * Miss policy : write-no-allocate.

---
## 5. CPU with DMA
DMA added on **[4. Cached CPU]** above.

* Cache design modified for simpler DMA control.
    * Hit policy : write-back.
    * Miss policy : write-allocate.
* Cycle stealing of DMA module can be enabled/disabled by setting `CYCLE_STEAL_ENABLED` in `opcodes.v`.

---