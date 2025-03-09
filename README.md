# PHASE 1
# RISC-V 32-bit Assembler

## Overview
This project is a lightweight assembler designed for a subset of the RISC-V 32-bit Instruction Set Architecture. It reads an assembly source file (`input.asm`) and generates a corresponding machine code file (`output.mc`). The output file is divided into two segments:
- **Code Segment:** Lists each instruction’s address, its machine code in hexadecimal format, and a detailed binary field breakdown.
- **Data Segment:** Contains the initial memory contents as specified by data directives (e.g., `.byte`, `.word`, etc.).

This tool is excellent for educational purposes and small-scale experiments with RISC-V.

## Features
- **Instruction Support:**  
  - **R-type:** e.g., `add x1, x2, x3`, `sub x4, x5, x6`  
  - **I-type:** e.g., `addi x1, x0, 10`, `lb x2, 0(x3)`, `jalr x1, 0(x5)`  
  - **S-type:** e.g., `sb x6, 4(x7)`  
  - **SB-type (Branch):** e.g., `beq x1, x2, label`, `bne x3, x4, loop`  
  - **U-type:** e.g., `lui x5, 0x12345`, `auipc x6, 0xABCD`  
  - **UJ-type:** e.g., `jal x1, label`
- **Data Directives:** Supports `.byte`, `.half`, `.word`, `.dword`, and `.asciz` along with section directives (`.text` and `.data`).
- **Detailed Output:** The machine code output includes each instruction’s address, hexadecimal machine code, and a binary field breakdown comment.

## Project Structure
The repository includes the following key files:
- `input.asm` – The assembly language file to be processed.
- `output.mc` – The resulting machine code file.
- `main.cpp`  – The complete source code for the assembler.

## Compilation & Execution
To build and run the assembler, use these steps:
1. **Open a Terminal** in the project’s main directory.
2. **Compile** the code with:
    ```bash
    g++ -std=c++11 main.cpp -o assembler
    ```
3. **Run** the assembler:
    ```bash
    ./assembler
    ```
The assembler reads `input.asm` and outputs `output.mc`, including both the code and data segments.

## Usage
Create or edit your assembly file (`input.asm`) using standard RISC-V syntax. For example:
```asm
.data
arrayA: .byte 10, 20, 30, 40
arrayB: .word 3735928559

.text
main:
    addi x1, x0, 5
    beq  x1, x0, end
    add  x0, x0, x0
end:
    jal  x0, end
