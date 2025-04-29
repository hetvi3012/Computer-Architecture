//2023CSB1123  Hetvi Bagdai
//2023CSB1174  Yashasvi Chaudhary
//2023CSB1170  Tanisha Gupta
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <cstdint>
#include <iomanip>
#include <algorithm>  // for std::sort
#include <set>

// =====================================================================
// Add ALU operation types
// =====================================================================
enum ALUOpType {
    ALU_ADD,
    ALU_SUB,
    ALU_MUL,
    ALU_DIV,
    ALU_REM,
    ALU_AND,
    ALU_OR,
    ALU_XOR,
    ALU_SLL,
    ALU_SRL,
    ALU_SRA,
    ALU_SLT,
    ALU_PASS, // Pass-through for LUI/AUIPC
    ALU_EQ,   // Equality comparison (RA == RB)
    ALU_GE    // Greater-than-or-equal comparison (RA >= RB)
};

// =====================================================================
// Global CPU State
// =====================================================================
static const int NUM_REGS = 32;
int32_t R[NUM_REGS];       // Register file
uint32_t PC = 0;           // Program Counter
uint32_t IR = 0;           // Instruction Register
int32_t  RA = 0;           // Operand A
int32_t  RB = 0;           // Operand B
int32_t  RM = 0;           // Used for store data
int32_t  RZ = 0;           // ALU output
int32_t  RY = 0;           // Write-back data
int32_t  MDR= 0;           // Memory data register
uint32_t MAR = 0;          // Memory Address Register

uint64_t clockCycle = 0;   // Cycle counter

// Global control signals
bool regWrite = false;
bool memRead = false;
bool memWrite = false;
bool branch = false;
bool jump = false;
uint8_t memToReg = 0; // 0: ALU result, 1: Memory, 2: PC+4
uint8_t memSize = 2;  // 0: Byte, 1: Halfword, 2: Word
bool memSignExtend = false;
ALUOpType aluOp = ALU_PASS;

// =====================================================================
// Instruction Memory (< 0x10000000)
// =====================================================================
std::map<uint32_t, uint32_t> instrMemory;

// =====================================================================
// DataSegment Class (handles a map of address -> byte)
// We'll use it for both data memory and stack memory
// =====================================================================
class MemSegment {
public:
    // Each address in "memory" is one byte
    std::map<uint32_t, uint8_t> memory;

    void writeByte(uint32_t address, uint8_t value) {
        memory[address] = value;
    }

    void writeWord(uint32_t address, int32_t value) {
        for (int i = 0; i < 4; i++) {
            memory[address + i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
        }
    }

    int8_t readByte(uint32_t address) const {
        auto it = memory.find(address);
        if (it != memory.end()) {
            return static_cast<int8_t>(it->second);
        }
        return 0;
    }

    int32_t readWord(uint32_t address) const {
        int32_t result = 0;
        for (int i = 0; i < 4; i++) {
            auto it = memory.find(address + i);
            uint8_t b = (it != memory.end()) ? it->second : 0;
            result |= (b << (8 * i));
        }
        return result;
    }
};

// =====================================================================
// We will have two separate MemSegments for data and stack
// =====================================================================
MemSegment dataSegment;   // for addresses in [0x10000000, 0x7FFFFFFF)
MemSegment stackSegment;  // for addresses >= 0x7FFFFFFF

// =====================================================================
// Dumping memory to an .mc file
//   - Writes each 4-byte aligned address in ascending order
//   - Only writes addresses actually stored in the memory map
//   - Skips addresses outside the intended segment's range
// =====================================================================
void dumpSegmentToFile(const std::string &filename, 
                       const MemSegment &seg,
                       uint32_t startAddr, 
                       uint32_t endAddr /* inclusive or exclusive? */)
{
    // Open file for overwrite
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "ERROR: Could not open/create " << filename << "\n";
        return;
    }

    // Gather addresses from seg.memory
    std::vector<uint32_t> addresses;
    addresses.reserve(seg.memory.size());
    for (const auto &kv : seg.memory) {
        addresses.push_back(kv.first);
    }
    std::sort(addresses.begin(), addresses.end());

    // We'll write lines only for addresses in [startAddr, endAddr]
    // For the stack region, if endAddr < startAddr, we'll interpret that accordingly.
    // But here, let's just check the range as you prefer:
    std::set<uint32_t> visited;

    for (uint32_t addr : addresses) {
        if (addr < startAddr) {
            continue;
        }
        if (endAddr >= startAddr) {
            // "normal" range check
            if (addr >= endAddr) {
                continue;
            }
        } else {
            // If endAddr < startAddr, you might interpret it differently, but
            // for simplicity, let's just not do anything special.
        }

        // Only write 4-byte-aligned addresses
        if ((addr % 4) != 0) {
            continue;
        }
        // skip if we've visited it
        if (visited.count(addr) != 0) {
            continue;
        }

        // read the 32-bit word
        int32_t wordVal = seg.readWord(addr);

        fout << std::hex << "0x" 
             << std::setw(8) << std::setfill('0') << addr << "  0x"
             << std::setw(8) << std::setfill('0') << static_cast<uint32_t>(wordVal)
             << std::dec << "\n";

        // Mark 4 bytes visited
        visited.insert(addr + 0);
        visited.insert(addr + 1);
        visited.insert(addr + 2);
        visited.insert(addr + 3);
    }

    fout.close();
}

// =====================================================================
// Dumping the instruction memory (which is map<uint32_t, uint32_t>)
// to instruction.mc
// =====================================================================
void dumpInstructionMemoryToFile(const std::string &filename) {
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "ERROR: Could not open/create " << filename << "\n";
        return;
    }

    // gather addresses
    std::vector<uint32_t> addresses;
    addresses.reserve(instrMemory.size());
    for (auto &kv : instrMemory) {
        addresses.push_back(kv.first);
    }
    std::sort(addresses.begin(), addresses.end());

    // write
    for (auto addr : addresses) {
        // each entry is already a full 32-bit instruction
        uint32_t word = instrMemory[addr];
        fout << std::hex 
             << "0x" << std::setw(8) << std::setfill('0') << addr 
             << "  0x" << std::setw(8) << std::setfill('0') << word
             << std::dec << "\n";
    }
    fout.close();
}

// =====================================================================
// Helper: signExtend, getBits
// =====================================================================
static inline uint32_t getBits(uint32_t val, int hi, int lo) {
    uint32_t mask = (1u << (hi - lo + 1)) - 1;
    return (val >> lo) & mask;
}

static inline int32_t signExtend(uint32_t value, int bitCount) {
    int shift = 32 - bitCount;
    return (int32_t)((int32_t)(value << shift) >> shift);
}

// =====================================================================
// DecodedInstr
// =====================================================================
struct DecodedInstr {
    uint32_t opcode;
    uint32_t rd;
    uint32_t rs1;
    uint32_t rs2;
    uint32_t funct3;
    uint32_t funct7;
    int32_t  imm;

    // Control signals
    bool regWrite;      // Enable register write
    bool memRead;       // Enable memory read
    bool memWrite;      // Enable memory write
    bool branch;        // Enable branch
    bool jump;          // Enable jump
    ALUOpType aluOp;    // ALU operation type
    uint8_t memToReg;   // Select memory or ALU result for write-back
    uint8_t memSize;    // Memory access size: 0=byte, 1=halfword, 2=word
    bool memSignExtend; // Sign-extend memory read data
    bool zero;          // ALU zero signal (result is 0)
};

DecodedInstr d; // Make d a global variable to persist across states

// =====================================================================
// decode
// =====================================================================
DecodedInstr decode(uint32_t instr) {
    DecodedInstr d{};
    d.opcode = getBits(instr, 6, 0);
    d.rd     = getBits(instr, 11, 7);
    d.funct3 = getBits(instr, 14, 12);
    d.rs1    = getBits(instr, 19, 15);
    d.rs2    = getBits(instr, 24, 20);
    d.funct7 = getBits(instr, 31, 25);

    // Default control signals
    d.regWrite = false;
    d.memRead = false;
    d.memWrite = false;
    d.branch = false;
    d.jump = false;
    d.aluOp = ALU_PASS;
    d.memToReg = 0;
    d.memSize = 2; // Default to word
    d.memSignExtend = false;
    d.zero = false;

    // Decode immediate
    switch(d.opcode) {
        case 0x13: // I-type ALU
        case 0x03: // I-type LOAD
        case 0x67: { // I-type JALR
            uint32_t imm12 = getBits(instr, 31, 20);
            d.imm = signExtend(imm12, 12);
        } break;
        case 0x23: { // S-type
            uint32_t immHigh = getBits(instr, 31, 25);
            uint32_t immLow  = getBits(instr, 11, 7);
            uint32_t imm12 = (immHigh << 5) | immLow;
            d.imm = signExtend(imm12, 12);
        } break;
        case 0x63: { // SB-type
            uint32_t immBit12    = getBits(instr, 31, 31);
            uint32_t immBit11    = getBits(instr, 7, 7);
            uint32_t immBits10_5 = getBits(instr, 30, 25);
            uint32_t immBits4_1  = getBits(instr, 11, 8);
            uint32_t immAll = (immBit12 << 12) | (immBit11 << 11) 
                            | (immBits10_5 << 5) | (immBits4_1 << 1);
            d.imm = signExtend(immAll, 13);
        } break;
        case 0x37: // LUI
        case 0x17: { // AUIPC
            uint32_t imm20 = getBits(instr, 31, 12);
            d.imm = (int32_t)(imm20 << 12);
        } break;
        case 0x6F: { // UJ-type (JAL)
            uint32_t immBit20    = getBits(instr, 31, 31);
            uint32_t immBits19_12= getBits(instr, 19, 12);
            uint32_t immBit11    = getBits(instr, 20, 20);
            uint32_t immBits10_1 = getBits(instr, 30, 21);
            uint32_t immAll = (immBit20 << 20) | (immBits19_12 << 12)
                            | (immBit11 << 11) | (immBits10_1 << 1);
            d.imm = signExtend(immAll, 21);
        } break;
        default:
            d.imm = 0;
            break;
    }

    return d;
}

// =====================================================================
// isTerminationInstr
// =====================================================================
bool isTerminationInstr(uint32_t instr) {
    return (instr == 0x00000000);
}

// =====================================================================
// parseInputMC: read addresses from input.mc and distribute them
//   - <0x10000000 => instrMemory
//   - [0x10000000, 0x7FFFFFFF) => dataSegment
//   - >=0x7FFFFFFF => stackSegment
// =====================================================================
bool parseInputMC(const std::string &filename) {
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        std::cerr << "ERROR: Could not open " << filename << "\n";
        return false;
    }

    std::string line;
    while (std::getline(fin, line)) {
        // remove comments
        size_t cpos = line.find('#');
        if (cpos != std::string::npos) {
            line = line.substr(0, cpos);
        }
        if (line.empty()) {
            continue;
        }

        std::stringstream ss(line);
        std::string addrStr, dataStr;
        ss >> addrStr >> dataStr;
        if(dataStr[0] == '<' || dataStr[0] == 't')continue;
        if (addrStr.empty() || dataStr.empty()) {
            continue;
        }

        // remove trailing comma
        size_t commaPos = dataStr.find(',');
        if (commaPos != std::string::npos) {
            dataStr = dataStr.substr(0, commaPos);
        }

        try {
            uint32_t address = std::stoul(addrStr, nullptr, 16);
            uint32_t word    = std::stoul(dataStr, nullptr, 16);

            if (address < 0x10000000) {
                // instructions
                instrMemory[address] = word;
            }
            else if (address < 0x7FFFFFFF) {
                // data
                dataSegment.writeWord(address, static_cast<int32_t>(word));
            }
            else {
                // stack
                // (we'll treat address >= 0x7FFFFFFF as stack region)
                stackSegment.writeWord(address, static_cast<int32_t>(word));
            }
        }
        catch (...) {
            std::cerr << "Parsing error on line: " << line << "\n";
            continue;
        }
    }
    fin.close();
    return true;
}

// =====================================================================
// printRegisters
// =====================================================================
void printRegisters() {
    std::cout << "Register File:\n";
    for (int i = 0; i < NUM_REGS; i++) {
        std::cout << "R[" << std::setw(2) << i << "]=" << R[i] << "   ";
        if ((i+1)%4 == 0) std::cout << "\n";
    }
    std::cout << "-------------------------------------\n";
    std::cout << "PC = 0x" << std::hex << PC 
              << "  IR = 0x" << IR << std::dec << "\n";
    std::cout << "RA=" << RA << "  RB=" << RB << "  RM=" << RM << "\n";
    std::cout << "RZ=" << RZ << "  RY=" << RY << "  MDR=" << MDR << "\n";
    std::cout << "===========================================\n";
}

// =====================================================================
// getMemSegmentForAddress:
//   Helper to figure out which segment an address belongs to.
//   We will read/write from the correct segment in LOAD/STORE ops.
// =====================================================================
MemSegment* getMemSegmentForAddress(uint32_t addr) {
    if (addr < 0x10000000) {
        // For simplicity, let's assume we do NOT allow loads/stores to instruction memory
        // But if you wanted self-modifying code, you'd handle it. 
        // We'll just return nullptr here to indicate invalid or unexpected.
        return nullptr;
    }
    else if (addr < 0x7FFFFFFF) {
        return &dataSegment;
    }
    else {
        return &stackSegment;
    }
}

// =====================================================================
// Instruction Address Generator (IAG)
// =====================================================================
class IAG {
public:
    uint32_t PCtemp; // Temporary storage for PC + 4

    // Update PC based on signals
    void updatePC(bool jump, bool branch, bool zero, int32_t offset, int32_t RZ) {
        PCtemp = PC + 4; // Always store PC + 4 in PCtemp
        if (jump) {
            if (branch) {
                std::cout << "current PC: " << std::hex << PC << std::dec << "\n";
                std::cout << "branch offset: " << std::hex << offset << std::dec << "\n";
                PC += offset; // For JAL, PC = PC + offset
            } else {
                PC = RZ & ~1U; // For JALR, PC = RZ (aligned to even address)
            }
        } else if (branch && zero) {
            PC += offset; // For branch, PC = PC + offset
        } else {
            PC = PCtemp; // Default to PC + 4
        }
    }
};

// Global IAG instance
IAG iag;

// =====================================================================
// Control circuitry function
// =====================================================================
void controlCircuitry(uint32_t opcode, uint32_t funct3, uint32_t funct7) {
    // Reset all control signals
    regWrite = false;
    memRead = false;
    memWrite = false;
    branch = false;
    jump = false;
    memToReg = 0;
    memSize = 2; // Default to word
    memSignExtend = false;
    aluOp = ALU_PASS;

    // Update control signals based on opcode, funct3, and funct7
    switch (opcode) {
        case 0x33: // R-type
            regWrite = true;
            switch (funct3) {
                case 0x0:
                    aluOp = (funct7 == 0x20) ? ALU_SUB : 
                            (funct7 == 0x01) ? ALU_MUL : ALU_ADD;
                    break;
                case 0x4:
                    aluOp = (funct7 == 0x01) ? ALU_DIV : ALU_XOR;
                    break;
                case 0x6:
                    aluOp = (funct7 == 0x01) ? ALU_REM : ALU_OR;
                    break;
                case 0x7:
                    aluOp = ALU_AND;
                    break;
                case 0x1:
                    aluOp = ALU_SLL;
                    break;
                case 0x2:
                    aluOp = ALU_SLT;
                    break;
                case 0x5:
                    aluOp = (funct7 == 0x20) ? ALU_SRA : ALU_SRL;
                    break;
                default:
                    break;
            }
            break;
        case 0x13: // I-type ALU
            regWrite = true;
            switch (funct3) {
                case 0x0:
                    aluOp = ALU_ADD; // ADDI
                    break;
                case 0x7:
                    aluOp = ALU_AND; // ANDI
                    break;
                case 0x6:
                    aluOp = ALU_OR; // ORI
                    break;
                case 0x4:
                    aluOp = ALU_XOR; // XORI
                    break;
                case 0x2:
                    aluOp = ALU_SLT; // SLTI
                    break;
                case 0x1:
                    aluOp = ALU_SLL; // SLLI
                    break;
                case 0x5:
                    aluOp = ((funct7 & 0x20) == 0x20) ? ALU_SRA : ALU_SRL; // SRLI/SRAI
                    break;
                default:
                    break;
            }
            break;
        case 0x03: // LOAD
            regWrite = true;
            memRead = true;
            aluOp = ALU_ADD; // Address calculation
            memToReg = 1; // Write-back from memory
            switch (funct3) {
                case 0x0: memSize = 0; memSignExtend = true; break; // LB
                case 0x1: memSize = 1; memSignExtend = true; break; // LH
                case 0x2: memSize = 2; memSignExtend = false; break; // LW
                case 0x4: memSize = 0; memSignExtend = false; break; // LBU
                case 0x5: memSize = 1; memSignExtend = false; break; // LHU
                default: break;
            }
            break;
        case 0x23: // STORE
            memWrite = true;
            aluOp = ALU_ADD; // Address calculation
            switch (funct3) {
                case 0x0: memSize = 0; break; // SB
                case 0x1: memSize = 1; break; // SH
                case 0x2: memSize = 2; break; // SW
                default: break;
            }
            break;
        case 0x63: // BRANCH
            branch = true;
            switch (funct3) {
                case 0x0: aluOp = ALU_SUB; break; // BEQ: RA - RB == 0
                case 0x1: aluOp = ALU_EQ; break;  // BNE: RA == RB
                case 0x4: aluOp = ALU_GE; break;  // BLT: RA >= RB
                case 0x5: aluOp = ALU_SLT; break; // BGE: RA < RB
                default: break;
            }
            break;
        case 0x6F: // JAL
            regWrite = true;
            jump = true;
            branch = true; // No branch for JAL
            aluOp = ALU_PASS; // No ALU operation needed for JAL
            memToReg = 2; // Write-back PC+4 (PCtemp from IAG)
            break;
        case 0x67: // JALR
            regWrite = true;
            jump = true;
            aluOp = ALU_ADD; // Calculate new PC using ALU
            memToReg = 2; // Write-back PC+4
            break;
        case 0x37: // LUI
            regWrite = true;
            aluOp = ALU_PASS; // Pass-through immediate
            break;
        case 0x17: // AUIPC
            regWrite = true;
            aluOp = ALU_ADD; // Add upper immediate to PC
            break;
        default:
            break;
    }
}

// =====================================================================
// Memory Processor Interface
// =====================================================================
void memoryProcessorInterface(bool memRead, bool memWrite, uint8_t memSize) {
    MemSegment* seg = getMemSegmentForAddress(MAR);
    if (!seg) return; // Invalid memory segment

    if (memRead) {
        // Perform memory read based on size
        switch (memSize) {
            case 0: // Byte
                MDR = memSignExtend ? static_cast<int8_t>(seg->readByte(MAR))
                                    : static_cast<uint8_t>(seg->readByte(MAR));
                break;
            case 1: // Halfword
                MDR = memSignExtend ? static_cast<int16_t>(seg->readByte(MAR) | (seg->readByte(MAR + 1) << 8))
                                    : static_cast<uint16_t>(seg->readByte(MAR) | (seg->readByte(MAR + 1) << 8));
                break;
            case 2: // Word
                MDR = seg->readWord(MAR);
                break;
            default:
                break;
        }
    }

    if (memWrite) {
        // Perform memory write based on size
        switch (memSize) {
            case 0: // Byte
                seg->writeByte(MAR, RM & 0xFF);
                break;
            case 1: // Halfword
                seg->writeByte(MAR, RM & 0xFF);
                seg->writeByte(MAR + 1, (RM >> 8) & 0xFF);
                break;
            case 2: // Word
                seg->writeWord(MAR, RM);
                break;
            default:
                break;
        }
    }
}

// =====================================================================
// Define states for the multi-cycle implementation
// =====================================================================
enum State {
    FETCH,
    DECODE,
    EXECUTE,
    MEMORY_ACCESS,
    WRITE_BACK,
    HALT
};

// Initialize the state machine
State currentState = FETCH;

// =====================================================================
// main
// =====================================================================
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.mc>\n";
        return 1;
    }

    std::string inputFile = argv[1];
    if (!parseInputMC(inputFile)) {
        return 1;
    }

    // Initialize registers and memory
    for (int i = 0; i < NUM_REGS; i++) {
        R[i] = 0;
    }
    R[2] = 0x7FFFFFFC; // stack pointer
    PC = 0;
    clockCycle = 0;

    // Dump initial contents to files
    dumpInstructionMemoryToFile("instruction.mc");
    dumpSegmentToFile("data.mc", dataSegment, 0x10000000, 0x7FFFFFFF);
    dumpSegmentToFile("stack.mc", stackSegment, 0x7FFFFFFF, 0xFFFFFFFF);

    // Print initial register state
    std::cout << "Initial state (before cycle 0):\n";
    printRegisters();

    // Prompt user for control
    char userInput;
    std::cout << "Enter N for next, R for remainder, E to exit: ";
    std::cin >> userInput;
    if (userInput == 'E' || userInput == 'e') {
        std::cout << "Exiting at user request.\n";
        return 0;
    }
    bool runAllRemaining = (userInput == 'R' || userInput == 'r');

    std::cout << "Starting simulation...\n";

    while (currentState != HALT) {
        std::cout << "Clock Cycle: " << clockCycle << "\n";

        switch (currentState) {
            case FETCH: {
                std::cout << "[Fetch] Current PC: 0x" << std::hex << PC << std::dec << "\n";
                auto it = instrMemory.find(PC);
                if (it == instrMemory.end()) {
                    std::cout << "[Fetch] No instruction at PC=0x" 
                              << std::hex << PC << ". Exiting.\n";
                    currentState = HALT;
                    break;
                }
                IR = it->second;
                std::cout << "[Fetch] PC=0x" << std::hex << PC 
                          << " IR=0x" << IR << std::dec << "\n";

                if (isTerminationInstr(IR)) {
                    std::cout << "[Fetch] Encountered 0x00000000 => stop.\n";
                    currentState = HALT;
                } else {
                    // PC += 4; // Increment PC by 4 unless explicitly modified
                    currentState = DECODE;
                }
            } break;

            case DECODE: {
                std::cout << "[Decode] Current PC: 0x" << std::hex << PC << std::dec << "\n";
                if (IR == 0) {
                    std::cout << "[Decode] Nothing to perform.\n";
                } else {
                    d = decode(IR);
                    std::cout << "[Decode] opcode=0x" << std::hex << d.opcode
                              << " rd=" << d.rd << " rs1=" << d.rs1 
                              << " rs2=" << d.rs2 
                              << " funct3=0x" << d.funct3
                              << " funct7=0x" << d.funct7 
                              << " imm=" << std::dec << d.imm << "\n";

                    controlCircuitry(d.opcode, d.funct3, d.funct7);

                    RA = R[d.rs1];
                    // Corrected logic for RB: Use immediate for I-type instructions, otherwise use rs2
                    RB = (d.opcode == 0x13 || d.opcode == 0x03 || d.opcode == 0x67 || d.opcode == 0x23) ? d.imm : R[d.rs2];
                    RM = R[d.rs2];
                    std::cout << "[Decode] RA=" << RA << " RB=" << RB << " RM=" << RM << "\n";
                }
                currentState = EXECUTE;
            } break;

            case EXECUTE: {
                std::cout << "[Execute] Current PC: 0x" << std::hex << PC << std::dec << "\n";
                if (aluOp == ALU_PASS && !branch && !jump) {
                    std::cout << "[Execute] Nothing to perform.\n";
                } else {
                    switch (aluOp) {
                        case ALU_ADD:
                            RZ = RA + RB;
                            std::cout << "[Execute] ALU_ADD: " << RA << " + " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_SUB:
                            RZ = RA - RB;
                            std::cout << "[Execute] ALU_SUB: " << RA << " - " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_MUL:
                            RZ = RA * RB;
                            std::cout << "[Execute] ALU_MUL: " << RA << " * " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_DIV:
                            RZ = (RB != 0) ? RA / RB : 0;
                            std::cout << "[Execute] ALU_DIV: " << RA << " / " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_REM:
                            RZ = (RB != 0) ? RA % RB : 0;
                            std::cout << "[Execute] ALU_REM: " << RA << " % " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_AND:
                            RZ = RA & RB;
                            std::cout << "[Execute] ALU_AND: " << RA << " & " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_OR:
                            RZ = RA | RB;
                            std::cout << "[Execute] ALU_OR: " << RA << " | " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_XOR:
                            RZ = RA ^ RB;
                            std::cout << "[Execute] ALU_XOR: " << RA << " ^ " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_SLL:
                            RZ = RA << (RB & 0x1F);
                            std::cout << "[Execute] ALU_SLL: " << RA << " << " << (RB & 0x1F) << " = " << RZ << "\n";
                            break;
                        case ALU_SRL:
                            RZ = static_cast<int32_t>(static_cast<uint32_t>(RA) >> (RB & 0x1F));
                            std::cout << "[Execute] ALU_SRL: " << RA << " >> " << (RB & 0x1F) << " = " << RZ << "\n";
                            break;
                        case ALU_SRA:
                            RZ = RA >> (RB & 0x1F);
                            std::cout << "[Execute] ALU_SRA: " << RA << " >> " << (RB & 0x1F) << " (arithmetic) = " << RZ << "\n";
                            break;
                        case ALU_SLT:
                            RZ = (RA < RB) ? 1 : 0;
                            std::cout << "[Execute] ALU_SLT: " << RA << " < " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_EQ:
                            RZ = (RA == RB) ? 1 : 0;
                            std::cout << "[Execute] ALU_EQ: " << RA << " == " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_GE:
                            RZ = (RA >= RB) ? 1 : 0;
                            std::cout << "[Execute] ALU_GE: " << RA << " >= " << RB << " = " << RZ << "\n";
                            break;
                        case ALU_PASS:
                            RZ = RB;
                            std::cout << "[Execute] ALU_PASS: Passing " << RB << " as RZ = " << RZ << "\n";
                            break;
                        default:
                            std::cout << "[Execute] Unknown ALU operation.\n";
                            break;
                    }

                    d.zero = (RZ == 0);
                    if (branch || jump) {
                        std::cout << "[Execute] Branch or jump detected. Delaying PC update to WRITE_BACK stage.\n";
                    }
                    MAR = RZ;
                    std::cout << "[Execute] RZ=" << RZ << "\n";
                }
                currentState = MEMORY_ACCESS;
            } break;

            case MEMORY_ACCESS: {
                std::cout << "[Memory Access] Current PC: 0x" << std::hex << PC << std::dec << "\n";
                if (!memRead && !memWrite) {
                    std::cout << "[Memory Access] Nothing to perform.\n";
                } else {
                    std::cout << "[Memory Access] Accessing memory for load/store operations.\n";
                    memoryProcessorInterface(memRead, memWrite, memSize);
                    std::cout << "[Memory Access] MAR=" << MAR << " MDR=" << MDR << "\n";
                }
                currentState = WRITE_BACK;
            } break;

            case WRITE_BACK: {
                std::cout << "[Write Back] Current PC: 0x" << std::hex << PC << std::dec << "\n";
                iag.updatePC(jump, branch, d.zero, d.imm, RZ);
                if (!regWrite) {
                    std::cout << "[Write Back] Nothing to perform.\n";
                } else {
                    std::cout << "[Write Back] Writing results back to the register file.\n";
                    if (memToReg == 1) {
                        RY = MDR;
                    } else if (memToReg == 2) {
                        RY = iag.PCtemp;
                    } else {
                        RY = RZ;
                    }
                    R[d.rd] = RY;
                    std::cout << "[Write Back] RY=" << RY << "\n";
                }
                R[0] = 0; // x0 always 0

                printRegisters();

                // Update memory dumps
                dumpInstructionMemoryToFile("instruction.mc");
                dumpSegmentToFile("data.mc", dataSegment, 0x10000000, 0x7FFFFFFF);
                dumpSegmentToFile("stack.mc", stackSegment, 0x7FFFFFFF, 0xFFFFFFFF);

                currentState = FETCH;
            } break;

            case HALT:
                break;
        }

        clockCycle++;

        // Prompt user if not running all remaining cycles
        if (!runAllRemaining && currentState != HALT) {
            std::cout << "Enter N=next, R=run remainder, E=exit: ";
            std::cin >> userInput;
            if (userInput == 'E' || userInput == 'e') {
                std::cout << "Exiting at user request.\n";
                break;
            } else if (userInput == 'R' || userInput == 'r') {
                runAllRemaining = true;
            }
        }
    }

    std::cout << "Simulation finished after " << clockCycle << " cycles.\n";
    return 0;
}