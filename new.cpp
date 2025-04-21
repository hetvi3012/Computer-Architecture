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
#include <unordered_map> // For branch prediction table

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

    int32_t RA, RB, RM; // Operands

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
// Pipeline registers
// =====================================================================
struct IF_ID {
    uint32_t PC;
    uint32_t IR;
    bool valid;
    bool isControlInstr; // New signal to indicate if the instruction is a control instruction
} if_id = {0, 0, false, false};

struct ID_EX {
    uint32_t PC;
    uint32_t IR;
    int32_t RA, RB, RM;
    DecodedInstr d;
    bool valid;
    bool forwardRAFromEX_MEM = false; // Forward RA from EX/MEM
    bool forwardRAFromMEM_WB = false; // Forward RA from MEM/WB
    bool forwardRBFromEX_MEM = false; // Forward RB from EX/MEM
    bool forwardRBFromMEM_WB = false; // Forward RB from MEM/WB
    bool forwardRMFromEX_MEM = false; // Forward RM from EX/MEM
    bool forwardRMFromMEM_WB = false; // Forward RM from MEM/WB
} id_ex = {0, 0, 0, 0, 0, {}, false};

struct EX_MEM {
    uint32_t PC;
    uint32_t IR;
    int32_t RZ, RM;
    DecodedInstr d;
    bool valid;
    bool forwardRMFromMEM_WB = false; // Forward RM from MEM/WB
} ex_mem = {0, 0, 0, 0, {}, false};

struct MEM_WB {
    uint32_t PC;
    uint32_t IR;
    int32_t RY;
    DecodedInstr d;
    bool valid;
} mem_wb = {0, 0, 0, {}, false};

// Function to detect RAW hazards
bool detectRAWHazard(const DecodedInstr &decodedInstr, const EX_MEM &ex_mem, const MEM_WB &mem_wb) {
    // Check if source registers in decoded instruction (rs1, rs2) match destination registers in EX/MEM or MEM/WB
    if (ex_mem.valid && ex_mem.d.regWrite && ex_mem.d.rd != 0) { // Check EX/MEM only if valid
        if (decodedInstr.rs1 == ex_mem.d.rd) {
            std::cout << "[RAW Hazard] Dependency detected with EX stage. rs1=" << decodedInstr.rs1 
                      << " matches rd=" << ex_mem.d.rd << "\n";
            return true; // Hazard with EX stage
        }
        if (decodedInstr.rs2 == ex_mem.d.rd) {
            std::cout << "[RAW Hazard] Dependency detected with EX stage. rs2=" << decodedInstr.rs2 
                      << " matches rd=" << ex_mem.d.rd << "\n";
            return true; // Hazard with EX stage
        }
    }
    if (mem_wb.valid && mem_wb.d.regWrite && mem_wb.d.rd != 0) { // Check MEM/WB only if valid
        if (decodedInstr.rs1 == mem_wb.d.rd) {
            std::cout << "[RAW Hazard] Dependency detected with MEM stage. rs1=" << decodedInstr.rs1 
                      << " matches rd=" << mem_wb.d.rd << "\n";
            return true; // Hazard with MEM stage
        }
        if (decodedInstr.rs2 == mem_wb.d.rd) {
            std::cout << "[RAW Hazard] Dependency detected with MEM stage. rs2=" << decodedInstr.rs2 
                      << " matches rd=" << mem_wb.d.rd << "\n";
            return true; // Hazard with MEM stage
        }
    }
    return false; // No hazard
}

// =====================================================================
// Control Hazard Detection Unit (CHDU)
// =====================================================================
class ControlHazardDetectionUnit {
public:
    bool stallPipeline = false; // Indicates if the pipeline should be stalled
    bool flushPipeline = false; // Indicates if the pipeline should be flushed
    bool branchTaken = false;   // Indicates if the branch condition is satisfied

    // Check for control hazards during decode
    void checkControlHazard(const DecodedInstr &d) {
        if (d.branch && !d.jump) {
            stallPipeline = true; // Stall pipeline if branch instruction is detected
        }
        else if (d.jump) {
            flushPipeline = true; // Flush pipeline if jump instruction is detected
        }
        else {
            stallPipeline = false; // Clear stall if no branch or jump
            flushPipeline = false; // Clear flush if no branch or jump
        }
    }

    // Resolve branch decision during execute
    void resolveBranch(bool zero, const DecodedInstr &d) {
        if (d.branch) {
            branchTaken = zero; // Branch is taken if ALU zero signal is true
            flushPipeline = branchTaken; // Flush pipeline if branch is taken
        }
        stallPipeline = false; // Clear stall after branch resolution
    }
};

// Global CHDU instance
ControlHazardDetectionUnit chdu;

// =====================================================================
// Control circuitry function
// =====================================================================
void controlCircuitry(const DecodedInstr &d, DecodedInstr &controlSignals) {
    // Reset all control signals
    controlSignals.regWrite = false;
    controlSignals.memRead = false;
    controlSignals.memWrite = false;
    controlSignals.branch = false;
    controlSignals.jump = false;
    controlSignals.memToReg = 0;
    controlSignals.memSize = 2; // Default to word
    controlSignals.memSignExtend = false;
    controlSignals.aluOp = ALU_PASS;

    // Update control signals based on opcode, funct3, and funct7
    switch (d.opcode) {
        case 0x33: // R-type
            controlSignals.regWrite = true;
            switch (d.funct3) {
                case 0x0:
                    controlSignals.aluOp = (d.funct7 == 0x20) ? ALU_SUB : 
                                            (d.funct7 == 0x01) ? ALU_MUL : ALU_ADD;
                    break;
                case 0x4:
                    controlSignals.aluOp = (d.funct7 == 0x01) ? ALU_DIV : ALU_XOR;
                    break;
                case 0x6:
                    controlSignals.aluOp = (d.funct7 == 0x01) ? ALU_REM : ALU_OR;
                    break;
                case 0x7:
                    controlSignals.aluOp = ALU_AND;
                    break;
                case 0x1:
                    controlSignals.aluOp = ALU_SLL;
                    break;
                case 0x2:
                    controlSignals.aluOp = ALU_SLT;
                    break;
                case 0x5:
                    controlSignals.aluOp = (d.funct7 == 0x20) ? ALU_SRA : ALU_SRL;
                    break;
                default:
                    break;
            }
            break;
        case 0x13: // I-type ALU
            controlSignals.regWrite = true;
            switch (d.funct3) {
                case 0x0:
                    controlSignals.aluOp = ALU_ADD; // ADDI
                    break;
                case 0x7:
                    controlSignals.aluOp = ALU_AND; // ANDI
                    break;
                case 0x6:
                    controlSignals.aluOp = ALU_OR; // ORI
                    break;
                case 0x4:
                    controlSignals.aluOp = ALU_XOR; // XORI
                    break;
                case 0x2:
                    controlSignals.aluOp = ALU_SLT; // SLTI
                    break;
                case 0x1:
                    controlSignals.aluOp = ALU_SLL; // SLLI
                    break;
                case 0x5:
                    controlSignals.aluOp = ((d.funct7 & 0x20) == 0x20) ? ALU_SRA : ALU_SRL; // SRLI/SRAI
                    break;
                default:
                    break;
            }
            break;
        case 0x03: // LOAD
            controlSignals.regWrite = true;
            controlSignals.memRead = true;
            controlSignals.aluOp = ALU_ADD; // Address calculation
            controlSignals.memToReg = 1; // Write-back from memory
            switch (d.funct3) {
                case 0x0: controlSignals.memSize = 0; controlSignals.memSignExtend = true; break; // LB
                case 0x1: controlSignals.memSize = 1; controlSignals.memSignExtend = true; break; // LH
                case 0x2: controlSignals.memSize = 2; controlSignals.memSignExtend = false; break; // LW
                case 0x4: controlSignals.memSize = 0; controlSignals.memSignExtend = false; break; // LBU
                case 0x5: controlSignals.memSize = 1; controlSignals.memSignExtend = false; break; // LHU
                default: break;
            }
            break;
        case 0x23: // STORE
            controlSignals.memWrite = true;
            controlSignals.aluOp = ALU_ADD; // Address calculation
            switch (d.funct3) {
                case 0x0: controlSignals.memSize = 0; break; // SB
                case 0x1: controlSignals.memSize = 1; break; // SH
                case 0x2: controlSignals.memSize = 2; break; // SW
                default: break;
            }
            break;
        case 0x63: // BRANCH
            controlSignals.branch = true;
            switch (d.funct3) {
                case 0x0: controlSignals.aluOp = ALU_SUB; break; // BEQ: RA - RB == 0
                case 0x1: controlSignals.aluOp = ALU_EQ; break;  // BNE: RA == RB
                case 0x4: controlSignals.aluOp = ALU_GE; break;  // BLT: RA >= RB
                case 0x5: controlSignals.aluOp = ALU_SLT; break; // BGE: RA < RB
                default: break;
            }
            break;
        case 0x6F: // JAL
            controlSignals.regWrite = true;
            controlSignals.jump = true;
            controlSignals.branch = true; // No branch for JAL
            controlSignals.aluOp = ALU_PASS; // No ALU operation needed for JAL
            controlSignals.memToReg = 2; // Write-back PC+4 (PCtemp from IAG)
            break;
        case 0x67: // JALR
            controlSignals.regWrite = true;
            controlSignals.jump = true;
            controlSignals.aluOp = ALU_ADD; // Calculate new PC using ALU
            controlSignals.memToReg = 2; // Write-back PC+4
            break;
        case 0x37: // LUI
            controlSignals.regWrite = true;
            controlSignals.aluOp = ALU_PASS; // Pass-through immediate
            break;
        case 0x17: // AUIPC
            controlSignals.regWrite = true;
            controlSignals.aluOp = ALU_ADD; // Add upper immediate to PC
            break;
        default:
            break;
    }
}

// =====================================================================
// decode
// =====================================================================
DecodedInstr decode(uint32_t instr) {
    DecodedInstr d{};
    d.opcode = getBits(instr, 6, 0);
    d.rd     = getBits(instr, 11, 7);
    d.funct3 = getBits(instr, 14, 12);
    d.rs1    = getBits(instr, 19, 15);

    // Some opcodes ignore rs2/funct7 in decode
    if(d.opcode == 0x33) d.funct7 = getBits(instr, 31, 25);
    if(d.opcode == 0x23 || d.opcode == 0x33 || d.opcode == 0x63) d.rs2 = getBits(instr, 24, 20);

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

    // Set RA, RB, RM based on the instruction type
    d.RA = (d.opcode == 0x17) ? PC : R[d.rs1]; // AUIPC uses PC
    d.RB = (d.opcode == 0x13 || d.opcode == 0x03 || d.opcode == 0x67 || d.opcode == 0x23) 
            ? d.imm 
            : R[d.rs2];
    d.RM = R[d.rs2];

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
// Forward declaration of getMemSegmentForAddress
// =====================================================================
MemSegment* getMemSegmentForAddress(uint32_t addr);

// =====================================================================
// Updated Memory Processor Interface
// =====================================================================
void memoryProcessorInterface(uint32_t &MAR, int32_t &MDR, int32_t RM, bool memRead, bool memWrite, uint8_t memSize, bool memSignExtend) {
    MemSegment* seg = getMemSegmentForAddress(MAR); // Use MAR as the memory address
    if (!seg) return; // Invalid memory segment

    if (memRead) {
        // Perform memory read based on size and store the result in MDR
        switch (memSize) {
            case 0: // Byte
                MDR = memSignExtend 
                      ? static_cast<int8_t>(seg->readByte(MAR))
                      : static_cast<uint8_t>(seg->readByte(MAR));
                break;
            case 1: // Halfword
                MDR = memSignExtend 
                      ? static_cast<int16_t>(seg->readByte(MAR) | (seg->readByte(MAR + 1) << 8))
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
        // Perform memory write using RM
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
// Branch Prediction Table (1-bit predictor)
// =====================================================================
std::unordered_map<uint32_t, bool> branchPredictionTable; // Maps PC to prediction (true = taken, false = not taken)

// Function to predict branch outcome
bool predictBranch(uint32_t pc) {
    auto it = branchPredictionTable.find(pc);
    return (it != branchPredictionTable.end()) ? it->second : false; // Default: not taken
}

// Function to update branch prediction table
void updateBranchPrediction(uint32_t pc, bool actualOutcome) {
    branchPredictionTable[pc] = actualOutcome; // Update prediction with actual outcome
}

// =====================================================================
// Updated Instruction Address Generator (IAG) with Branch Prediction
// =====================================================================
class IAG {
public:
    uint32_t PCtemp; // Temporary storage for PC + 4

    // Update PC based on signals and branch prediction
    void updatePC(bool jump, bool branch, bool zero, int32_t offset, int32_t RZ, uint32_t pc) {
        PCtemp = PC + 4; // Always store PC + 4 in PCtemp
        if (jump) {
            if (branch) {
                PC += offset; // For JAL, PC = PC + offset
            } else {
                PC = RZ & ~1U; // For JALR, PC = RZ (aligned to even address)
            }
        } else if (branch) {
            bool predictedTaken = predictBranch(pc); // Predict branch outcome
            if (predictedTaken) {
                PC += offset; // Predicted taken: PC = PC + offset
            } else {
                PC = PCtemp; // Predicted not taken: PC = PC + 4
            }
        } else {
            PC = PCtemp; // Default to PC + 4
        }
    }
};

// Global IAG instance
IAG iag;

// =====================================================================
// Updated Print Registers
// =====================================================================
void printRegisters() {
    std::cout << "Register File:\n";
    for (int i = 0; i < NUM_REGS; i++) {
        std::cout << "R[" << std::dec << i << "]=" << std::dec << R[i] << "   "; // Register number in decimal
        if ((i + 1) % 4 == 0) std::cout << "\n";
    }
    std::cout << "-------------------------------------\n";
    std::cout << "PC = 0x" << std::hex << PC 
              << "  RA=0x" << RA << "  RB=0x" << RB << "  RM=0x" << RM << "\n";
    std::cout << "RZ=0x" << RZ << "  RY=0x" << RY << "  MDR=0x" << MDR << "\n";
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

// Knobs
bool Knob2 = false; // Enable/disable data forwarding
bool Knob3 = true; // Enable/disable printing all the register file content at the end of each cycle
bool Knob4 = true; // Enable/disable printing pipeline registers at the end of each cycle
bool Knob5 = false; // Enable/disable tracing for a specific instruction
int Knob5InstructionNumber = 0; // Instruction number to trace if Knob5 is enabled
bool Knob6 = false; // Enable/disable printing branch prediction unit content

// =====================================================================
// Function to print the contents of pipeline buffers
// =====================================================================
void printPipelineBuffers() {
    std::cout << "================ Pipeline Buffers ================\n";

    // IF/ID Buffer
    if (if_id.valid) {
        std::cout << "IF/ID: PC=0x" << std::hex << if_id.PC << " IR=0x" << if_id.IR 
                  << " Valid=1\n";
    } else if (if_id.PC == 0 && if_id.IR == 0) {
        std::cout << "IF/ID: Empty\n";
    } else {
        std::cout << "IF/ID: Bubble (Valid=0)\n";
    }

    // ID/EX Buffer
    if (id_ex.valid) {
        std::cout << "ID/EX: PC=0x" << std::hex << id_ex.PC << " IR=0x" << id_ex.IR 
                  << " RA=" << id_ex.RA << " RB=" << id_ex.RB << " RM=" << id_ex.RM 
                  << " Valid=1\n";
    } else if (id_ex.PC == 0 && id_ex.IR == 0) {
        std::cout << "ID/EX: Empty\n";
    } else {
        std::cout << "ID/EX: Bubble (Valid=0)\n";
    }

    // EX/MEM Buffer
    if (ex_mem.valid) {
        std::cout << "EX/MEM: PC=0x" << std::hex << ex_mem.PC << " IR=0x" << ex_mem.IR 
                  << " RZ=" << ex_mem.RZ << " RM=" << ex_mem.RM 
                  << " Valid=1\n";
    } else if (ex_mem.PC == 0 && ex_mem.IR == 0) {
        std::cout << "EX/MEM: Empty\n";
    } else {
        std::cout << "EX/MEM: Bubble (Valid=0)\n";
    }

    // MEM/WB Buffer
    if (mem_wb.valid) {
        std::cout << "MEM/WB: PC=0x" << std::hex << mem_wb.PC << " IR=0x" << mem_wb.IR 
                  << " RY=" << mem_wb.RY 
                  << " Valid=1\n";
    } else if (mem_wb.PC == 0 && mem_wb.IR == 0) {
        std::cout << "MEM/WB: Empty\n";
    } else {
        std::cout << "MEM/WB: Bubble (Valid=0)\n";
    }

    std::cout << "=================================================\n";
}

// =====================================================================
// Function to print unresolved dependencies
// =====================================================================
void printUnresolvedDependencies(const std::multiset<uint32_t> &dependencies) {
    std::cout << "Unresolved Dependencies: ";
    if (dependencies.empty()) {
        std::cout << "None";
    } else {
        for (const auto &dep : dependencies) {
            std::cout << "R[" << dep << "] ";
        }
    }
    std::cout << "\n";
}

// =====================================================================
// Function to print branch prediction unit content
// =====================================================================
void printBranchPredictionUnit() {
    std::cout << "Branch Prediction Unit:\n";
    for (const auto &entry : branchPredictionTable) {
        std::cout << "PC=0x" << std::hex << entry.first 
                  << " Prediction=" << (entry.second ? "Taken" : "Not Taken") << "\n";
    }
    std::cout << "-------------------------------------\n";
}

// Statistics tracking variables
uint64_t totalCycles = 0;
uint64_t totalInstructions = 0;
uint64_t dataTransferInstructions = 0;
uint64_t aluInstructions = 0;
uint64_t controlInstructions = 0;
uint64_t pipelineStalls = 0;
uint64_t dataHazards = 0;
uint64_t controlHazards = 0;
uint64_t branchMispredictions = 0;
uint64_t dataHazardStalls = 0;
uint64_t controlHazardStalls = 0;

// =====================================================================
// Pre-update dependencies before any stage begins
// =====================================================================
void preUpdateDependencies() {
    // Update ID/EX values from EX/MEM or MEM/WB
    if (id_ex.valid) {
        id_ex.RA = id_ex.d.RA; // Default to original RA
        id_ex.RB = id_ex.d.RB; // Default to original RB
        id_ex.RM = id_ex.d.RM; // Default to original RM

        if (id_ex.forwardRAFromEX_MEM) {
            id_ex.RA = ex_mem.RZ; // Forward RA from EX/MEM
            std::cout << "[Forwarding] RZ = " << ex_mem.RZ << " to RA\n";
        }  if (id_ex.forwardRAFromMEM_WB) {
            id_ex.RA = mem_wb.RY; // Forward RA from MEM/WB
            std::cout << "[Forwarding] RY = " << mem_wb.RY << " to RA\n";
        }

        if (id_ex.forwardRBFromEX_MEM) {
            id_ex.RB = ex_mem.RZ; // Forward RB from EX/MEM
            std::cout << "[Forwarding] RZ = " << ex_mem.RZ << " to RB\n";
        }  if (id_ex.forwardRBFromMEM_WB) {
            id_ex.RB = mem_wb.RY; // Forward RB from MEM/WB
            std::cout << "[Forwarding] RY = " << mem_wb.RY << " to RB\n";
        }

        if (id_ex.forwardRMFromEX_MEM) {
            id_ex.RM = ex_mem.RZ; // Forward RM from EX/MEM
        }  if (id_ex.forwardRMFromMEM_WB) {
            id_ex.RM = mem_wb.RY; // Forward RM from MEM/WB
        }
    }

    // Update EX/MEM values from MEM/WB
    if (ex_mem.valid) {
        if (ex_mem.forwardRMFromMEM_WB) {
            ex_mem.RM = mem_wb.RY; // Forward RM from MEM/WB
        }
    }
}

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

    bool stallSignal = false; // Initialize stall signal

    // Track dependencies for RAW hazards
    std::multiset<uint32_t> unresolvedDependencies;

    // Function to check if all dependencies are resolved
    auto areDependenciesResolved = [&]() {
        return unresolvedDependencies.empty();
    };

    while (currentState != HALT) {
        std::cout << "Clock Cycle: " << std::dec << clockCycle << "\n"; // Cycle number in decimal

        // Increment total cycles
        totalCycles++;

        // Pre-update dependencies before any stage begins
        preUpdateDependencies();

        // Print branch prediction unit if Knob6 is enabled
        if (Knob6) {
            printBranchPredictionUnit();
        }

        // Print unresolved dependencies
        printUnresolvedDependencies(unresolvedDependencies);

        // Write Back (MEM_WB)
        if (mem_wb.valid) { // Write Back only if MEM_WB is valid
            totalInstructions++; // Increment total instructions executed
            if (mem_wb.d.memRead || mem_wb.d.memWrite) {
                dataTransferInstructions++; // Increment data-transfer instructions
            } else if (mem_wb.d.branch || mem_wb.d.jump) {
                controlInstructions++; // Increment control instructions
            } else {
                aluInstructions++; // Increment ALU instructions
            }

            if (mem_wb.d.regWrite) {
                std::cout << "[Write Back] Writing R[" << std::dec << mem_wb.d.rd << "] = " << mem_wb.RY << "\n"; // Register number in decimal
                R[mem_wb.d.rd] = mem_wb.RY;
                R[0] = 0; // Ensure x0 is always 0

                // Remove resolved dependency
                unresolvedDependencies.erase(mem_wb.d.rd);
            }
            printUnresolvedDependencies(unresolvedDependencies); // Print unresolved dependencies after write-back

            std::cout << "[Write Back] PC=0x" << std::hex << mem_wb.PC << " IR=0x" << mem_wb.IR << "\n";

            // Check if all dependencies are resolved
            if (stallSignal && areDependenciesResolved()) {
                stallSignal = false; // Clear stall signal
                std::cout << "[Write Back] All dependencies resolved. Resuming pipeline.\n";
            }
        } else if (mem_wb.IR == 0 && !mem_wb.valid) {
            std::cout << "[Write Back] Bubble detected in MEM/WB.\n";
        }

        bool finalStallSignal = false;

        // Memory Access (EX_MEM -> MEM_WB)
        if (ex_mem.valid) { // Memory Access only if EX_MEM is valid
            mem_wb.PC = ex_mem.PC;
            mem_wb.IR = ex_mem.IR;
            mem_wb.d = ex_mem.d;
            mem_wb.valid = true;

            // Set MAR to the address calculated by the ALU (RZ)
            MAR = ex_mem.RZ;

            // Use memoryProcessorInterface to handle LOAD/STORE
            memoryProcessorInterface(MAR, MDR, ex_mem.RM, ex_mem.d.memRead, ex_mem.d.memWrite, ex_mem.d.memSize, ex_mem.d.memSignExtend);

            // Ensure memRead is correctly used
            if (ex_mem.d.memRead) {
                std::cout << "[Memory Access] LOAD instruction: Reading data into MDR.\n";
            }

            // Determine the value of RY based on control signals
            if (ex_mem.d.memToReg == 1) {
                mem_wb.RY = MDR; // Load: Use data from memory
            } else if (ex_mem.d.memToReg == 2) {
                mem_wb.RY = ex_mem.PC + 4; // JAL/JALR: Use return address
            } else {
                mem_wb.RY = ex_mem.RZ; // Default: Use ALU result
            }

            std::cout << "[Memory Access] MAR=0x" << std::hex << MAR << " MDR=" << MDR << " RY=" << mem_wb.RY << "\n";
        } else {
            mem_wb.valid = false; // No valid instruction to access memory
        }

        bool updatePC_ex_mem = false; // Flag to indicate if PC should be updated
        bool updatePC_id_ex = false; // Flag to indicate if PC should be updated in ID_EX

        // Execute (ID_EX -> EX_MEM)
        if (id_ex.valid) { // Execute only if ID_EX is valid
            ex_mem.PC = id_ex.PC;
            ex_mem.IR = id_ex.IR;
            ex_mem.d = id_ex.d;
            ex_mem.valid = true;

            // Perform ALU operation
            switch (id_ex.d.aluOp) {
                case ALU_ADD: ex_mem.RZ = id_ex.RA + id_ex.RB; break;
                case ALU_SUB: ex_mem.RZ = id_ex.RA - id_ex.RB; break;
                case ALU_MUL: ex_mem.RZ = id_ex.RA * id_ex.RB; break;
                case ALU_DIV: ex_mem.RZ = (id_ex.RB != 0) ? id_ex.RA / id_ex.RB : 0; break;
                case ALU_REM: ex_mem.RZ = (id_ex.RB != 0) ? id_ex.RA % id_ex.RB : 0; break;
                case ALU_AND: ex_mem.RZ = id_ex.RA & id_ex.RB; break;
                case ALU_OR: ex_mem.RZ = id_ex.RA | id_ex.RB; break;
                case ALU_XOR: ex_mem.RZ = id_ex.RA ^ id_ex.RB; break;
                case ALU_SLL: ex_mem.RZ = id_ex.RA << (id_ex.RB & 0x1F); break;
                case ALU_SRL: ex_mem.RZ = static_cast<int32_t>(static_cast<uint32_t>(id_ex.RA) >> (id_ex.RB & 0x1F)); break;
                case ALU_SRA: ex_mem.RZ = id_ex.RA >> (id_ex.RB & 0x1F); break;
                case ALU_SLT: ex_mem.RZ = (id_ex.RA < id_ex.RB) ? 1 : 0; break;
                case ALU_EQ: ex_mem.RZ = (id_ex.RA == id_ex.RB) ? 1 : 0; break;
                case ALU_GE: ex_mem.RZ = (id_ex.RA >= id_ex.RB) ? 1 : 0; break;
                case ALU_PASS: ex_mem.RZ = id_ex.d.imm; break;
                default: ex_mem.RZ = 0; break;
            }
            ex_mem.RM = id_ex.RM;

            // Restore zero signal functionality
            id_ex.d.zero = (ex_mem.RZ == 0); // Set zero signal if ALU result is zero

            // Resolve branch decision
            if (id_ex.d.branch && !id_ex.d.jump) {
                chdu.resolveBranch(id_ex.d.zero, id_ex.d); // Use zero signal for branch resolution
                bool actualOutcome = chdu.branchTaken; // Actual branch outcome
                bool predictedOutcome = predictBranch(id_ex.PC); // Predicted branch outcome

                if (actualOutcome == predictedOutcome) {
                    std::cout << "[Execute] Branch prediction was correct. Continuing pipeline.\n";
                } else {
                    std::cout << "[Execute] Branch prediction was incorrect. Flushing the next instruction.\n";
                    if_id.valid = false; // Flush the instruction in IF/ID (next instruction)
                    PC = id_ex.PC + (actualOutcome ? id_ex.d.imm : 4); // Correct PC
                }

                // Update branch prediction table with the actual outcome
                updateBranchPrediction(id_ex.PC, actualOutcome);
            }

            // Handle jump instructions (JAL, JALR) without flushing the pipeline
            if (id_ex.d.jump && !id_ex.d.branch) {
                std::cout << "[Execute] Jump detected. Updating PC without flushing pipeline.\n";
                PC = (id_ex.d.opcode == 0x6F) ? id_ex.PC + id_ex.d.imm : (id_ex.RA + id_ex.d.imm) & ~1U; // Update PC for JAL or JALR
            }

            std::cout << "[Execute] RZ=" << ex_mem.RZ << " RM=" << ex_mem.RM << " Zero=" << id_ex.d.zero << "\n";
        } else {
            ex_mem.valid = false; // No valid instruction to execute
        }

        // Decode (IF_ID -> ID_EX)
        if (!stallSignal && if_id.IR != 0 && if_id.valid) { // Decode only if no stall signal, IF_ID is valid, and IR is not empty
            id_ex.PC = if_id.PC;
            id_ex.IR = if_id.IR;
            id_ex.d = decode(if_id.IR);

            // Generate control signals using control circuitry
            controlCircuitry(id_ex.d, id_ex.d);

            // Ensure memRead is correctly toggled for LOAD instructions
            if (id_ex.d.memRead) {
                std::cout << "[Decode] LOAD instruction detected. memRead enabled.\n";
            }

            // Default forwarding control signals
            id_ex.forwardRAFromEX_MEM = false;
            id_ex.forwardRAFromMEM_WB = false;
            id_ex.forwardRBFromEX_MEM = false;
            id_ex.forwardRBFromMEM_WB = false;
            id_ex.forwardRMFromEX_MEM = false;
            id_ex.forwardRMFromMEM_WB = false;

            // Check for RAW hazards (data dependencies)
            if (detectRAWHazard(id_ex.d, ex_mem, mem_wb)) {
                if (Knob2) { // Data forwarding enabled
                    // Forward data from EX/MEM to ID/EX
                    if (ex_mem.valid && ex_mem.d.regWrite && ex_mem.d.rd != 0) {
                        if (id_ex.d.rs1 == ex_mem.d.rd) {
                            id_ex.forwardRAFromEX_MEM = true; // Signal to forward RA from EX/MEM
                            std::cout << "[Forwarding] EX/MEM -> ID/EX: Forwarding RZ=" << ex_mem.RZ << " to RA\n";
                        }
                        // if (id_ex.d.rs2 == ex_mem.d.rd) {
                        //     id_ex.forwardRBFromEX_MEM = true; // Signal to forward RB from EX/MEM
                        //     std::cout << "[Forwarding] EX/MEM -> ID/EX: Forwarding RZ=" << ex_mem.RZ << " to RB\n";
                        // }
                    }

                    // Forward data from MEM/WB to ID/EX
                    if (mem_wb.valid && mem_wb.d.regWrite && mem_wb.d.rd != 0) {
                        if (id_ex.d.rs1 == mem_wb.d.rd) {
                            id_ex.forwardRAFromMEM_WB = true; // Signal to forward RA from MEM/WB
                            std::cout << "[Forwarding] MEM/WB -> ID/EX: Forwarding RY=" << mem_wb.RY << " to RA\n";
                        }
                        // if (id_ex.d.rs2 == mem_wb.d.rd) {
                        //     id_ex.forwardRBFromMEM_WB = true; // Signal to forward RB from MEM/WB
                        //     std::cout << "[Forwarding] MEM/WB -> ID/EX: Forwarding RY=" << mem_wb.RY << " to RB\n";
                        // }
                    }

                    // Forward RM for store instructions
                    if (id_ex.d.memWrite) {
                        if (ex_mem.valid && ex_mem.d.regWrite && ex_mem.d.rd != 0 && id_ex.d.rs2 == ex_mem.d.rd) {
                            id_ex.forwardRMFromEX_MEM = true; // Signal to forward RM from EX/MEM
                            std::cout << "[Forwarding] EX/MEM -> ID/EX: Forwarding RZ=" << ex_mem.RZ << " to RM\n";
                        }
                        if (mem_wb.valid && mem_wb.d.regWrite && mem_wb.d.rd != 0 && id_ex.d.rs2 == mem_wb.d.rd) {
                            id_ex.forwardRMFromMEM_WB = true; // Signal to forward RM from MEM/WB
                            std::cout << "[Forwarding] MEM/WB -> ID/EX: Forwarding RY=" << mem_wb.RY << " to RM\n";
                        }
                    }

                    // Handle load-use hazard (stall for one cycle)
                    if (ex_mem.valid && ex_mem.d.memRead && (id_ex.d.rs1 == ex_mem.d.rd || id_ex.d.rs2 == ex_mem.d.rd)) {
                        dataHazardStalls++; // Increment stalls due to data hazards
                        pipelineStalls++; // Increment pipeline stalls
                        stallSignal = true; // Stall the pipeline for one cycle
                        finalStallSignal = true; // Set final stall signal
                        id_ex.valid = false; // Create a bubble in ID/EX
                        std::cout << "[Stall] Load-use hazard detected. Stalling pipeline for one cycle.\n";
                    } else {
                        id_ex.valid = true; // Mark ID_EX as valid
                    }
                } else { // Data forwarding disabled
                    dataHazards++; // Increment data hazards
                    dataHazardStalls++; // Increment stalls due to data hazards
                    pipelineStalls++; // Increment pipeline stalls
                    id_ex.valid = false; // Stall the decode stage
                    stallSignal = true; // Set stall signal
                    finalStallSignal = true; // Set final stall signal

                    // Add unresolved dependencies
                    if (ex_mem.valid && ex_mem.d.regWrite && ex_mem.d.rd != 0) {
                        if (id_ex.d.rs1 == ex_mem.d.rd || id_ex.d.rs2 == ex_mem.d.rd) {
                            unresolvedDependencies.insert(ex_mem.d.rd);
                        }
                    }
                    if (mem_wb.valid && mem_wb.d.regWrite && mem_wb.d.rd != 0) {
                        if (id_ex.d.rs1 == mem_wb.d.rd || id_ex.d.rs2 == mem_wb.d.rd) {
                            unresolvedDependencies.insert(mem_wb.d.rd);
                        }
                    }

                    std::cout << "[Stall] RAW hazard detected. Stalling Decode stage.\n";
                }
                std :: cout << "RA:" << id_ex.RA << " RB:" << id_ex.RB << " RM:" << id_ex.RM << "\n";
            } else {
                chdu.checkControlHazard(id_ex.d);

                if (chdu.stallPipeline) {
                    controlHazards++; // Increment control hazards
                    controlHazardStalls++; // Increment stalls due to control hazards
                    pipelineStalls++; // Increment pipeline stalls

                    // Forward branch data to ID/EX buffer
                    id_ex.RA = id_ex.d.RA;
                    id_ex.RB = id_ex.d.RB;
                    id_ex.RM = id_ex.d.RM;
                    id_ex.valid = true; // Mark ID_EX as valid
                    // stallSignal = true; // Set stall signal
                    finalStallSignal = true; // Set final stall signal
                    std::cout << "[Decode] Control hazard detected for conditional branch. Waiting for EX stage.\n";
                } else if (chdu.flushPipeline && !id_ex.d.jump) { // Do not flush for JAL or JALR
                    branchMispredictions++; // Increment branch mispredictions
                    std::cout << "[Decode] Flushing pipeline due to branch misprediction.\n";
                    id_ex.RA = id_ex.d.RA;
                    id_ex.RB = id_ex.d.RB;
                    id_ex.RM = id_ex.d.RM;
                    id_ex.valid = true; // Mark ID_EX as valid
                    if_id.valid = false; // Flush IF/ID
                    if (id_ex.d.branch) updatePC_id_ex = true; // Set flag to update PC
                } else {
                    // Forward RA, RB, RM to ID_EX buffer if no stall or flush
                    id_ex.RA = id_ex.d.RA;
                    id_ex.RB = id_ex.d.RB;
                    id_ex.RM = id_ex.d.RM;
                    id_ex.valid = true; // Mark ID_EX as valid
                }
            }
            
        }  else if (stallSignal) {
            // Still have unresolved RAW hazard → count another stall
            dataHazardStalls++;
            pipelineStalls++;
            finalStallSignal = true;
            std::cout << "[Decode] Stalled due to unresolved RAW hazard. Bubble created in ID_EX.\n";
            id_ex.valid = false;
        }
        
        else {
            id_ex.valid = false; // No valid instruction to decode
        }

        // Fetch (PC -> IF_ID) with Control Instruction Signal and Prediction
        if (!stallSignal) { // Fetch only if no stall signal is detected
            if(chdu.stallPipeline) {
                stallSignal = true; // Set stall signal if control hazard detected
                finalStallSignal = true; // Set final stall signal
            }
            auto it = instrMemory.find(PC);
            if (it != instrMemory.end()) {
                if_id.PC = PC;
                if_id.IR = it->second;
                if_id.valid = true; // Mark IF_ID as valid

                // Decode opcode to determine if the instruction is a control instruction
                uint32_t opcode = getBits(if_id.IR, 6, 0);
                if (opcode == 0x63 || opcode == 0x6F || opcode == 0x67) { // Branch, JAL, JALR
                    if_id.isControlInstr = true;

                    if (opcode == 0x6F || opcode == 0x67) { // JAL or JALR
                        // Direct jump: Update PC immediately
                        PC = (opcode == 0x6F) ? PC + decode(if_id.IR).imm : (R[getBits(if_id.IR, 19, 15)] + decode(if_id.IR).imm) & ~1U;
                        std::cout << "[Fetch] Jump detected. PC updated to 0x" << std::hex << PC << "\n";
                    } else if (opcode == 0x63) { // Conditional branch
                        // Predict branch outcome
                        if (predictBranch(PC)) {
                            PC += decode(if_id.IR).imm; // Predicted taken: Update PC with offset
                            std::cout << "[Fetch] Branch predicted taken. PC updated to 0x" << std::hex << PC << "\n";
                        } else {
                            PC += 4; // Predicted not taken: Increment PC
                            std::cout << "[Fetch] Branch predicted not taken. PC updated to 0x" << std::hex << PC << "\n";
                        }
                    }
                } else {
                    if_id.isControlInstr = false; // Not a control instruction
                    PC += 4; // Increment PC for next instruction fetch
                }

                std::cout << "[Fetch] PC=0x" << std::hex << if_id.PC << " IR=0x" << if_id.IR 
                          << " isControlInstr=" << if_id.isControlInstr << "\n";
            } else {
                std::cout << "[Fetch] No valid instruction to fetch. IF_ID retains its content.\n";
            }
        } else {
            std::cout << "[Fetch] Stalled due to stall signal. IF_ID retains its content.\n";
        }

        if(updatePC_ex_mem) {
            if(ex_mem.d.branch) PC = ex_mem.PC + ex_mem.d.imm; // Update PC using EX_MEM
            else PC = ex_mem.RZ; // Update PC using EX_MEM
            if_id.valid = false; // Flush IF/ID
        }
        else if(updatePC_id_ex) {
            PC = id_ex.PC + id_ex.d.imm; // Update PC using ID_EX
            if_id.valid = false; // Flush IF/ID
        }

        stallSignal = finalStallSignal; // Update stall signal for the next cycle

        // Check for termination condition
        if (if_id.IR == 0 && !id_ex.valid && !ex_mem.valid && !mem_wb.valid) {
            std::cout << "[Termination] All pipeline buffers are empty. Halting simulation.\n";
            currentState = HALT;
        }

        // Dump memory segments to files every cycle
        dumpSegmentToFile("data.mc", dataSegment, 0x10000000, 0x7FFFFFFF);
        dumpSegmentToFile("stack.mc", stackSegment, 0x7FFFFFFF, 0xFFFFFFFF);

        // Print pipeline buffers at the end of the cycle if Knob4 is enabled
        if (Knob4) {
            printPipelineBuffers();
        }

        // Print pipeline buffers for a specific instruction if Knob5 is enabled
        if (Knob5) {
            uint32_t targetPC = (Knob5InstructionNumber - 1) * 4; // Calculate PC for the specified instruction number

            // Check IF/ID buffer
            if (if_id.valid && if_id.PC == targetPC) {
                std::cout << "[Knob5] Tracing IF/ID buffer for instruction number " << Knob5InstructionNumber << ":\n";
                std::cout << "IF/ID: PC=0x" << std::hex << if_id.PC << " IR=0x" << if_id.IR << " Valid=1\n";
            }

            // Check ID/EX buffer
            if (id_ex.valid && id_ex.PC == targetPC) {
                std::cout << "[Knob5] Tracing ID/EX buffer for instruction number " << Knob5InstructionNumber << ":\n";
                std::cout << "ID/EX: PC=0x" << std::hex << id_ex.PC << " IR=0x" << id_ex.IR 
                          << " RA=" << id_ex.RA << " RB=" << id_ex.RB << " RM=" << id_ex.RM << " Valid=1\n";
            }

            // Check EX/MEM buffer
            if (ex_mem.valid && ex_mem.PC == targetPC) {
                std::cout << "[Knob5] Tracing EX/MEM buffer for instruction number " << Knob5InstructionNumber << ":\n";
                std::cout << "EX/MEM: PC=0x" << std::hex << ex_mem.PC << " IR=0x" << ex_mem.IR 
                          << " RZ=" << ex_mem.RZ << " RM=" << ex_mem.RM << " Valid=1\n";
            }

            // Check MEM/WB buffer
            if (mem_wb.valid && mem_wb.PC == targetPC) {
                std::cout << "[Knob5] Tracing MEM/WB buffer for instruction number " << Knob5InstructionNumber << ":\n";
                std::cout << "MEM/WB: PC=0x" << std::hex << mem_wb.PC << " IR=0x" << mem_wb.IR 
                          << " RY=" << mem_wb.RY << " Valid=1\n";
            }
        }

        // Print register file if Knob3 is enabled
        if (Knob3) {
            printRegisters();
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

    // Print statistics at the end of the simulation
    std::cout << "\n================ Simulation Statistics ================\n";
    std::cout << "Stat1: Total number of cycles = " << std::dec << totalCycles << "\n";
    std::cout << "Stat2: Total instructions executed = " << std::dec << totalInstructions << "\n";
    std::cout << "Stat3: CPI = " << std::fixed << std::setprecision(2) 
              << (totalCycles / static_cast<double>(totalInstructions)) << "\n";
    std::cout << "Stat4: Number of Data-transfer instructions executed = " << std::dec << dataTransferInstructions << "\n";
    std::cout << "Stat5: Number of ALU instructions executed = " << std::dec << aluInstructions << "\n";
    std::cout << "Stat6: Number of Control instructions executed = " << std::dec << controlInstructions << "\n";
    std::cout << "Stat7: Number of stalls/bubbles in the pipeline = " << std::dec << pipelineStalls << "\n";
    std::cout << "Stat8: Number of data hazards = " << std::dec << dataHazards << "\n";
    std::cout << "Stat9: Number of control hazards = " << std::dec << controlHazards << "\n";
    std::cout << "Stat10: Number of branch mispredictions = " << std::dec << branchMispredictions << "\n";
    std::cout << "Stat11: Number of stalls due to data hazards = " << std::dec << dataHazardStalls << "\n";
    std::cout << "Stat12: Number of stalls due to control hazards = " << std::dec << controlHazardStalls << "\n";
    std::cout << "=======================================================\n";

    std::cout << "Simulation finished after " << std::dec << clockCycle << " cycles.\n";
    return 0;
}