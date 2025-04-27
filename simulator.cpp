
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
#include <algorithm>
#include <set>

// ----------------------------------------------------------------------
// CPU State Variables
// ----------------------------------------------------------------------
static const int TOTAL_REGISTERS = 32;
int32_t cpuRegisters[TOTAL_REGISTERS];   // General-purpose registers

uint32_t pc = 0;             // Program Counter
uint32_t ir = 0;             // Instruction Register (holds the fetched instruction)
int32_t operand1 = 0;        // First ALU operand
int32_t operand2 = 0;        // Second ALU operand or immediate value
int32_t storeValue = 0;      // Value used for store instructions
int32_t aluOutput = 0;       // Result from ALU operations
int32_t writeBackData = 0;   // Data to be written back to a register
int32_t memDataReg = 0;      // Register for data loaded from memory

uint64_t cycleCount = 0;     // Clock cycle counter

// ----------------------------------------------------------------------
// Instruction Memory
// ----------------------------------------------------------------------
// This map holds instructions for addresses below 0x10000000.
std::map<uint32_t, uint32_t> instrMemory;

// ----------------------------------------------------------------------
// MemorySegment Class
// ----------------------------------------------------------------------
// This class represents a memory segment where each address maps to a byte.
// It is used for both the main data memory and the stack memory.
class MemorySegment {
public:
    std::map<uint32_t, uint8_t> memBytes;  // Underlying storage mapping addresses to bytes

    // Write a single byte at the specified address.
    void writeByte(uint32_t address, uint8_t value) {
        memBytes[address] = value;
    }

    // Write a 32-bit word in little-endian format.
    void writeWord(uint32_t address, int32_t word) {
        for (int i = 0; i < 4; i++) {
            memBytes[address + i] = static_cast<uint8_t>((word >> (8 * i)) & 0xFF);
        }
    }

    // Read a single signed byte from memory.
    int8_t readByte(uint32_t address) const {
        auto iter = memBytes.find(address);
        if (iter != memBytes.end()) {
            return static_cast<int8_t>(iter->second);
        }
        return 0;
    }

    // Read a 32-bit word from memory (little-endian).
    int32_t readWord(uint32_t address) const {
        int32_t result = 0;
        for (int i = 0; i < 4; i++) {
            auto iter = memBytes.find(address + i);
            uint8_t b = (iter != memBytes.end()) ? iter->second : 0;
            result |= (b << (8 * i));
        }
        return result;
    }
};

// ----------------------------------------------------------------------
// Data and Stack Memory Segments
// ----------------------------------------------------------------------
// The data segment covers addresses in the range [0x10000000, 0x7FFFFFFF),
// while the stack segment is used for addresses >= 0x7FFFFFFF.
MemorySegment dataMemory;
MemorySegment stackMemory;
// ----------------------------------------------------------------------
// Dump a memory segment to a file
// ----------------------------------------------------------------------
// This function writes each 4-byte-aligned address (within the specified
// range) and its corresponding 32-bit word from the given memory segment.
void dumpMemorySegmentToFile(const std::string &filename,
    const MemorySegment &segment,
    uint32_t rangeStart,
    uint32_t rangeEnd) {
std::ofstream outFile(filename);
if (!outFile.is_open()) {
std::cerr << "ERROR: Unable to open or create " << filename << "\n";
return;
}

// Collect all addresses present in the segment
std::vector<uint32_t> addresses;
addresses.reserve(segment.memBytes.size());
for (const auto &entry : segment.memBytes) {
addresses.push_back(entry.first);
}
std::sort(addresses.begin(), addresses.end());

// Use a set to ensure we process each 4-byte block only once.
std::set<uint32_t> processed;

for (uint32_t addr : addresses) {
if (addr < rangeStart)
continue;
if (rangeEnd >= rangeStart && addr >= rangeEnd)
continue;

// Process only 4-byte aligned addresses
if (addr % 4 != 0)
continue;
if (processed.count(addr) != 0)
continue;

// Read the 32-bit word at this address
int32_t wordValue = segment.readWord(addr);

outFile << std::hex << "0x"
<< std::setw(8) << std::setfill('0') << addr << "  0x"
<< std::setw(8) << std::setfill('0') << static_cast<uint32_t>(wordValue)
<< std::dec << "\n";

// Mark the four bytes as processed
processed.insert(addr);
processed.insert(addr + 1);
processed.insert(addr + 2);
processed.insert(addr + 3);
}

outFile.close();
}

// ----------------------------------------------------------------------
// Dump Instruction Memory to File
// ----------------------------------------------------------------------
// This function writes the 32-bit instructions stored in the instruction
// memory map to a file.
void dumpInstrMemoryToFile(const std::string &filename) {
std::ofstream outFile(filename);
if (!outFile.is_open()) {
std::cerr << "ERROR: Unable to open or create " << filename << "\n";
return;
}

// Collect instruction addresses
std::vector<uint32_t> instrAddresses;
instrAddresses.reserve(instrMemory.size());
for (const auto &entry : instrMemory) {
instrAddresses.push_back(entry.first);
}
std::sort(instrAddresses.begin(), instrAddresses.end());

// Write each instruction
for (uint32_t address : instrAddresses) {
uint32_t instructionWord = instrMemory[address];
outFile << std::hex << "0x" << std::setw(8) << std::setfill('0') << address
<< "  0x" << std::setw(8) << std::setfill('0') << instructionWord
<< std::dec << "\n";
}
outFile.close();
}

// ----------------------------------------------------------------------
// Helper Functions for Bit Manipulation
// ----------------------------------------------------------------------

// Extract a sequence of bits from 'value' between positions 'lo' and 'hi' (inclusive)
static inline uint32_t extractBits(uint32_t value, int hi, int lo) {
uint32_t mask = (1u << (hi - lo + 1)) - 1;
return (value >> lo) & mask;
}

// Extend the sign of a bit-field value assuming it is 'bitWidth' bits wide.
static inline int32_t extendSign(uint32_t value, int bitWidth) {
int shift = 32 - bitWidth;
return (int32_t)((int32_t)(value << shift) >> shift);
}
//=====================================================================
// InstructionComponents Structure
//=====================================================================
// This structure holds the decomposed fields of a 32-bit instruction.
struct InstructionComponents {
    uint32_t opCode;    // Main opcode field
    uint32_t rd;        // Destination register index
    uint32_t rs1;       // First source register index
    uint32_t rs2;       // Second source register index
    uint32_t func3;     // 3-bit function field
    uint32_t func7;     // 7-bit function field
    int32_t  imm;       // Immediate value
};

//=====================================================================
// Function: decodeInstruction
// Purpose: Decompose a 32-bit instruction into its individual fields.
//=====================================================================
InstructionComponents decodeInstruction(uint32_t inst) {
    InstructionComponents comp{};
    comp.opCode = extractBits(inst, 6, 0);
    comp.rd     = extractBits(inst, 11, 7);
    comp.func3  = extractBits(inst, 14, 12);
    comp.rs1    = extractBits(inst, 19, 15);

    // For opcodes that do not require the second source and func7 fields.
    if (comp.opCode == 0x13 || comp.opCode == 0x03 || comp.opCode == 0x67) {
        comp.rs2 = 0;
        comp.func7 = 0;
    } else {
        comp.rs2   = extractBits(inst, 24, 20);
        comp.func7 = extractBits(inst, 31, 25);
    }

    // Decode immediate value based on instruction type.
    switch(comp.opCode) {
        case 0x13: // I-type arithmetic
        case 0x03: // I-type load
        case 0x67: // I-type jump (JALR)
        {
            uint32_t imm12 = extractBits(inst, 31, 20);
            comp.imm = extendSign(imm12, 12);
            break;
        }
        case 0x23: // S-type (store)
        {
            uint32_t immHigh = extractBits(inst, 31, 25);
            uint32_t immLow  = extractBits(inst, 11, 7);
            uint32_t imm12 = (immHigh << 5) | immLow;
            comp.imm = extendSign(imm12, 12);
            break;
        }
        case 0x63: // SB-type (branch)
        {
            uint32_t bit12   = extractBits(inst, 31, 31);
            uint32_t bit11   = extractBits(inst, 7, 7);
            uint32_t bits10_5 = extractBits(inst, 30, 25);
            uint32_t bits4_1  = extractBits(inst, 11, 8);
            uint32_t fullImm = (bit12 << 12) | (bit11 << 11)
                             | (bits10_5 << 5) | (bits4_1 << 1);
            comp.imm = extendSign(fullImm, 13);
            break;
        }
        case 0x37: // LUI
        case 0x17: // AUIPC
        {
            uint32_t imm20 = extractBits(inst, 31, 12);
            comp.imm = static_cast<int32_t>(imm20 << 12);
            break;
        }
        case 0x6F: // UJ-type (JAL)
        {
            uint32_t bit20      = extractBits(inst, 31, 31);
            uint32_t bits19_12  = extractBits(inst, 19, 12);
            uint32_t bit11      = extractBits(inst, 20, 20);
            uint32_t bits10_1   = extractBits(inst, 30, 21);
            uint32_t fullImm    = (bit20 << 20) | (bits19_12 << 12)
                                 | (bit11 << 11) | (bits10_1 << 1);
            comp.imm = extendSign(fullImm, 21);
            break;
        }
        default:
            comp.imm = 0;
            break;
    }
    return comp;
}

//=====================================================================
// Function: isTerminatorInstruction
// Purpose: Check if the provided instruction signals termination (all zeros).
//=====================================================================
bool isTerminatorInstruction(uint32_t inst) {
    return (inst == 0x00000000);
}

//=====================================================================
// Function: loadMemoryConfiguration
// Purpose: Parse a memory configuration file and distribute the words into
//          the appropriate segments (instruction, data, or stack) based on address.
//=====================================================================
bool loadMemoryConfiguration(const std::string &filename) {
    std::ifstream fileIn(filename);
    if (!fileIn.is_open()) {
        std::cerr << "ERROR: Failed to open file " << filename << "\n";
        return false;
    }

    std::string line;
    while (std::getline(fileIn, line)) {
        // Remove any comment starting with '#'
        size_t commentIndex = line.find('#');
        if (commentIndex != std::string::npos) {
            line = line.substr(0, commentIndex);
        }
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string addrStr, wordStr;
        iss >> addrStr >> wordStr;
        if (addrStr.empty() || wordStr.empty()) continue;
        if (wordStr[0] == '<' || wordStr[0] == 't') continue;

        // Eliminate any trailing comma from the data field.
        size_t commaIndex = wordStr.find(',');
        if (commaIndex != std::string::npos) {
            wordStr = wordStr.substr(0, commaIndex);
        }

        try {
            uint32_t addr = std::stoul(addrStr, nullptr, 16);
            uint32_t wordVal = std::stoul(wordStr, nullptr, 16);

            if (addr < 0x10000000) {
                // Instruction region: store into instruction memory.
                instrMemory[addr] = wordVal;
            } else if (addr < 0x7FFFFFFF) {
                // Data region.
                dataMemory.writeWord(addr, static_cast<int32_t>(wordVal));
            } else {
                // Stack region.
                stackMemory.writeWord(addr, static_cast<int32_t>(wordVal));
            }
        } catch (const std::exception &ex) {
            std::cerr << "Error processing line: " << line << "\n";
            continue;
        }
    }
    fileIn.close();
    return true;
}

//=====================================================================
// Function: displayRegisterState
// Purpose: Print the current state of the CPU registers and key control values.
//=====================================================================
void displayRegisterState() {
    std::cout << "CPU Register State:\n";
    for (int i = 0; i < TOTAL_REGISTERS; i++) {
        std::cout << "R[" << std::setw(2) << i << "] = " << cpuRegisters[i] << "   ";
        if ((i + 1) % 4 == 0) {
            std::cout << "\n";
        }
    }
    std::cout << "-------------------------------------\n";
    std::cout << "PC = 0x" << std::hex << pc << "  IR = 0x" << ir << std::dec << "\n";
    std::cout << "operand1 = " << operand1 << "  operand2 = " << operand2
              << "  storeValue = " << storeValue << "\n";
    std::cout << "aluOutput = " << aluOutput << "  writeBackData = " << writeBackData
              << "  memDataReg = " << memDataReg << "\n";
    std::cout << "===========================================\n";
}

//=====================================================================
// Function: selectMemorySegment
// Purpose: Given an address, determine the correct memory segment for accesses.
// Returns a pointer to the appropriate MemorySegment, or nullptr if the address
// falls within the instruction area (which is not allowed).
//=====================================================================
MemorySegment* selectMemorySegment(uint32_t addr) {
    if (addr < 0x10000000) {
        // Loads/stores to the instruction area are prohibited.
        return nullptr;
    } else if (addr < 0x7FFFFFFF) {
        return &dataMemory;
    } else {
        return &stackMemory;
    }
}
int main(int argc, char* argv[]) {
    // Ensure an input file is provided.
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <memory_config.mc>\n";
        return 1;
    }

    std::string configFilename = argv[1];
    if (!loadMemoryConfiguration(configFilename)) {
        return 1;
    }

    // Dump initial memory contents into their respective files.
    dumpInstrMemoryToFile("instruction.mc");
    dumpMemorySegmentToFile("data.mc", dataMemory, 0x10000000, 0x7FFFFFFF);
    dumpMemorySegmentToFile("stack.mc", stackMemory, 0x7FFFFFFF, 0xFFFFFFFF);

    // Initialize all CPU registers to zero.
    for (int i = 0; i < TOTAL_REGISTERS; i++) {
        cpuRegisters[i] = 0;
    }
    // Set the program counter and initialize the stack pointer (register x2).
    pc = 0;
    cpuRegisters[2] = 0x7FFFFFFF;

    std::cout << "Initial CPU State (pre-cycle):\n";
    displayRegisterState();

    char command;
    std::cout << "Enter N for next cycle, R to run continuously, or E to exit: ";
    std::cin >> command;
    if (command == 'E' || command == 'e') {
        std::cout << "Simulation terminated on user request.\n";
        return 0;
    }
    bool continuousRun = (command == 'R' || command == 'r');

    std::cout << "Starting simulation...\n";
    while (true) {
        std::cout << "Cycle: " << cycleCount << "\n";
        // Fetch the instruction from instruction memory based on the current pc.
        auto fetchIt = instrMemory.find(pc);
        if (fetchIt == instrMemory.end()) {
            std::cout << "[Fetch] No instruction at PC=0x" << std::hex << pc 
                      << ". Halting simulation.\n";
            break;
        }
        ir = fetchIt->second;
        std::cout << "[Fetch] PC=0x" << std::hex << pc 
                  << " IR=0x" << ir << std::dec << "\n";

        // Check if the fetched instruction indicates termination.
        if (isTerminatorInstruction(ir)) {
            std::cout << "[Fetch] Termination instruction encountered (0x00000000).\n";
            break;
        }

        // Decode the fetched instruction.
        InstructionComponents inst = decodeInstruction(ir);
        std::cout << "[Decode] opCode=0x" << std::hex << inst.opCode
                  << " rd=" << inst.rd << " rs1=" << inst.rs1
                  << " rs2=" << inst.rs2 << " func3=0x" << inst.func3
                  << " func7=0x" << inst.func7 << " imm=" << std::dec << inst.imm << "\n";

        // Prepare the operands for execution.
        operand1 = cpuRegisters[inst.rs1];
        if (inst.opCode == 0x13 || inst.opCode == 0x03 ||
            inst.opCode == 0x67 || inst.opCode == 0x37 ||
            inst.opCode == 0x23) {
            operand2 = inst.imm;
        } else if (inst.opCode == 0x17) {
            operand1 = pc;
            operand2 = inst.imm;
        } else {
            operand2 = cpuRegisters[inst.rs2];
        }
        storeValue = cpuRegisters[inst.rs2];

        uint32_t updatedPC = pc + 4;
        aluOutput = 0;
        writeBackData = 0;

        // Execution Stage: Process the instruction based on its opcode.
        switch (inst.opCode) {
            // --- R-type Instructions (opcode 0x33)
            case 0x33:
                switch (inst.func3) {
                    case 0x0:
                        if (inst.func7 == 0x00) {
                            aluOutput = operand1 + operand2;
                            std::cout << "[Execute] ADD: " << operand1 << " + " << operand2 
                                      << " = " << aluOutput << "\n";
                        } else if (inst.func7 == 0x20) {
                            aluOutput = operand1 - operand2;
                            std::cout << "[Execute] SUB: " << operand1 << " - " << operand2 
                                      << " = " << aluOutput << "\n";
                        } else if (inst.func7 == 0x01) {
                            aluOutput = operand1 * operand2;
                            std::cout << "[Execute] MUL: " << operand1 << " * " << operand2 
                                      << " = " << aluOutput << "\n";
                        }
                        break;
                    case 0x4:
                        if (inst.func7 == 0x00) {
                            aluOutput = operand1 ^ operand2;
                            std::cout << "[Execute] XOR: " << operand1 << " ^ " << operand2 
                                      << " = " << aluOutput << "\n";
                        } else if (inst.func7 == 0x01) {
                            if (operand2 == 0) {
                                aluOutput = 0;
                                std::cout << "[Execute] DIV: Division by zero detected!\n";
                            } else {
                                aluOutput = operand1 / operand2;
                                std::cout << "[Execute] DIV: " << operand1 << " / " << operand2 
                                          << " = " << aluOutput << "\n";
                            }
                        }
                        break;
                    case 0x6:
                        if (inst.func7 == 0x00) {
                            aluOutput = operand1 | operand2;
                            std::cout << "[Execute] OR: " << operand1 << " | " << operand2 
                                      << " = " << aluOutput << "\n";
                        } else if (inst.func7 == 0x01) {
                            if (operand2 == 0) {
                                aluOutput = 0;
                                std::cout << "[Execute] REM: Division by zero for remainder!\n";
                            } else {
                                aluOutput = operand1 % operand2;
                                std::cout << "[Execute] REM: " << operand1 << " % " << operand2 
                                          << " = " << aluOutput << "\n";
                            }
                        }
                        break;
                    case 0x7:
                        aluOutput = operand1 & operand2;
                        std::cout << "[Execute] AND: " << operand1 << " & " << operand2 
                                  << " = " << aluOutput << "\n";
                        break;
                    case 0x1: {
                        int shiftAmt = operand2 & 0x1F;
                        aluOutput = operand1 << shiftAmt;
                        std::cout << "[Execute] SLL: " << operand1 << " << " << shiftAmt 
                                  << " = " << aluOutput << "\n";
                        break;
                    }
                    case 0x2:
                        aluOutput = (operand1 < operand2) ? 1 : 0;
                        std::cout << "[Execute] SLT: " << aluOutput << "\n";
                        break;
                    case 0x5: {
                        int shiftAmt = operand2 & 0x1F;
                        if (inst.func7 == 0x00) {
                            aluOutput = static_cast<int32_t>(static_cast<uint32_t>(operand1) >> shiftAmt);
                            std::cout << "[Execute] SRL: " << aluOutput << "\n";
                        } else if (inst.func7 == 0x20) {
                            aluOutput = operand1 >> shiftAmt;
                            std::cout << "[Execute] SRA: " << aluOutput << "\n";
                        }
                        break;
                    }
                    default:
                        std::cout << "[Execute] R-type: Unsupported func3 value.\n";
                        break;
                }
                writeBackData = aluOutput;
                break;

            // --- I-type ALU Operations (opcode 0x13)
            case 0x13:
                switch (inst.func3) {
                    case 0x0:
                        aluOutput = operand1 + operand2;
                        std::cout << "[Execute] ADDI: " << aluOutput << "\n";
                        break;
                    case 0x7:
                        aluOutput = operand1 & operand2;
                        std::cout << "[Execute] ANDI: " << aluOutput << "\n";
                        break;
                    case 0x6:
                        aluOutput = operand1 | operand2;
                        std::cout << "[Execute] ORI: " << aluOutput << "\n";
                        break;
                    case 0x4:
                        aluOutput = operand1 ^ operand2;
                        std::cout << "[Execute] XORI: " << aluOutput << "\n";
                        break;
                    case 0x2:
                        aluOutput = (operand1 < operand2) ? 1 : 0;
                        std::cout << "[Execute] SLTI: " << aluOutput << "\n";
                        break;
                    case 0x1: {
                        int shiftAmt = operand2 & 0x1F;
                        aluOutput = operand1 << shiftAmt;
                        std::cout << "[Execute] SLLI: " << aluOutput << "\n";
                        break;
                    }
                    case 0x5: {
                        int shiftAmt = operand2 & 0x1F;
                        int highBits = (operand2 >> 5) & 0x7F;
                        if (highBits == 0x00) {
                            aluOutput = static_cast<int32_t>(static_cast<uint32_t>(operand1) >> shiftAmt);
                            std::cout << "[Execute] SRLI: " << aluOutput << "\n";
                        } else if (highBits == 0x20) {
                            aluOutput = operand1 >> shiftAmt;
                            std::cout << "[Execute] SRAI: " << aluOutput << "\n";
                        }
                        break;
                    }
                    default:
                        std::cout << "[Execute] I-type: Unsupported func3 value.\n";
                        break;
                }
                writeBackData = aluOutput;
                break;

            // --- LOAD Instructions (opcode 0x03)
            case 0x03: {
                uint32_t effectiveAddr = operand1 + inst.imm;
                MemorySegment* memSeg = selectMemorySegment(effectiveAddr);
                if (!memSeg) {
                    std::cout << "[Execute] LOAD: Invalid memory access at 0x" 
                              << std::hex << effectiveAddr << std::dec << "\n";
                    break;
                }
                aluOutput = effectiveAddr;
                switch (inst.func3) {
                    case 0x0: { // LB
                        int8_t byteVal = memSeg->readByte(effectiveAddr);
                        memDataReg = byteVal;
                        writeBackData = memDataReg;
                        std::cout << "[Execute] LB: Loaded " << static_cast<int>(byteVal) << "\n";
                        break;
                    }
                    case 0x1: { // LH
                        int16_t halfVal = 0;
                        halfVal |= (memSeg->readByte(effectiveAddr) & 0xFF);
                        halfVal |= (memSeg->readByte(effectiveAddr + 1) & 0xFF) << 8;
                        memDataReg = halfVal;
                        writeBackData = memDataReg;
                        std::cout << "[Execute] LH: Loaded " << halfVal << "\n";
                        break;
                    }
                    case 0x2: { // LW
                        int32_t wordVal = memSeg->readWord(effectiveAddr);
                        memDataReg = wordVal;
                        writeBackData = memDataReg;
                        std::cout << "[Execute] LW: Loaded " << wordVal << "\n";
                        break;
                    }
                    default:
                        std::cout << "[Execute] LOAD: Unsupported func3 value.\n";
                        break;
                }
                break;
            }

            // --- STORE Instructions (opcode 0x23)
            case 0x23: {
                uint32_t effectiveAddr = operand1 + inst.imm;
                MemorySegment* memSeg = selectMemorySegment(effectiveAddr);
                if (!memSeg) {
                    std::cout << "[Execute] STORE: Invalid memory access at 0x" 
                              << std::hex << effectiveAddr << std::dec << "\n";
                    break;
                }
                aluOutput = effectiveAddr;
                switch (inst.func3) {
                    case 0x0: { // SB
                        memSeg->writeByte(effectiveAddr, static_cast<int8_t>(storeValue & 0xFF));
                        std::cout << "[Execute] SB: Stored byte " << (storeValue & 0xFF) << "\n";
                        break;
                    }
                    case 0x1: { // SH
                        int16_t halfToStore = static_cast<int16_t>(storeValue & 0xFFFF);
                        memSeg->writeByte(effectiveAddr, static_cast<uint8_t>(halfToStore & 0xFF));
                        memSeg->writeByte(effectiveAddr + 1, static_cast<uint8_t>((halfToStore >> 8) & 0xFF));
                        std::cout << "[Execute] SH: Stored half-word " << halfToStore << "\n";
                        break;
                    }
                    case 0x2: { // SW
                        memSeg->writeWord(effectiveAddr, storeValue);
                        std::cout << "[Execute] SW: Stored word " << storeValue << "\n";
                        break;
                    }
                    default:
                        std::cout << "[Execute] STORE: Unsupported func3 value.\n";
                        break;
                }
                break;
            }

            // --- Branch Instructions (opcode 0x63)
            case 0x63:
                switch (inst.func3) {
                    case 0x0: // BEQ
                        if (operand1 == storeValue) {
                            updatedPC = pc + inst.imm;
                            std::cout << "[Execute] BEQ: Branch taken.\n";
                        } else {
                            std::cout << "[Execute] BEQ: Branch not taken.\n";
                        }
                        break;
                    case 0x1: // BNE
                        if (operand1 != storeValue) {
                            updatedPC = pc + inst.imm;
                            std::cout << "[Execute] BNE: Branch taken.\n";
                        } else {
                            std::cout << "[Execute] BNE: Branch not taken.\n";
                        }
                        break;
                    case 0x4: // BLT
                        if (operand1 < storeValue) {
                            updatedPC = pc + inst.imm;
                            std::cout << "[Execute] BLT: Branch taken.\n";
                        } else {
                            std::cout << "[Execute] BLT: Branch not taken.\n";
                        }
                        break;
                    case 0x5: // BGE
                        if (operand1 >= storeValue) {
                            updatedPC = pc + inst.imm;
                            std::cout << "[Execute] BGE: Branch taken.\n";
                        } else {
                            std::cout << "[Execute] BGE: Branch not taken.\n";
                        }
                        break;
                    default:
                        std::cout << "[Execute] Branch: Unsupported func3 value.\n";
                        break;
                }
                break;

            // --- Jump and Link Instructions
            case 0x6F: // JAL
                aluOutput = pc + 4;
                updatedPC = pc + inst.imm;
                writeBackData = aluOutput;
                std::cout << "[Execute] JAL: Jumping to 0x" << std::hex << updatedPC 
                          << std::dec << "\n";
                break;
            case 0x67: // JALR
                aluOutput = pc + 4;
                {
                    uint32_t jumpTarget = (operand1 + inst.imm) & ~1U;
                    updatedPC = jumpTarget;
                    writeBackData = aluOutput;
                    std::cout << "[Execute] JALR: Jumping to 0x" << std::hex << updatedPC 
                              << std::dec << "\n";
                }
                break;
            // --- Upper Immediate Instructions
            case 0x37: // LUI
                aluOutput = inst.imm;
                writeBackData = aluOutput;
                std::cout << "[Execute] LUI: Loaded upper immediate " << aluOutput << "\n";
                break;
            case 0x17: // AUIPC
                aluOutput = pc + inst.imm;
                writeBackData = aluOutput;
                std::cout << "[Execute] AUIPC: Computed " << aluOutput << "\n";
                break;
            default:
                std::cout << "[Execute] Unhandled opcode: 0x" << std::hex 
                          << inst.opCode << std::dec << "\n";
                break;
        }

        // Write-back Stage: Update destination register if needed.
        switch (inst.opCode) {
            case 0x33: // R-type
            case 0x13: // I-type ALU
            case 0x17: // AUIPC
            case 0x37: // LUI
            case 0x03: // LOAD
            case 0x6F: // JAL
            case 0x67: // JALR
                if (inst.rd != 0) {
                    cpuRegisters[inst.rd] = writeBackData;
                    std::cout << "[WB] Register R[" << inst.rd << "] updated to " 
                              << cpuRegisters[inst.rd] << "\n";
                }
                break;
            default:
                // No write-back for branch or store instructions.
                break;
        }

        // Ensure register zero remains 0.
        cpuRegisters[0] = 0;
        // Update the program counter for the next cycle.
        pc = updatedPC;

        // Display the updated CPU register state.
        displayRegisterState();

        // Update memory dump files after each cycle.
        dumpInstrMemoryToFile("instruction.mc");
        dumpMemorySegmentToFile("data.mc", dataMemory, 0x10000000, 0x7FFFFFFF);
        dumpMemorySegmentToFile("stack.mc", stackMemory, 0x7FFFFFFF, 0xFFFFFFFF);

        cycleCount++;

        // If not in continuous run mode, prompt the user for further action.
        if (!continuousRun) {
            std::cout << "Enter N for next cycle, R to run continuously, or E to exit: ";
            std::cin >> command;
            if (command == 'E' || command == 'e') {
                std::cout << "Simulation terminated by user.\n";
                break;
            } else if (command == 'R' || command == 'r') {
                continuousRun = true;
            }
        }
    }

    std::cout << "Simulation completed after " << cycleCount << " cycles.\n";
     // Print Data Memory Contents
    cout << "\nData Memory Contents (0x10000000 - 0x100FFFFF):" << endl;
    cout << "Address\t\tValue" << endl;
    cout << "----------------------" << endl;
    for (const auto &entry : dataMemory) {
        cout << "0x" << hex << setw(8) << setfill('0') << entry.first << "\t0x" 
             << setw(8) << setfill('0') << entry.second << endl;
    }
    
    // Simulate Stack Memory (from 0x7FFFFFFC downwards)
    const uint32_t stackBase = 0x7FFFFFFC;
    const uint32_t stackSize = 0x100; // 256 bytes for demonstration
    unordered_map<uint32_t, uint32_t> stackMemory;
    
    // Initialize stack with some dummy values for demonstration
    for (uint32_t addr = stackBase; addr > stackBase - stackSize; addr -= 4) {
        stackMemory[addr] = 0xDEADBEEF; // Example value
    }
    
    // Print Stack Memory Contents
    cout << "\nStack Memory Contents (0x7FFFFFFC downwards):" << endl;
    cout << "Address\t\tValue" << endl;
    cout << "----------------------" << endl;
    for (uint32_t addr = stackBase; addr > stackBase - stackSize; addr -= 4) {
        cout << "0x" << hex << setw(8) << setfill('0') << addr << "\t0x" 
             << setw(8) << setfill('0') << stackMemory[addr] << endl;
    }
    
    cout << "\nAssembly to machine-code conversion complete. See output.mc" << endl;
    return 0;
}
    return 0;
}
