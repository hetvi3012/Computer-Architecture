#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <cstdint>
#include <iomanip>

// ----------------------------------------------------------------------
// Global CPU State Definitions
// ----------------------------------------------------------------------
static const int NUM_REGS = 32;
int32_t registers[NUM_REGS];         // Register file (32 registers)
uint32_t programCounter = 0;         // Program Counter (PC)
uint32_t instructionRegister = 0;    // Instruction Register (IR)

// Operand and ALU registers:
int32_t operandA = 0;         // First operand (from source register or PC)
int32_t operandB = 0;         // Second operand (immediate or register value)
int32_t operandM = 0;         // Operand for store operations (rs2 value)
int32_t aluResult = 0;        // ALU computation result
int32_t writeBackResult = 0;  // Result to write back to the register file
int32_t memoryDataRegister = 0; // Data loaded from memory

uint64_t cycleCount = 0;      // Simulation clock cycle counter

// Instruction memory: maps addresses to 32-bit instructions.
std::map<uint32_t, uint32_t> instructionMemory;

// ----------------------------------------------------------------------
// DataSegment Class: Dynamic Data Segment
// ----------------------------------------------------------------------
class DataSegment {
public:
    // Byte-addressable memory: each address maps to an 8-bit value.
    std::map<uint32_t, uint8_t> storage;

    // Load data from a file. Only addresses >= 0x10000000 are processed.
    void loadFromFile(const std::string &fileName) {
        std::ifstream input(fileName);
        if (!input.is_open()) {
            std::cerr << "ERROR: Unable to open file " << fileName << " for data segment loading.\n";
            return;
        }
        std::string line;
        while (std::getline(input, line)) {
            // Skip empty lines or comments.
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream iss(line);
            std::string addrStr, dataStr;
            if (!(iss >> addrStr >> dataStr))
                continue;
            // Remove trailing comma from data string if present.
            if (!dataStr.empty() && dataStr.back() == ',')
                dataStr.pop_back();
            try {
                uint32_t address = std::stoul(addrStr, nullptr, 16);
                if (address >= 0x10000000) {
                    uint32_t value = std::stoul(dataStr, nullptr, 16);
                    // Split the 32-bit word into 4 bytes.
                    for (int i = 0; i < 4; ++i) {
                        storage[address + i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
                    }
                }
            } catch (...) {
                continue;
            }
        }
        input.close();
    }

    // Write a single byte at the specified address.
    void writeByte(uint32_t address, uint8_t value) {
        storage[address] = value;
    }

    // Write a 32-bit word by breaking it into four bytes.
    void writeWord(uint32_t address, int32_t value) {
        for (int i = 0; i < 4; ++i) {
            storage[address + i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
        }
    }

    // Read a 32-bit word by reassembling 4 consecutive bytes.
    int32_t readWord(uint32_t address) {
        int32_t result = 0;
        for (int i = 0; i < 4; ++i) {
            uint8_t byte = 0;
            if (storage.find(address + i) != storage.end())
                byte = storage[address + i];
            result |= (byte << (8 * i));
        }
        return result;
    }

    // Read a single byte.
    int8_t readByte(uint32_t address) {
        if (storage.find(address) != storage.end())
            return static_cast<int8_t>(storage[address]);
        return 0;
    }

    // Update the original input file to reflect the current data segment state.
    void updateInputFile(const std::string &fileName) {
        std::ifstream input(fileName);
        if (!input.is_open()){
            std::cerr << "ERROR: Unable to open file " << fileName << " for updating.\n";
            return;
        }
        std::vector<std::string> lines;
        std::string line;
        while (std::getline(input, line))
            lines.push_back(line);
        input.close();

        // Process each line and update if the address belongs to the data segment.
        for (auto &line : lines) {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream iss(line);
            std::string addrStr, dataStr;
            if (!(iss >> addrStr >> dataStr))
                continue;
            try {
                uint32_t addr = std::stoul(addrStr, nullptr, 16);
                if (addr >= 0x10000000) {
                    int32_t word = readWord(addr);
                    std::ostringstream oss;
                    oss << "0x" << std::hex << std::setw(8) << std::setfill('0') << word;
                    std::string remaining;
                    std::getline(iss, remaining);
                    line = addrStr + " " + oss.str() + remaining;
                }
            } catch (...) {
                continue;
            }
        }
        std::ofstream output(fileName);
        if (!output.is_open()){
            std::cerr << "ERROR: Unable to open file " << fileName << " for writing.\n";
            return;
        }
        for (const auto &line : lines)
            output << line << "\n";
        output.close();
        std::cout << "Data segment in the input file updated successfully.\n";
    }
};

DataSegment dataSegment;

// ----------------------------------------------------------------------
// Bit Extraction and Sign Extension Utilities
// ----------------------------------------------------------------------

// Extracts bits from 'low' to 'high' (inclusive) from a given value.
static inline uint32_t extractBits(uint32_t value, int high, int low) {
    uint32_t mask = (1u << (high - low + 1)) - 1;
    return (value >> low) & mask;
}

// Sign-extends a value with 'bitCount' bits to a full 32-bit integer.
static inline int32_t extendSign(uint32_t value, int bitCount) {
    int shift = 32 - bitCount;
    return static_cast<int32_t>((static_cast<int32_t>(value << shift)) >> shift);
}

// ----------------------------------------------------------------------
// Instruction Decoding Structures and Functions
// ----------------------------------------------------------------------

// Structure representing a decoded instruction.
struct DecodedInstruction {
    uint32_t opcode;    // Operation code.
    uint32_t rd;        // Destination register.
    uint32_t rs1;       // First source register.
    uint32_t rs2;       // Second source register.
    uint32_t funct3;    // 3-bit function field.
    uint32_t funct7;    // 7-bit function field.
    int32_t immediate;  // Sign-extended immediate.
};

// Decodes a 32-bit instruction into its components.
DecodedInstruction decodeInstruction(uint32_t instruction) {
    DecodedInstruction dec = {};
    dec.opcode = extractBits(instruction, 6, 0);
    dec.rd     = extractBits(instruction, 11, 7);
    dec.funct3 = extractBits(instruction, 14, 12);
    dec.rs1    = extractBits(instruction, 19, 15);
    
    // For I-type instructions (ALU, LOAD, JALR), rs2 and funct7 are not used.
    if (dec.opcode == 0x13 || dec.opcode == 0x03 || dec.opcode == 0x67) {
        dec.rs2 = 0;
        dec.funct7 = 0;
    } else {
        dec.rs2 = extractBits(instruction, 24, 20);
        dec.funct7 = extractBits(instruction, 31, 25);
    }
    
    // Determine the immediate based on opcode type.
    switch(dec.opcode) {
        // I-type (ALU, LOAD, JALR)
        case 0x13:
        case 0x03:
        case 0x67: {
            uint32_t imm12 = extractBits(instruction, 31, 20);
            dec.immediate = extendSign(imm12, 12);
            break;
        }
        // S-type (store)
        case 0x23: {
            uint32_t immHigh = extractBits(instruction, 31, 25);
            uint32_t immLow  = extractBits(instruction, 11, 7);
            uint32_t imm12   = (immHigh << 5) | immLow;
            dec.immediate = extendSign(imm12, 12);
            break;
        }
        // SB-type (branch)
        case 0x63: {
            uint32_t immBit12    = extractBits(instruction, 31, 31);
            uint32_t immBit11    = extractBits(instruction, 7, 7);
            uint32_t immBits10_5 = extractBits(instruction, 30, 25);
            uint32_t immBits4_1  = extractBits(instruction, 11, 8);
            uint32_t immCombined = (immBit12 << 12) | (immBit11 << 11) |
                                   (immBits10_5 << 5) | (immBits4_1 << 1);
            dec.immediate = extendSign(immCombined, 13);
            break;
        }
        // U-type (LUI, AUIPC)
        case 0x37:
        case 0x17: {
            uint32_t imm20 = extractBits(instruction, 31, 12);
            dec.immediate = imm20 << 12;
            break;
        }
        // UJ-type (JAL)
        case 0x6F: {
            uint32_t immBit20    = extractBits(instruction, 31, 31);
            uint32_t immBits19_12 = extractBits(instruction, 19, 12);
            uint32_t immBit11     = extractBits(instruction, 20, 20);
            uint32_t immBits10_1  = extractBits(instruction, 30, 21);
            uint32_t immCombined  = (immBit20 << 20) | (immBits19_12 << 12) |
                                    (immBit11 << 11) | (immBits10_1 << 1);
            dec.immediate = extendSign(immCombined, 21);
            break;
        }
        default:
            dec.immediate = 0;
            break;
    }
    return dec;
}

// ----------------------------------------------------------------------
// Check if an instruction is a termination instruction (all zeros).
// ----------------------------------------------------------------------
bool isTerminationInstruction(uint32_t instruction) {
    return (instruction == 0x00000000);
}

// ----------------------------------------------------------------------
// Load machine code from a file.
// This reads each line, skips comments/empty lines, and populates either the
// instruction memory (addresses below 0x10000000) or the data segment.
// ----------------------------------------------------------------------
bool loadMachineCodeFile(const std::string &fileName) {
    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "ERROR: Unable to open file " << fileName << "\n";
        return false;
    }
    std::string line;
    while (std::getline(file, line)) {
        size_t commentIndex = line.find('#');
        if (commentIndex != std::string::npos) {
            line = line.substr(0, commentIndex);
        }
        if (line.empty())
            continue;
        std::istringstream iss(line);
        std::string addressStr, dataStr;
        if (!(iss >> addressStr >> dataStr))
            continue;
        size_t commaPos = dataStr.find(',');
        if (commaPos != std::string::npos)
            dataStr = dataStr.substr(0, commaPos);
        try {
            uint32_t address = std::stoul(addressStr, nullptr, 16);
            uint32_t data = std::stoul(dataStr, nullptr, 16);
            if (address < 0x10000000) {
                instructionMemory[address] = data;
            } else {
                for (int i = 0; i < 4; ++i) {
                    dataSegment.storage[address + i] = static_cast<uint8_t>((data >> (8 * i)) & 0xFF);
                }
            }
        } catch (...) {
            std::cerr << "Parsing error on line: " << line << "\n";
            continue;
        }
    }
    file.close();
    return true;
}

// ----------------------------------------------------------------------
// Display the current state of the register file and key CPU registers.
// ----------------------------------------------------------------------
void displayRegisters() {
    std::cout << "Register File:\n";
    for (int i = 0; i < NUM_REGS; ++i) {
        std::cout << "R[" << std::setw(2) << i << "] = " << std::setw(10)
                  << registers[i] << "   ";
        if ((i + 1) % 4 == 0)
            std::cout << "\n";
    }
    std::cout << "-------------------------------------\n";
    std::cout << "PC = 0x" << std::hex << programCounter << std::dec
              << "  IR = 0x" << std::hex << instructionRegister << std::dec << "\n";
    std::cout << "operandA = " << operandA << "  operandB = " << operandB 
              << "  operandM = " << operandM << "\n";
    std::cout << "ALU Result = " << aluResult << "  WriteBackResult = " << writeBackResult 
              << "  Memory Data Register = " << memoryDataRegister << "\n";
    std::cout << "===========================================\n";
}

// ----------------------------------------------------------------------
// Main Simulation Loop
// ----------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::cout << "Starting RISC-V Simulator..." << std::endl;
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <machine_code_file>" << std::endl;
        return 1;
    }
    
    // Load machine code into instruction memory and data segment.
    if (!loadMachineCodeFile(argv[1])) {
        return 1;
    }
    dataSegment.loadFromFile(argv[1]);
    
    // Initialize registers and core state.
    for (int i = 0; i < NUM_REGS; ++i)
        registers[i] = 0;
    programCounter = 0;
    cycleCount = 0;
    
    std::cout << "Initial register state:" << std::endl;
    displayRegisters();
    
    // Prompt user for simulation mode.
    char userChoice;
    std::cout << "Enter N for next instruction, R for run-to-end, or E to exit: ";
    std::cin >> userChoice;
    if (userChoice == 'E' || userChoice == 'e') {
        std::cout << "Exiting simulation per user request." << std::endl;
        return 0;
    }
    bool runToEnd = (userChoice == 'R' || userChoice == 'r');
    
    std::cout << "Commencing simulation..." << std::endl;
    while (true) {
        std::cout << "Cycle: " << cycleCount << std::endl;
        
        // Fetch Stage: Retrieve instruction at current PC.
        if (instructionMemory.find(programCounter) == instructionMemory.end()) {
            std::cout << "[Fetch] No instruction at PC = 0x" << std::hex << programCounter
                      << ". Ending simulation." << std::endl;
            break;
        }
        instructionRegister = instructionMemory[programCounter];
        std::cout << "[Fetch] PC = 0x" << std::hex << programCounter
                  << ", IR = 0x" << instructionRegister << std::dec << std::endl;
        if (isTerminationInstruction(instructionRegister)) {
            std::cout << "[Fetch] Termination instruction encountered. Exiting simulation." << std::endl;
            break;
        }
        
        // Decode Stage.
        DecodedInstruction decoded = decodeInstruction(instructionRegister);
        std::cout << "[Decode] opcode=0x" << std::hex << decoded.opcode
                  << " rd=" << std::dec << decoded.rd
                  << " rs1=" << decoded.rs1
                  << " rs2=" << decoded.rs2
                  << " funct3=0x" << std::hex << decoded.funct3
                  << " funct7=0x" << decoded.funct7
                  << " immediate=" << std::dec << decoded.immediate << std::endl;
        
        // Operand Setup.
        operandA = registers[decoded.rs1];
        if (decoded.opcode == 0x13 || decoded.opcode == 0x03 ||
            decoded.opcode == 0x67 || decoded.opcode == 0x37 ||
            decoded.opcode == 0x23) {
            operandB = decoded.immediate;
        } else if (decoded.opcode == 0x17) { // AUIPC
            operandA = programCounter;
            operandB = decoded.immediate;
        } else {
            operandB = registers[decoded.rs2];
        }
        operandM = registers[decoded.rs2];  // For store operations.
        
        uint32_t nextPC = programCounter + 4;
        aluResult = 0;
        writeBackResult = 0;
        
        // Execute Stage.
        switch(decoded.opcode) {
            // R-type Instructions.
            case 0x33:
                switch(decoded.funct3) {
                    case 0x0:
                        if (decoded.funct7 == 0x00) {
                            aluResult = operandA + operandB;
                            std::cout << "[Execute] ADD: " << operandA << " + " << operandB
                                      << " = " << aluResult << std::endl;
                        } else if (decoded.funct7 == 0x20) {
                            aluResult = operandA - operandB;
                            std::cout << "[Execute] SUB: " << operandA << " - " << operandB
                                      << " = " << aluResult << std::endl;
                        } else if (decoded.funct7 == 0x01) {
                            aluResult = operandA * operandB;
                            std::cout << "[Execute] MUL: " << operandA << " * " << operandB
                                      << " = " << aluResult << std::endl;
                        }
                        break;
                    case 0x4:
                        if (decoded.funct7 == 0x00) {
                            aluResult = operandA ^ operandB;
                            std::cout << "[Execute] XOR: " << operandA << " ^ " << operandB
                                      << " = " << aluResult << std::endl;
                        } else if (decoded.funct7 == 0x01) {
                            if (operandB == 0) {
                                aluResult = 0;
                                std::cout << "[Execute] DIV: Division by zero!" << std::endl;
                            } else {
                                aluResult = operandA / operandB;
                                std::cout << "[Execute] DIV: " << operandA << " / " << operandB
                                          << " = " << aluResult << std::endl;
                            }
                        }
                        break;
                    case 0x6:
                        if (decoded.funct7 == 0x00) {
                            aluResult = operandA | operandB;
                            std::cout << "[Execute] OR: " << operandA << " | " << operandB
                                      << " = " << aluResult << std::endl;
                        } else if (decoded.funct7 == 0x01) {
                            if (operandB == 0) {
                                aluResult = 0;
                                std::cout << "[Execute] REM: Division by zero!" << std::endl;
                            } else {
                                aluResult = operandA % operandB;
                                std::cout << "[Execute] REM: " << operandA << " % " << operandB
                                          << " = " << aluResult << std::endl;
                            }
                        }
                        break;
                    case 0x7:
                        aluResult = operandA & operandB;
                        std::cout << "[Execute] AND: " << operandA << " & " << operandB
                                  << " = " << aluResult << std::endl;
                        break;
                    case 0x1: {
                        int shiftAmt = operandB & 0x1F;
                        aluResult = operandA << shiftAmt;
                        std::cout << "[Execute] SLL: " << operandA << " << " << shiftAmt
                                  << " = " << aluResult << std::endl;
                    } break;
                    case 0x2:
                        aluResult = (operandA < operandB) ? 1 : 0;
                        std::cout << "[Execute] SLT: (" << operandA << " < " << operandB
                                  << ") = " << aluResult << std::endl;
                        break;
                    case 0x5: {
                        int shiftAmt = operandB & 0x1F;
                        if (decoded.funct7 == 0x00) {
                            aluResult = static_cast<uint32_t>(operandA) >> shiftAmt;
                            std::cout << "[Execute] SRL: " << operandA << " >> " << shiftAmt
                                      << " = " << aluResult << std::endl;
                        } else if (decoded.funct7 == 0x20) {
                            aluResult = operandA >> shiftAmt;
                            std::cout << "[Execute] SRA: " << operandA << " >> " << shiftAmt
                                      << " = " << aluResult << std::endl;
                        }
                    } break;
                    default:
                        std::cout << "[Execute] Unimplemented R-type funct3" << std::endl;
                        break;
                }
                writeBackResult = aluResult;
                break;
            // I-type ALU Instructions.
            case 0x13:
                switch(decoded.funct3) {
                    case 0x0:
                        aluResult = operandA + operandB;
                        std::cout << "[Execute] ADDI: " << operandA << " + " << operandB
                                  << " = " << aluResult << std::endl;
                        break;
                    case 0x7:
                        aluResult = operandA & operandB;
                        std::cout << "[Execute] ANDI: " << operandA << " & " << operandB
                                  << " = " << aluResult << std::endl;
                        break;
                    case 0x6:
                        aluResult = operandA | operandB;
                        std::cout << "[Execute] ORI: " << operandA << " | " << operandB
                                  << " = " << aluResult << std::endl;
                        break;
                    case 0x4:
                        aluResult = operandA ^ operandB;
                        std::cout << "[Execute] XORI: " << operandA << " ^ " << operandB
                                  << " = " << aluResult << std::endl;
                        break;
                    case 0x2:
                        aluResult = (operandA < operandB) ? 1 : 0;
                        std::cout << "[Execute] SLTI: (" << operandA << " < " << operandB
                                  << ") = " << aluResult << std::endl;
                        break;
                    case 0x1: {
                        int shiftAmt = operandB & 0x1F;
                        aluResult = operandA << shiftAmt;
                        std::cout << "[Execute] SLLI: " << operandA << " << " << shiftAmt
                                  << " = " << aluResult << std::endl;
                    } break;
                    case 0x5: {
                        int shiftAmt = operandB & 0x1F;
                        int topExtension = (operandB >> 5) & 0x7F;
                        if (topExtension == 0x00) {
                            aluResult = static_cast<uint32_t>(operandA) >> shiftAmt;
                            std::cout << "[Execute] SRLI: " << operandA << " >> " << shiftAmt
                                      << " = " << aluResult << std::endl;
                        } else if (topExtension == 0x20) {
                            aluResult = operandA >> shiftAmt;
                            std::cout << "[Execute] SRAI: " << operandA << " >> " << shiftAmt
                                      << " = " << aluResult << std::endl;
                        } else {
                            std::cout << "[Execute] Unknown I-type shift extension." << std::endl;
                        }
                    } break;
                    default:
                        std::cout << "[Execute] Unimplemented I-type funct3" << std::endl;
                        break;
                }
                writeBackResult = aluResult;
                break;
            // I-type LOAD Instructions.
            case 0x03: {
                uint32_t effectiveAddr = operandA + decoded.immediate;
                aluResult = effectiveAddr; // Effective address.
                switch(decoded.funct3) {
                    case 0x0: {
                        int8_t loadedByte = dataSegment.readByte(effectiveAddr);
                        memoryDataRegister = loadedByte;
                        writeBackResult = memoryDataRegister;
                        std::cout << "[Execute] LB: loaded byte " << static_cast<int>(loadedByte)
                                  << " from 0x" << std::hex << effectiveAddr << std::dec << std::endl;
                    } break;
                    case 0x1: {
                        int16_t loadedHalf = 0;
                        for (int i = 0; i < 2; ++i) {
                            uint8_t byte = 0;
                            if (dataSegment.storage.find(effectiveAddr + i) != dataSegment.storage.end())
                                byte = dataSegment.storage[effectiveAddr + i];
                            loadedHalf |= (byte << (8 * i));
                        }
                        memoryDataRegister = loadedHalf;
                        writeBackResult = memoryDataRegister;
                        std::cout << "[Execute] LH: loaded halfword " << loadedHalf
                                  << " from 0x" << std::hex << effectiveAddr << std::dec << std::endl;
                    } break;
                    case 0x2: {
                        int32_t loadedWord = dataSegment.readWord(effectiveAddr);
                        memoryDataRegister = loadedWord;
                        writeBackResult = memoryDataRegister;
                        std::cout << "[Execute] LW: loaded word " << loadedWord
                                  << " from 0x" << std::hex << effectiveAddr << std::dec << std::endl;
                    } break;
                    default:
                        std::cout << "[Execute] Unimplemented LOAD funct3" << std::endl;
                        break;
                }
            }
            break;
            // S-type STORE Instructions.
            case 0x23: {
                uint32_t effectiveAddr = operandA + decoded.immediate;
                aluResult = effectiveAddr;
                switch(decoded.funct3) {
                    case 0x0: {
                        int8_t byteToStore = static_cast<int8_t>(operandM & 0xFF);
                        dataSegment.writeByte(effectiveAddr, byteToStore);
                        std::cout << "[Execute] SB: stored byte " << static_cast<int>(byteToStore)
                                  << " to 0x" << std::hex << effectiveAddr << std::dec << std::endl;
                    } break;
                    case 0x1: {
                        int16_t halfToStore = static_cast<int16_t>(operandM & 0xFFFF);
                        for (int i = 0; i < 2; ++i) {
                            dataSegment.writeByte(effectiveAddr + i, (halfToStore >> (8 * i)) & 0xFF);
                        }
                        std::cout << "[Execute] SH: stored halfword " << halfToStore
                                  << " to 0x" << std::hex << effectiveAddr << std::dec << std::endl;
                    } break;
                    case 0x2: {
                        dataSegment.writeWord(effectiveAddr, operandM);
                        std::cout << "[Execute] SW: stored word " << operandM
                                  << " to 0x" << std::hex << effectiveAddr << std::dec << std::endl;
                    } break;
                    default:
                        std::cout << "[Execute] Unimplemented STORE funct3" << std::endl;
                        break;
                }
            }
            break;
            // SB-type Branch Instructions.
            case 0x63: {
                switch(decoded.funct3) {
                    case 0x0:
                        if (operandA == operandM) {
                            nextPC = programCounter + decoded.immediate;
                            std::cout << "[Execute] BEQ taken: new PC = 0x" << std::hex << nextPC << std::dec << std::endl;
                        } else {
                            std::cout << "[Execute] BEQ not taken." << std::endl;
                        }
                        break;
                    case 0x1:
                        if (operandA != operandM) {
                            nextPC = programCounter + decoded.immediate;
                            std::cout << "[Execute] BNE taken: new PC = 0x" << std::hex << nextPC << std::dec << std::endl;
                        } else {
                            std::cout << "[Execute] BNE not taken." << std::endl;
                        }
                        break;
                    case 0x4:
                        if (operandA < operandM) {
                            nextPC = programCounter + decoded.immediate;
                            std::cout << "[Execute] BLT taken: new PC = 0x" << std::hex << nextPC << std::dec << std::endl;
                        } else {
                            std::cout << "[Execute] BLT not taken." << std::endl;
                        }
                        break;
                    case 0x5:
                        if (operandA >= operandM) {
                            nextPC = programCounter + decoded.immediate;
                            std::cout << "[Execute] BGE taken: new PC = 0x" << std::hex << nextPC << std::dec << std::endl;
                        } else {
                            std::cout << "[Execute] BGE not taken." << std::endl;
                        }
                        break;
                    default:
                        std::cout << "[Execute] Unimplemented branch funct3." << std::endl;
                        break;
                }
            }
            break;
            // UJ-type: JAL Instruction.
            case 0x6F: {
                aluResult = programCounter + 4; // Save return address.
                nextPC = programCounter + decoded.immediate;
                std::cout << "[Execute] JAL: Jump to 0x" << std::hex << nextPC
                          << " with return address 0x" << (programCounter + 4) << std::dec << std::endl;
                writeBackResult = aluResult;
            }
            break;
            // I-type: JALR Instruction.
            case 0x67: {
                aluResult = programCounter + 4; // Save return address.
                uint32_t target = static_cast<uint32_t>((operandA + decoded.immediate) & ~1);
                nextPC = target;
                std::cout << "[Execute] JALR: Jump to 0x" << std::hex << nextPC
                          << " with return address 0x" << (programCounter + 4) << std::dec << std::endl;
                writeBackResult = aluResult;
            }
            break;
            // U-type: LUI Instruction.
            case 0x37: {
                aluResult = decoded.immediate;
                std::cout << "[Execute] LUI: Result = 0x" << std::hex << aluResult << std::dec << std::endl;
                writeBackResult = aluResult;
            }
            break;
            // U-type: AUIPC Instruction.
            case 0x17: {
                aluResult = programCounter + decoded.immediate;
                std::cout << "[Execute] AUIPC: Result = 0x" << std::hex << aluResult << std::dec << std::endl;
                writeBackResult = aluResult;
            }
            break;
            default:
                std::cout << "[Execute] Unknown or unimplemented opcode: 0x" 
                          << std::hex << decoded.opcode << std::dec << std::endl;
                break;
        }
        
        // Write-Back Stage: Update destination register (if not register 0).
        switch(decoded.opcode) {
            case 0x33: // R-type
            case 0x13: // I-type ALU
            case 0x17: // AUIPC
            case 0x37: // LUI
            case 0x03: // LOAD
            case 0x6F: // JAL
            case 0x67: // JALR
                if (decoded.rd != 0) {
                    registers[decoded.rd] = writeBackResult;
                    std::cout << "[WB] Updated R[" << decoded.rd << "] = " 
                              << registers[decoded.rd] << std::endl;
                }
                break;
            default:
                break;
        }
        
        // Ensure register 0 remains 0.
        registers[0] = 0;
        programCounter = nextPC;
        displayRegisters();
        cycleCount++;
        
        // If not running continuously, prompt the user.
        if (!runToEnd) {
            std::cout << "Enter N for next instruction, R for run-to-end, or E to exit: ";
            std::cin >> userChoice;
            if (userChoice == 'E' || userChoice == 'e') {
                std::cout << "Exiting simulation per user request." << std::endl;
                break;
            } else if (userChoice == 'R' || userChoice == 'r') {
                runToEnd = true;
            }
        }
    }
    
    std::cout << "Simulation complete after " << cycleCount << " cycles." << std::endl;
    dataSegment.updateInputFile(argv[1]);
    return 0;
}
