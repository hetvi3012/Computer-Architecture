#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <iomanip>
#include <bitset>
#include <algorithm>
#include <cctype>
#include <cstdint>    // For fixed-width integer types like uint32_t
#include <stdexcept>

using namespace std;

//--------------------------------------------------
// Global Label Table: maps label name -> address
unordered_map<string, uint32_t> labelTable;

//--------------------------------------------------
// Helper Functions

// Remove inline comments: anything following a '#' character is discarded.
string removeComments(const string &s) {
    size_t pos = s.find('#');
    if (pos != string::npos)
        return s.substr(0, pos);
    return s;
}

// Remove leading/trailing whitespace (and also remove carriage returns/newlines).
string trim(const string &s) {
    string result = s;
    // Remove carriage return and newline characters.
    result.erase(remove(result.begin(), result.end(), '\r'), result.end());
    result.erase(remove(result.begin(), result.end(), '\n'), result.end());
    // Remove inline comments (if any)
    result = removeComments(result);
    size_t start = result.find_first_not_of(" \t");
    if (start == string::npos)
        return "";
    size_t end = result.find_last_not_of(" \t");
    return result.substr(start, end - start + 1);
}

// Split a string by a given delimiter.
vector<string> split(const string &s, char delimiter) {
    vector<string> tokens;
    string token;
    istringstream tokenStream(s);
    while(getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Check if a string represents a (possibly negative) number.
bool isNumber(const string &s) {
    string str = trim(s);
    if (str.empty()) return false;
    size_t i = 0;
    if (str[0] == '-' || str[0] == '+') i = 1;
    for (; i < str.size(); ++i) {
        if (!isdigit(str[i]))
            return false;
    }
    return true;
}

// Parse a register string (expects format "x5" or "X5").
int parseRegister(const string &reg) {
    string r = trim(reg);
    if (!r.empty() && (r[0] == 'x' || r[0] == 'X')) {
        return stoi(r.substr(1));
    }
    return stoi(r);
}

//--------------------------------------------------
// Resolve an immediate operand string.
// For branch (SB) or jump (UJ) instructions, compute PC-relative offset.
// For other instructions, if the immediate is non-numeric, treat it as a label.
int resolveImmediateValue(const string &immStr, uint32_t currentAddress, const string &instrType) {
    string s = trim(immStr);
    if (isNumber(s))
        return stoi(s);
    if (labelTable.find(s) == labelTable.end()) {
        cerr << "Error: Undefined label '" << s << "'" << endl;
        return 0;
    }
    int labelAddr = static_cast<int>(labelTable[s]);
    if (instrType == "SB" || instrType == "UJ")
        return labelAddr - currentAddress;
    return labelAddr;
}

// In load/store instructions the offset is given as "offset(rs)".
// This function supports only numeric immediates.
void parseOffsetOperand(const string &operand, int &rs, int &imm) {
    size_t pos1 = operand.find('(');
    size_t pos2 = operand.find(')');
    if (pos1 != string::npos && pos2 != string::npos) {
        string immStr = operand.substr(0, pos1);
        string regStr = operand.substr(pos1 + 1, pos2 - pos1 - 1);
        imm = stoi(trim(immStr));
        rs = parseRegister(trim(regStr));
    } else {
        imm = 0;
        rs = 0;
    }
}

// Convert an integer value to its binary representation (fixed bit-width).
string intToBinary(int value, int bits) {
    bitset<32> bs(value);
    string s = bs.to_string();
    return s.substr(32 - bits, bits);
}

//--------------------------------------------------
// Instruction Encoding Functions

uint32_t encodeR(int funct7, int rs2, int rs1, int funct3, int rd, int opcode) {
    return ((funct7 & 0x7F) << 25) |
           ((rs2   & 0x1F) << 20) |
           ((rs1   & 0x1F) << 15) |
           ((funct3 & 0x7)  << 12) |
           ((rd    & 0x1F) << 7)  |
           (opcode  & 0x7F);
}

uint32_t encodeI(int imm, int rs1, int funct3, int rd, int opcode) {
    return ((imm   & 0xFFF) << 20) |
           ((rs1   & 0x1F)  << 15) |
           ((funct3 & 0x7)   << 12) |
           ((rd    & 0x1F)  << 7)  |
           (opcode  & 0x7F);
}

uint32_t encodeS(int imm, int rs2, int rs1, int funct3, int opcode) {
    int imm_high = (imm >> 5) & 0x7F;
    int imm_low  = imm & 0x1F;
    return (imm_high << 25) |
           ((rs2   & 0x1F) << 20) |
           ((rs1   & 0x1F) << 15) |
           ((funct3 & 0x7)  << 12) |
           (imm_low  << 7)  |
           (opcode  & 0x7F);
}

uint32_t encodeSB(int imm, int rs2, int rs1, int funct3, int opcode) {
    int bit12    = (imm >> 12) & 0x1;
    int bit11    = (imm >> 11) & 0x1;
    int bits10_5 = (imm >> 5)  & 0x3F;
    int bits4_1  = (imm >> 1)  & 0xF;
    return (bit12    << 31) |
           (bits10_5 << 25) |
           ((rs2    & 0x1F) << 20) |
           ((rs1    & 0x1F) << 15) |
           ((funct3 & 0x7)  << 12) |
           (bits4_1  << 8)  |
           (bit11    << 7)  |
           (opcode   & 0x7F);
}

uint32_t encodeU(int imm, int rd, int opcode) {
    return (imm << 12) |
           ((rd & 0x1F) << 7) |
           (opcode & 0x7F);
}

uint32_t encodeUJ(int imm, int rd, int opcode) {
    int bit20     = (imm >> 20) & 0x1;
    int bits10_1  = (imm >> 1)  & 0x3FF;
    int bit11     = (imm >> 11) & 0x1;
    int bits19_12 = (imm >> 12) & 0xFF;
    return (bit20     << 31) |
           (bits19_12 << 12) |
           (bit11     << 20) |
           (bits10_1  << 21) |
           ((rd       & 0x1F) << 7) |
           (opcode    & 0x7F);
}

//--------------------------------------------------
// Instruction Set Definition
// Each supported instruction is defined with its type and encoding fields.
struct InstructionInfo {
    string type; // "R", "I", "S", "SB", "U", or "UJ"
    int opcode;
    int funct3;
    int funct7; // Only used for R-type; -1 otherwise.
};

unordered_map<string, InstructionInfo> instructionSet = {
    // R-type instructions
    {"add", {"R", 0x33, 0x0, 0x00}},
    {"sub", {"R", 0x33, 0x0, 0x20}},
    {"and", {"R", 0x33, 0x7, 0x00}},
    {"or",  {"R", 0x33, 0x6, 0x00}},
    {"sll", {"R", 0x33, 0x1, 0x00}},
    {"slt", {"R", 0x33, 0x2, 0x00}},
    {"sra", {"R", 0x33, 0x5, 0x20}},
    {"srl", {"R", 0x33, 0x5, 0x00}},
    {"xor", {"R", 0x33, 0x4, 0x00}},
    {"mul", {"R", 0x33, 0x0, 0x01}},
    {"div", {"R", 0x33, 0x4, 0x01}},
    {"rem", {"R", 0x33, 0x6, 0x01}},
    
    // I-type instructions
    {"addi", {"I", 0x13, 0x0, -1}},
    {"andi", {"I", 0x13, 0x7, -1}},
    {"ori",  {"I", 0x13, 0x6, -1}},
    {"lb",   {"I", 0x03, 0x0, -1}},
    {"lh",   {"I", 0x03, 0x1, -1}},
    {"lw",   {"I", 0x03, 0x2, -1}},
    {"ld",   {"I", 0x03, 0x3, -1}}, // Note: ld is for RV64 but included per spec.
    {"jalr", {"I", 0x67, 0x0, -1}},
    
    // S-type instructions
    {"sb",   {"S", 0x23, 0x0, -1}},
    {"sh",   {"S", 0x23, 0x1, -1}},
    {"sw",   {"S", 0x23, 0x2, -1}},
    {"sd",   {"S", 0x23, 0x3, -1}}, // sd is for RV64
    
    // SB-type (branch) instructions
    {"beq", {"SB", 0x63, 0x0, -1}},
    {"bne", {"SB", 0x63, 0x1, -1}},
    {"blt", {"SB", 0x63, 0x4, -1}},
    {"bge", {"SB", 0x63, 0x5, -1}},
    
    // U-type instructions
    {"lui",   {"U", 0x37, -1, -1}},
    {"auipc", {"U", 0x17, -1, -1}},
    
    // UJ-type instructions
    {"jal",   {"UJ", 0x6F, -1, -1}}
};

//--------------------------------------------------
// Process a single text (instruction) line during the second pass.
// 'address' is the current instruction address.
string processTextInstruction(const string &line, uint32_t address) {
    string trimmedLine = trim(line);
    if (trimmedLine.empty())
        return "";
    
    istringstream iss(trimmedLine);
    string mnemonic;
    iss >> mnemonic;
    
    if (instructionSet.find(mnemonic) == instructionSet.end()) {
        cerr << "Unknown instruction: " << mnemonic << endl;
        return "";
    }
    InstructionInfo info = instructionSet[mnemonic];
    
    // Get operands by splitting the rest of the line on commas.
    string rest;
    getline(iss, rest);
    vector<string> operands = split(rest, ',');
    for (auto &op : operands) {
        op = trim(op);
    }
    
    uint32_t machineCode = 0;
    string detail = "";
    
    if (info.type == "R") {
        // R-type: expects: rd, rs1, rs2.
        if (operands.size() != 3) {
            cerr << "R-type instruction requires 3 operands: " << line << endl;
            return "";
        }
        int rd = parseRegister(operands[0]);
        int rs1 = parseRegister(operands[1]);
        int rs2 = parseRegister(operands[2]);
        machineCode = encodeR(info.funct7, rs2, rs1, info.funct3, rd, info.opcode);
        detail = intToBinary(info.opcode, 7) + "-" +
                 intToBinary(info.funct3, 3) + "-" +
                 intToBinary(info.funct7, 7) + "-" +
                 intToBinary(rd, 5) + "-" +
                 intToBinary(rs1, 5) + "-" +
                 intToBinary(rs2, 5) + "-NULL";
    }
    else if (info.type == "I") {
        // I-type: for load/jalr (rd, offset(rs1)) or arithmetic (rd, rs1, imm).
        if (mnemonic == "lb" || mnemonic == "lh" || mnemonic == "lw" ||
            mnemonic == "ld" || mnemonic == "jalr") {
            if (operands.size() != 2) {
                cerr << "I-type (load/jalr) instruction requires 2 operands: " << line << endl;
                return "";
            }
            int rd = parseRegister(operands[0]);
            int rs1, imm;
            parseOffsetOperand(operands[1], rs1, imm);
            machineCode = encodeI(imm, rs1, info.funct3, rd, info.opcode);
            detail = intToBinary(info.opcode, 7) + "-" +
                     intToBinary(info.funct3, 3) + "-NULL-" +
                     intToBinary(rd, 5) + "-" +
                     intToBinary(rs1, 5) + "-" +
                     intToBinary(imm, 12);
        } else {
            if (operands.size() != 3) {
                cerr << "I-type arithmetic instruction requires 3 operands: " << line << endl;
                return "";
            }
            int rd = parseRegister(operands[0]);
            int rs1 = parseRegister(operands[1]);
            int imm = resolveImmediateValue(operands[2], address, info.type);
            machineCode = encodeI(imm, rs1, info.funct3, rd, info.opcode);
            detail = intToBinary(info.opcode, 7) + "-" +
                     intToBinary(info.funct3, 3) + "-NULL-" +
                     intToBinary(rd, 5) + "-" +
                     intToBinary(rs1, 5) + "-" +
                     intToBinary(imm, 12);
        }
    }
    else if (info.type == "S") {
        // S-type (store): format: rs2, offset(rs1)
        if (operands.size() != 2) {
            cerr << "S-type instruction requires 2 operands: " << line << endl;
            return "";
        }
        int rs2 = parseRegister(operands[0]);
        int rs1, imm;
        parseOffsetOperand(operands[1], rs1, imm);
        machineCode = encodeS(imm, rs2, rs1, info.funct3, info.opcode);
        detail = intToBinary(info.opcode, 7) + "-" +
                 intToBinary(info.funct3, 3) + "-NULL-NULL-" +
                 intToBinary(rs1, 5) + "-" +
                 intToBinary(rs2, 5) + "-" +
                 intToBinary(imm, 12);
    }
    else if (info.type == "SB") {
        // SB-type (branch): format: rs1, rs2, immediate (which can be a label).
        if (operands.size() != 3) {
            cerr << "SB-type instruction requires 3 operands: " << line << endl;
            return "";
        }
        int rs1 = parseRegister(operands[0]);
        int rs2 = parseRegister(operands[1]);
        int imm = resolveImmediateValue(operands[2], address, info.type);
        machineCode = encodeSB(imm, rs2, rs1, info.funct3, info.opcode);
        detail = intToBinary(info.opcode, 7) + "-" +
                 intToBinary(info.funct3, 3) + "-NULL-NULL-" +
                 intToBinary(rs1, 5) + "-" +
                 intToBinary(rs2, 5) + "-" +
                 intToBinary(imm, 13);
    }
    else if (info.type == "U") {
        // U-type: format: rd, immediate.
        if (operands.size() != 2) {
            cerr << "U-type instruction requires 2 operands: " << line << endl;
            return "";
        }
        int rd = parseRegister(operands[0]);
        int imm = resolveImmediateValue(operands[1], address, info.type);
        machineCode = encodeU(imm, rd, info.opcode);
        detail = intToBinary(info.opcode, 7) + "-NULL-NULL-" +
                 intToBinary(rd, 5) + "-NULL-NULL-" +
                 intToBinary(imm, 20);
    }
    else if (info.type == "UJ") {
        // UJ-type: format: rd, immediate (which can be a label).
        if (operands.size() != 2) {
            cerr << "UJ-type instruction requires 2 operands: " << line << endl;
            return "";
        }
        int rd = parseRegister(operands[0]);
        int imm = resolveImmediateValue(operands[1], address, info.type);
        machineCode = encodeUJ(imm, rd, info.opcode);
        detail = intToBinary(info.opcode, 7) + "-NULL-NULL-" +
                 intToBinary(rd, 5) + "-NULL-NULL-" +
                 intToBinary(imm, 21);
    }
    
    ostringstream oss;
    oss << "0x" << hex << address << " 0x" << setw(8) << setfill('0') << machineCode
        << " , " << trimmedLine << " # " << detail;
    return oss.str();
}

//--------------------------------------------------
// Process a data directive line (for the .data section) during the second pass.
vector<string> processDataDirective(const string &line, uint32_t &dataAddress) {
    vector<string> linesOut;
    string trimmed = trim(line);
    if (trimmed.empty())
        return linesOut;
    
    istringstream iss(trimmed);
    string directive;
    iss >> directive;
    
    if (directive == ".byte") {
        string rest;
        getline(iss, rest);
        vector<string> values = split(rest, ',');
        for (auto &val : values) {
            int byteVal = stoi(trim(val));
            ostringstream oss;
            oss << "0x" << hex << dataAddress << " 0x" 
                << setw(2) << setfill('0') << (byteVal & 0xFF);
            linesOut.push_back(oss.str());
            dataAddress += 1;
        }
    }
    else if (directive == ".half") {
        string rest;
        getline(iss, rest);
        vector<string> values = split(rest, ',');
        for (auto &val : values) {
            int halfVal = stoi(trim(val));
            ostringstream oss;
            oss << "0x" << hex << dataAddress << " 0x" 
                << setw(4) << setfill('0') << (halfVal & 0xFFFF);
            linesOut.push_back(oss.str());
            dataAddress += 2;
        }
    }
    else if (directive == ".word") {
        string rest;
        getline(iss, rest);
        vector<string> values = split(rest, ',');
        for (auto &val : values) {
            int wordVal = stoi(trim(val));
            ostringstream oss;
            oss << "0x" << hex << dataAddress << " 0x" 
                << setw(8) << setfill('0') << (wordVal & 0xFFFFFFFF);
            linesOut.push_back(oss.str());
            dataAddress += 4;
        }
    }
    else if (directive == ".dword") {
        string rest;
        getline(iss, rest);
        vector<string> values = split(rest, ',');
        for (auto &val : values) {
            long long dwordVal = stoll(trim(val));
            ostringstream oss;
            oss << "0x" << hex << dataAddress << " 0x" 
                << setw(16) << setfill('0') << (dwordVal & 0xFFFFFFFFFFFFFFFFLL);
            linesOut.push_back(oss.str());
            dataAddress += 8;
        }
    }
    else if (directive == ".asciz") {
        string str;
        getline(iss, str);
        size_t start = str.find('\"');
        size_t end = str.rfind('\"');
        if (start != string::npos && end != string::npos && end > start) {
            string content = str.substr(start + 1, end - start - 1);
            for (char c : content) {
                ostringstream oss;
                oss << "0x" << hex << dataAddress << " 0x" 
                    << setw(2) << setfill('0') << (int)(static_cast<unsigned char>(c));
                linesOut.push_back(oss.str());
                dataAddress += 1;
            }
            ostringstream oss;
            oss << "0x" << hex << dataAddress << " 0x00";
            linesOut.push_back(oss.str());
            dataAddress += 1;
        }
    }
    return linesOut;
}

//--------------------------------------------------
// Format a termination code line for the text segment.
string formatTerminator(uint32_t address) {
    uint32_t termCode = 0;  // 0x00000000 signals termination.
    ostringstream oss;
    oss << "0x" << hex << address << " 0x"
        << setw(8) << setfill('0') << termCode
        << " , TERM # NULL";
    return oss.str();
}

//--------------------------------------------------
// First pass: Scan the input lines and record labels with their addresses.
void firstPass(const vector<string>& lines) {
    uint32_t textAddr = 0x0;         // Code segment base.
    uint32_t dataAddr = 0x10000000;    // Data segment base.
    string currentSection = "";
    
    for (const auto &origLine : lines) {
        string line = trim(origLine);
        if (line.empty())
            continue;
        
        // Check for section directives.
        if (line.substr(0,5) == ".text") {
            currentSection = "text";
            continue;
        }
        if (line.substr(0,5) == ".data") {
            currentSection = "data";
            continue;
        }
        
        // If the line contains a label (indicated by a colon).
        size_t colonPos = line.find(':');
        if (colonPos != string::npos) {
            string label = trim(line.substr(0, colonPos));
            if (currentSection == "text")
                labelTable[label] = textAddr;
            else if (currentSection == "data")
                labelTable[label] = dataAddr;
            string remainder = trim(line.substr(colonPos + 1));
            if (remainder.empty())
                continue;
            line = remainder;
        }
        
        // Update address counters.
        if (currentSection == "text") {
            textAddr += 4; // One instruction per line.
        } else if (currentSection == "data") {
            istringstream iss(line);
            string directive;
            iss >> directive;
            if (directive == ".byte") {
                string rest;
                getline(iss, rest);
                vector<string> values = split(rest, ',');
                dataAddr += static_cast<uint32_t>(values.size());
            }
            else if (directive == ".half") {
                string rest;
                getline(iss, rest);
                vector<string> values = split(rest, ',');
                dataAddr += static_cast<uint32_t>(values.size() * 2);
            }
            else if (directive == ".word") {
                string rest;
                getline(iss, rest);
                vector<string> values = split(rest, ',');
                dataAddr += static_cast<uint32_t>(values.size() * 4);
            }
            else if (directive == ".dword") {
                string rest;
                getline(iss, rest);
                vector<string> values = split(rest, ',');
                dataAddr += static_cast<uint32_t>(values.size() * 8);
            }
            else if (directive == ".asciz") {
                string str;
                getline(iss, str);
                size_t start = str.find('\"');
                size_t end = str.rfind('\"');
                if (start != string::npos && end != string::npos && end > start) {
                    string content = str.substr(start + 1, end - start - 1);
                    dataAddr += static_cast<uint32_t>(content.size() + 1);
                }
            }
        }
    }
}

//--------------------------------------------------
// Main: Perform two passes over the input file.
int main() {
    ifstream infile("input.asm");
    if (!infile) {
        cerr << "Error: Cannot open input.asm file!" << endl;
        return 1;
    }
    
    vector<string> lines;
    string line;
    while (getline(infile, line)) {
        lines.push_back(line);
    }
    infile.close();
    
    // First pass: record all labels.
    firstPass(lines);
    
    // Second pass: generate machine-code output.
    uint32_t textAddress = 0x0;       // Reset text segment address.
    uint32_t dataAddress = 0x10000000;  // Reset data segment address.
    string currentSection = "";
    vector<string> outputLines;
    
    for (const auto &origLine : lines) {
        string currLine = trim(origLine);
        if (currLine.empty())
            continue;
        
        // Section directives.
        if (currLine.substr(0,5) == ".text") {
            currentSection = "text";
            continue;
        }
        if (currLine.substr(0,5) == ".data") {
            currentSection = "data";
            continue;
        }
        
        // Remove any label if present.
        size_t colonPos = currLine.find(':');
        if (colonPos != string::npos) {
            string remainder = trim(currLine.substr(colonPos + 1));
            if (remainder.empty())
                continue;
            currLine = remainder;
        }
        
        if (currentSection == "text") {
            string outLine = processTextInstruction(currLine, textAddress);
            if (!outLine.empty()) {
                outputLines.push_back(outLine);
                textAddress += 4;
            }
        } else if (currentSection == "data") {
            vector<string> dataLines = processDataDirective(currLine, dataAddress);
            for (const auto &dl : dataLines)
                outputLines.push_back(dl);
        }
    }
    
    // Append termination code for the text segment.
    outputLines.push_back(formatTerminator(textAddress));
    
    ofstream outfile("output.mc");
    if (!outfile) {
        cerr << "Error: Cannot create output.mc file!" << endl;
        return 1;
    }
    for (const auto &ol : outputLines)
        outfile << ol << "\n";
    outfile.close();
    
    cout << "Assembly to machine-code conversion complete. See output.mc" << endl;
    return 0;
}
