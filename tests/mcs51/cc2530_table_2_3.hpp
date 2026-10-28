#pragma once

// CC2530 CPU timing facts for the tests, transcribed from TI SWRU191F (not
// read from the emulator). No test macros here, so that every CC2530 test
// support header can include it.

#include <cstdint>
#include <stdexcept>

namespace swru191f {

// CC2530 timing contract (docs/cc2530.md "Timing contract"): the CPU clock
// counts of TI SWRU191F Table 2-3, Instruction Set Summary, transcribed row
// by row from the datasheet (line numbers of the SWRU191F text rendering,
// swru191f.txt), not read from the emulator. test_cc2530f256_r2's T0 checks
// the engine against it for every opcode.
struct Table23Row {
    const char *mnemonic;
    std::uint8_t first, last; // "Hex Opcode" column, a single opcode or a range
    unsigned cycles;          // "Cycles" column
    unsigned line;            // swru191f.txt line
};
// "xxx11"/"xxx01" (ACALL/AJMP) are the eight opcodes with these low five bits.
inline constexpr std::uint8_t AcallLow = 0x11, AjmpLow = 0x01;
inline constexpr Table23Row table_2_3[] = {
    // ARITHMETIC OPERATIONS
    {"ADD A,Rn", 0x28, 0x2f, 1, 2060},
    {"ADD A,direct", 0x25, 0x25, 2, 2061},
    {"ADD A,@Ri", 0x26, 0x27, 2, 2062},
    {"ADD A,#data", 0x24, 0x24, 2, 2063},
    {"ADDC A,Rn", 0x38, 0x3f, 1, 2064},
    {"ADDC A,direct", 0x35, 0x35, 2, 2065},
    {"ADDC A,@Ri", 0x36, 0x37, 2, 2066},
    {"ADDC A,#data", 0x34, 0x34, 2, 2067},
    {"SUBB A,Rn", 0x98, 0x9f, 1, 2068},
    {"SUBB A,direct", 0x95, 0x95, 2, 2069},
    {"SUBB A,@Ri", 0x96, 0x97, 2, 2070},
    {"SUBB A,#data", 0x94, 0x94, 2, 2071},
    {"INC A", 0x04, 0x04, 1, 2072},
    {"INC Rn", 0x08, 0x0f, 2, 2073},
    {"INC direct", 0x05, 0x05, 3, 2074},
    {"INC @Ri", 0x06, 0x07, 3, 2075},
    {"INC DPTR", 0xa3, 0xa3, 1, 2076},
    {"DEC A", 0x14, 0x14, 1, 2077},
    {"DEC Rn", 0x18, 0x1f, 2, 2078},
    {"DEC direct", 0x15, 0x15, 3, 2079},
    {"DEC @Ri", 0x16, 0x17, 3, 2080},
    {"MUL AB", 0xa4, 0xa4, 5, 2081},
    {"DIV A", 0x84, 0x84, 5, 2082}, // DIV AB
    {"DA A", 0xd4, 0xd4, 1, 2083},
    // LOGICAL OPERATIONS
    {"ANL A,Rn", 0x58, 0x5f, 1, 2085},
    {"ANL A,direct", 0x55, 0x55, 2, 2086},
    {"ANL A,@Ri", 0x56, 0x57, 2, 2087},
    {"ANL A,#data", 0x54, 0x54, 2, 2088},
    {"ANL direct,A", 0x52, 0x52, 3, 2089},
    {"ANL direct,#data", 0x53, 0x53, 4, 2090},
    {"ORL A,Rn", 0x48, 0x4f, 1, 2091},
    {"ORL A,direct", 0x45, 0x45, 2, 2092},
    {"ORL A,@Ri", 0x46, 0x47, 2, 2093},
    {"ORL A,#data", 0x44, 0x44, 2, 2094},
    {"ORL direct,A", 0x42, 0x42, 3, 2095},
    {"ORL direct,#data", 0x43, 0x43, 4, 2096},
    {"XRL A,Rn", 0x68, 0x6f, 1, 2097},
    {"XRL A,direct", 0x65, 0x65, 2, 2098},
    {"XRL A,@Ri", 0x66, 0x67, 2, 2099},
    {"XRL A,#data", 0x64, 0x64, 2, 2100},
    {"XRL direct,A", 0x62, 0x62, 3, 2101},
    {"XRL direct,#data", 0x63, 0x63, 4, 2113},
    {"CLR A", 0xe4, 0xe4, 1, 2114},
    {"CPL A", 0xf4, 0xf4, 1, 2115},
    {"RL A", 0x23, 0x23, 1, 2116},
    {"RLC A", 0x33, 0x33, 1, 2117},
    {"RR A", 0x03, 0x03, 1, 2118},
    {"RRC A", 0x13, 0x13, 1, 2119},
    {"SWAP A", 0xc4, 0xc4, 1, 2120},
    // DATA TRANSFERS
    {"MOV A,Rn", 0xe8, 0xef, 1, 2122},
    {"MOV A,direct", 0xe5, 0xe5, 2, 2123},
    {"MOV A,@Ri", 0xe6, 0xe7, 2, 2124},
    {"MOV A,#data", 0x74, 0x74, 2, 2125},
    {"MOV Rn,A", 0xf8, 0xff, 2, 2126},
    {"MOV Rn,direct", 0xa8, 0xaf, 4, 2127},
    {"MOV Rn,#data", 0x78, 0x7f, 2, 2128},
    {"MOV direct,A", 0xf5, 0xf5, 3, 2129},
    {"MOV direct,Rn", 0x88, 0x8f, 3, 2130},
    {"MOV direct1,direct2", 0x85, 0x85, 4, 2131},
    {"MOV direct,@Ri", 0x86, 0x87, 4, 2132},
    {"MOV direct,#data", 0x75, 0x75, 3, 2133},
    {"MOV @Ri,A", 0xf6, 0xf7, 3, 2134},
    {"MOV @Ri,direct", 0xa6, 0xa7, 5, 2135},
    {"MOV @Ri,#data", 0x76, 0x77, 3, 2136},
    {"MOV DPTR,#data16", 0x90, 0x90, 3, 2137},
    {"MOVC A,@A+DPTR", 0x93, 0x93, 3, 2138},
    {"MOVC A,@A+PC", 0x83, 0x83, 3, 2139},
    {"MOVX A,@Ri", 0xe2, 0xe3, 3, 2140},
    {"MOVX A,@DPTR", 0xe0, 0xe0, 3, 2141},
    {"MOVX @Ri,A", 0xf2, 0xf3, 4, 2142},
    {"MOVX @DPTR,A", 0xf0, 0xf0, 4, 2143},
    {"PUSH direct", 0xc0, 0xc0, 4, 2144},
    {"POP direct", 0xd0, 0xd0, 3, 2145},
    {"XCH A,Rn", 0xc8, 0xcf, 2, 2146},
    {"XCH A,direct", 0xc5, 0xc5, 3, 2147},
    {"XCH A,@Ri", 0xc6, 0xc7, 3, 2148},
    {"XCHD A,@Ri", 0xd6, 0xd7, 3, 2149},
    // PROGRAM BRANCHING (one count per conditional branch, jump or not)
    {"ACALL addr11", AcallLow, AcallLow, 6, 2151}, // xxx11
    {"LCALL addr16", 0x12, 0x12, 6, 2152},
    {"RET", 0x22, 0x22, 4, 2153},
    {"RETI", 0x32, 0x32, 4, 2154},
    {"AJMP addr11", AjmpLow, AjmpLow, 3, 2155}, // xxx01
    {"LJMP addr16", 0x02, 0x02, 4, 2156},
    {"SJMP rel", 0x80, 0x80, 3, 2157},
    {"JMP @A+DPTR", 0x73, 0x73, 2, 2158},
    {"JZ rel", 0x60, 0x60, 3, 2159},
    {"JNZ rel", 0x70, 0x70, 3, 2169},
    {"JC rel", 0x40, 0x40, 3, 2170},
    {"JNC", 0x50, 0x50, 3, 2171},
    {"JB bit,rel", 0x20, 0x20, 4, 2172},
    {"JNB bit,rel", 0x30, 0x30, 4, 2173},
    {"JBC bit,direct rel", 0x10, 0x10, 4, 2174},
    {"CJNE A,direct rel", 0xb5, 0xb5, 4, 2175},
    {"CJNE A,#data rel", 0xb4, 0xb4, 4, 2176},
    {"CJNE Rn,#data rel", 0xb8, 0xbf, 4, 2177},
    {"CJNE @Ri,#data rel", 0xb6, 0xb7, 4, 2178},
    {"DJNZ Rn,rel", 0xd8, 0xdf, 3, 2179}, // listed as 1 byte; it has 2
    {"DJNZ direct,rel", 0xd5, 0xd5, 4, 2180},
    {"NOP", 0x00, 0x00, 1, 2181},
    // BOOLEAN VARIABLE OPERATIONS
    {"CLR C", 0xc3, 0xc3, 1, 2183},
    {"CLR bit", 0xc2, 0xc2, 3, 2184},
    {"SETB C", 0xd3, 0xd3, 1, 2185},
    {"SETB bit", 0xd2, 0xd2, 3, 2186},
    {"CPL C", 0xb3, 0xb3, 1, 2187},
    {"CPL bit", 0xb2, 0xb2, 3, 2188},
    {"ANL C,bit", 0x82, 0x82, 2, 2189},
    {"ANL C,/bit", 0xb0, 0xb0, 2, 2190},
    {"ORL C,bit", 0x72, 0x72, 2, 2191},
    {"ORL C,/bit", 0xa0, 0xa0, 2, 2192},
    {"MOV C,bit", 0xa2, 0xa2, 2, 2193},
    {"MOV bit,C", 0x92, 0x92, 3, 2194},
};
constexpr bool table_2_3_covers(const Table23Row &row, std::uint8_t op) {
    if (row.first == AcallLow || row.first == AjmpLow)
        return (op & 0x1fu) == row.first;
    return op >= row.first && op <= row.last;
}
// The Table 2-3 row of `op` (0xa5, the one reserved opcode, has none).
constexpr const Table23Row *table_2_3_row(std::uint8_t op) {
    for (const Table23Row &row : table_2_3)
        if (table_2_3_covers(row, op))
            return &row;
    return nullptr;
}
constexpr unsigned contract_cycles(std::uint8_t op) {
    const Table23Row *row = table_2_3_row(op);
    if (!row)
        throw std::logic_error("opcode without a Table 2-3 row");
    return row->cycles;
}
// Interrupt entry: SWRU191F 2.5.2 (swru191f.txt lines 2538-2539), "The
// fastest possible response to an interrupt is seven machine cycles. This
// includes one machine cycle for detecting the interrupt and six cycles to
// perform the LCALL." The model takes a request at an instruction boundary
// and charges exactly these 7 cycles; the entry counts as one instruction.
constexpr unsigned EntryCycles = 7;

} // namespace swru191f
