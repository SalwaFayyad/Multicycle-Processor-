# Multicycle-Processor

## Processor Specifications

- The instruction size and the word size is 16 bits.
- 8 16-bit general-purpose registers: from R0 to R7.
- R0 is hardwired to zero. Any attempt to write to it will be discarded.
- 16-bit special purpose register for the program counter (PC).
- Four instruction types (R-type, I-type, J-type, and S-type).
- Separate data and instruction memories.
- Byte addressable memory.
- Little endian byte ordering.
- The ALU generates required signals to calculate the condition branch outcome (taken/not taken). These signals might include zero, carry, overflow, etc.
- Support for basic arithmetic, logical, and control operations.
- Immediate values are sign-extended where applicable.
- Conditional and unconditional branching supported.
- Function call and return mechanism with a dedicated return address register.

## Instruction Types and Formats

### R-Type (Register Type) Instruction Format
| Field      | Bits   | Description              |
|------------|--------|--------------------------|
| Opcode     | 4 bits | Specifies the operation  |
| Rd         | 3 bits | Destination register     |
| Rs1        | 3 bits | First source register    |
| Rs2        | 3 bits | Second source register   |
| Unused     | 3 bits | Unused bits              |

### I-Type (Immediate Type) Instruction Format
| Field      | Bits   | Description                                   |
|------------|--------|-----------------------------------------------|
| Opcode     | 4 bits | Specifies the operation                       |
| Mode       | 1 bit  | Mode bit for load/branch instructions         |
| Rd         | 3 bits | Destination register                          |
| Rs1        | 3 bits | First source register                         |
| Immediate  | 5 bits | Immediate value (unsigned for logic, signed otherwise) |

### J-Type (Jump Type) Instruction Formats

#### Unconditional Jump
| Field      | Bits   | Description             |
|------------|--------|-------------------------|
| Opcode     | 4 bits | Specifies the operation |
| Jump Offset| 12 bits| Jump offset             |

#### Call Function
| Field      | Bits   | Description                                 |
|------------|--------|---------------------------------------------|
| Opcode     | 4 bits | Specifies the operation                     |
| Jump Offset| 12 bits| Jump offset (target address calculation)    |

#### Return from Function
| Field      | Bits   | Description             |
|------------|--------|-------------------------|
| Opcode     | 4 bits | Specifies the operation |
| Unused     | 12 bits| Unused bits             |

### S-Type (Store) Instruction Format
| Field      | Bits   | Description              |
|------------|--------|--------------------------|
| Opcode     | 4 bits | Specifies the operation  |
| Rs         | 3 bits | Source register          |
| Immediate  | 8 bits | Immediate value to store |

## Contributors

- [Salwa Fayyad](https://github.com/SalwaFayyad)
- [Sami Moqbel](https://github.com/SamiMoqbel)
- [Lama Abuthaher](https://github.com/lamahabu)

![image](https://github.com/SalwaFayyad/Multicycle-Processor-/assets/104863637/4cd0bdbf-91cd-4914-9278-925878367b05)


