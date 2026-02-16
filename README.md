# Fault Injection Attacks on Non-Volatile Memories (NVMs)
This project focuses on performing fault injection attacks on non-volatile memories(FRAM and MRAM) using an STM32 microcontroller. The goal is to analyze how memory data 
gets corrupted under controlled fault conditions and to classify different fault patterns.

## Project Objective
- To study the behavior of FRAM and MRAM under fault injection
- To analyze data corruption patterns caused by EMFI
- To classify faults such as bit flips, nibble shifts, and partial writes

## Hardware Components
- STM32 Microcontroller
- FRAM / MRAM (SPI-based)
- Power control circuitry
- Impulse / fault injection generator

## Software & Tools
- STM32CubeIDE
- Embedded C
- Python (for data analysis)
- CSV data logging

## Methodology
1. Data is written to memory in fixed-size chunks using SPI communication.
2. A fault is injected during the write operation by:
   - Power interruption
   - Timing-based fault trigger
3. The system power is restored, and data is read back from memory.
4. Written and read values are logged through UART and saved as CSV files.
5. Python scripts are used to analyze and classify fault patterns.

## Fault Types Analyzed
- Single-bit faults
- Multi-bit faults
- High-nibble and low-nibble corruption
- Shift and rotation-based faults
- Partial write failures

## Results
The experiments show that fault injection can cause predictable and repeatable corruption 
patterns in non-volatile memories. These patterns can be classified and used to better 
understand memory reliability and security vulnerabilities.

## Applications
- Hardware security research
- Reliability testing of embedded memories
- Fault tolerance evaluation
- Secure embedded system design
