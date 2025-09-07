
`timescale 1ns / 1ps


module cpu_19bit (
    input clk,
    input reset
);

    // Architectural Parameters
    parameter REG_ADDR_WIDTH = 3;
    parameter DATA_WIDTH = 16;
    parameter INSTR_WIDTH = 19;
    parameter PC_WIDTH = 15;
    parameter MEM_ADDR_WIDTH = 12;

    // Internal Signals
    reg [PC_WIDTH-1:0] pc;
    wire [INSTR_WIDTH-1:0] current_instruction;
    
    // Instruction Fields
    wire [3:0] opcode;
    wire [REG_ADDR_WIDTH-1:0] rd, rs1, rs2;
    wire [MEM_ADDR_WIDTH-1:0] immediate;
    wire [PC_WIDTH-1:0] jump_addr;
    wire [REG_ADDR_WIDTH-1:0] reg_addr_jmp_branch;
    
    // Control Signals
    wire reg_write_en;
    wire mem_read_en, mem_write_en;
    wire pc_branch, pc_jump, pc_call, pc_ret;
    wire [3:0] alu_op;
    
    // ALU Signals
    wire [DATA_WIDTH-1:0] alu_op1_val, alu_op2_val;
    wire [DATA_WIDTH-1:0] alu_result;
    wire zero_flag;
    
    // Register File Signals
    wire [DATA_WIDTH-1:0] reg_data_read1, reg_data_read2;
    wire [DATA_WIDTH-1:0] reg_data_write;
    
    // Memory Signals
    wire [DATA_WIDTH-1:0] mem_read_data;
    wire [DATA_WIDTH-1:0] mem_write_data;
    
    // Stack Pointer for CALL/RET
    reg [PC_WIDTH-1:0] stack [0:63]; // Simple stack
    reg [5:0] sp;

    reg [INSTR_WIDTH-1:0] instruction_memory [2**PC_WIDTH-1:0];
    initial begin
        // Example Program - Loaded by testbench
    end
    
    assign current_instruction = instruction_memory[pc];
  
    assign opcode = current_instruction[18:15];
    
    // R-Type format: Opcode | rd | rs1 | rs2 | Unused
    assign rd = current_instruction[14:12];
    assign rs1 = current_instruction[11:9];
    assign rs2 = current_instruction[8:6];
    
    // I-Type format: Opcode | rd/rs | Address/Immediate
    assign reg_addr_jmp_branch = current_instruction[14:12];
    assign immediate = current_instruction[11:0];
    
    // J-Type format: Opcode | Address
    assign jump_addr = current_instruction[14:0];
   
    // Control Unit
    // Generates all control signals based on the opcode
  
    control_unit ctrl (
        .opcode(opcode),
        .zero_flag(zero_flag),
        .reg_write_en(reg_write_en),
        .mem_read_en(mem_read_en),
        .mem_write_en(mem_write_en),
        .pc_branch(pc_branch),
        .pc_jump(pc_jump),
        .pc_call(pc_call),
        .pc_ret(pc_ret),
        .alu_op(alu_op)
    );

    
    // PC (Program Counter) Logic
 
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= {PC_WIDTH{1'b0}};
        end else begin
            if (pc_jump) begin
                pc <= jump_addr;
            end else if (pc_branch) begin
                pc <= immediate;
            end else if (pc_call) begin
                stack[sp] <= pc + 1;
                sp <= sp + 1;
                pc <= jump_addr;
            end else if (pc_ret) begin
                sp <= sp - 1;
                pc <= stack[sp - 1];
            end else begin
                pc <= pc + 1;
            end
        end
    end
    
   
    // Register File
  
    register_file reg_file (
        .clk(clk),
        .read_addr1(rs1),
        .read_addr2(rs2),
        .write_addr(rd),
        .write_en(reg_write_en),
        .write_data(reg_data_write),
        .read_data1(reg_data_read1),
        .read_data2(reg_data_read2)
    );
    
    // ALU (Arithmetic Logic Unit)
   
    alu alu_inst (
        .in1(alu_op1_val),
        .in2(alu_op2_val),
        .alu_op(alu_op),
        .result(alu_result),
        .zero_flag(zero_flag)
    );

    // ALU Muxes for operand selection
    assign alu_op1_val = reg_data_read1;
    assign alu_op2_val = (opcode[15]) ? immediate : reg_data_read2; // Simple immediate support
   
    // Data Memory Interface
 
    reg [DATA_WIDTH-1:0] data_memory [2**MEM_ADDR_WIDTH-1:0];
    
    assign mem_write_data = reg_data_read1; // For ST instruction
    assign reg_data_write = (mem_read_en) ? mem_read_data : alu_result;
    
    always @(posedge clk) begin
        if (mem_write_en) begin
            data_memory[immediate] <= mem_write_data;
        end
    end
    
    assign mem_read_data = data_memory[immediate];
   
    // Specialized Instructions (Stubs)
    
    // FFT
    fftc_core #(DATA_WIDTH) fft_unit (
        .data_in(reg_data_read2),
        .data_out(alu_result)
    );
    
    // Encryption/Decryption
    crypto_core #(DATA_WIDTH) crypto_unit (
        .data_in(reg_data_read2),
        .data_out(alu_result)
    );
    
endmodule

// Sub-Modules (simplified for clarity)


module control_unit (
    input [3:0] opcode,
    input zero_flag,
    output reg reg_write_en,
    output reg mem_read_en,
    output reg mem_write_en,
    output reg pc_branch,
    output reg pc_jump,
    output reg pc_call,
    output reg pc_ret,
    output reg [3:0] alu_op
);
    always @(*) begin
        // Default values
        reg_write_en = 0;
        mem_read_en = 0;
        mem_write_en = 0;
        pc_branch = 0;
        pc_jump = 0;
        pc_call = 0;
        pc_ret = 0;
        alu_op = 4'b0000;
        
        case (opcode)
            // Arithmetic
            4'b0000: begin reg_write_en = 1; alu_op = 4'b0000; end // ADD
            4'b0001: begin reg_write_en = 1; alu_op = 4'b0001; end // SUB
            4'b0010: begin reg_write_en = 1; alu_op = 4'b0010; end // MUL
            4'b0011: begin reg_write_en = 1; alu_op = 4'b0011; end // DIV
            4'b0100: begin reg_write_en = 1; alu_op = 4'b0100; end // INC
            4'b0101: begin reg_write_en = 1; alu_op = 4'b0101; end // DEC
            
            // Logical
            4'b0110: begin reg_write_en = 1; alu_op = 4'b0110; end // AND
            4'b0111: begin reg_write_en = 1; alu_op = 4'b0111; end // OR
            4'b1000: begin reg_write_en = 1; alu_op = 4'b1000; end // XOR
            4'b1001: begin reg_write_en = 1; alu_op = 4'b1001; end // NOT
            
            // Control Flow
            4'b1010: begin pc_jump = 1; end // JMP
            4'b1011: begin pc_branch = zero_flag; end // BEQ
            4'b1100: begin pc_branch = ~zero_flag; end // BNE
            4'b1101: begin pc_call = 1; end // CALL
            4'b1110: begin pc_ret = 1; end // RET
            
            // Memory Access
            4'b1111: begin mem_read_en = 1; reg_write_en = 1; end // LD
            4'b0000: begin mem_write_en = 1; end // ST (reusing opcode 0 to avoid collision)
            
            // Custom
            4'b0001: begin reg_write_en = 1; alu_op = 4'b1110; end // FFT (reusing)
            4'b0010: begin reg_write_en = 1; alu_op = 4'b1111; end // ENC (reusing)
            4'b0011: begin reg_write_en = 1; alu_op = 4'b1101; end // DEC (reusing)
        endcase
    end
endmodule

module alu (
    input [15:0] in1, in2,
    input [3:0] alu_op,
    output reg [15:0] result,
    output zero_flag
);
    assign zero_flag = (result == 0);
    
    always @(*) begin
        case (alu_op)
            4'b0000: result = in1 + in2; // ADD
            4'b0001: result = in1 - in2; // SUB
            4'b0010: result = in1 * in2; // MUL
            4'b0011: result = in1 / in2; // DIV
            4'b0100: result = in1 + 1;   // INC
            4'b0101: result = in1 - 1;   // DEC
            4'b0110: result = in1 & in2; // AND
            4'b0111: result = in1 | in2; // OR
            4'b1000: result = in1 ^ in2; // XOR
            4'b1001: result = ~in1;      // NOT
            default: result = 16'b0;
        endcase
    end
endmodule

module register_file (
    input clk,
    input [2:0] read_addr1, read_addr2, write_addr,
    input write_en,
    input [15:0] write_data,
    output [15:0] read_data1, read_data2
);
    reg [15:0] registers [0:7];
    
    // Read from registers
    assign read_data1 = (read_addr1 == 3'b000) ? 16'b0 : registers[read_addr1];
    assign read_data2 = (read_addr2 == 3'b000) ? 16'b0 : registers[read_addr2];
    
    // Write to register file
    always @(posedge clk) begin
        if (write_en && write_addr != 3'b000) begin
            registers[write_addr] <= write_data;
        end
    end
endmodule

// Stub module for FFT core
module fftc_core (
    input [15:0] data_in,
    output reg [15:0] data_out
);
    always @(*) begin
        // Placeholder for complex FFT logic
        // For simulation, we'll just double the value
        data_out = data_in * 2;
    end
endmodule

// Stub module for Cryptography core
module crypto_core (
    input [15:0] data_in,
    output reg [15:0] data_out
);
    always @(*) begin
        // Placeholder for complex encryption/decryption logic
        // For simulation, we'll just bitwise NOT the value
        data_out = ~data_in;
    end
endmodule



`timescale 1ns / 1ps
module cpu_19bit_testbench;

    // Testbench signals
    reg clk, reset;
    
    // Instantiate the CPU
    cpu_19bit UUT (
        .clk(clk),
        .reset(reset)
    );
  
    initial begin
        clk = 0;
        forever #5 clk = ~clk; 
    end
  
    initial begin
      
        $dumpfile("cpu_19bit.vcd");
        $dumpvars(0, cpu_19bit_testbench);
        
      
        reset = 1;
        #10;
        reset = 0;
       
        UUT.data_memory[100] = 16'd50;
        UUT.data_memory[101] = 16'd10;
        
        UUT.instruction_memory[0] = {4'b1111, 3'd1, 12'd100}; // LD r1, 100
        UUT.instruction_memory[1] = {4'b1111, 3'd2, 12'd101}; // LD r2, 101
        UUT.instruction_memory[2] = {4'b0000, 3'd3, 3'd1, 3'd2, 6'b0}; // ADD r3, r1, r2 (50+10 = 60)
        UUT.instruction_memory[3] = {4'b0001, 3'd4, 3'd1, 3'd2, 6'b0}; // SUB r4, r1, r2 (50-10 = 40)
        UUT.instruction_memory[4] = {4'b0010, 3'd5, 3'd1, 3'd2, 6'b0}; // MUL r5, r1, r2 (50*10 = 500)
        UUT.instruction_memory[5] = {4'b0011, 3'd6, 3'd1, 3'd2, 6'b0}; // DIV r6, r1, r2 (50/10 = 5)
        UUT.instruction_memory[6] = {4'b0100, 3'd1, 9'b0}; // INC r1 (50+1 = 51)
        UUT.instruction_memory[7] = {4'b0101, 3'd2, 9'b0}; // DEC r2 (10-1 = 9)

        // B. Logical Operations (Addresses 8-11)
        UUT.instruction_memory[8] = {4'b0110, 3'd3, 3'd1, 3'd2, 6'b0}; // AND r3, r1, r2 (51&9 = 1)
        UUT.instruction_memory[9] = {4'b0111, 3'd4, 3'd1, 3'd2, 6'b0}; // OR r4, r1, r2 (51|9 = 59)
        UUT.instruction_memory[10] = {4'b1000, 3'd5, 3'd1, 3'd2, 6'b0}; // XOR r5, r1, r2 (51^9 = 58)
        UUT.instruction_memory[11] = {4'b1001, 3'd6, 3'd1, 9'b0}; // NOT r6, r1 (~51)

        // C. Control Flow (Addresses 12-16)
        UUT.instruction_memory[12] = {4'b1011, 3'd1, 3'd2, 12'd15}; // BEQ r1,r2, 15 (should not branch)
        UUT.instruction_memory[13] = {4'b1100, 3'd1, 3'd2, 12'd16}; // BNE r1,r2, 16 (should branch to 16)
        UUT.instruction_memory[14] = {4'b0000, 3'd0, 3'd0, 3'd0, 6'b0}; // Placeholder, will be skipped
        UUT.instruction_memory[15] = {4'b1010, 15'd20}; // JMP 20
        UUT.instruction_memory[16] = {4'b1101, 15'd25}; // CALL 25
        UUT.instruction_memory[17] = {4'b0000, 3'd0, 3'd0, 3'd0, 6'b0}; // Should be skipped by CALL
        
        // Subroutine at 25
        UUT.instruction_memory[25] = {4'b0100, 3'd7, 9'b0}; // INC r7
        UUT.instruction_memory[26] = {4'b1110, 15'b0}; // RET

        // D. Memory Access (Addresses 18-19)
        UUT.instruction_memory[18] = {4'b1111, 3'd1, 12'd102}; // LD r1, 102
        UUT.instruction_memory[19] = {4'b0000, 3'd1, 12'd103}; // ST 103, r1

        UUT.data_memory[102] = 16'd300;

        // E. Custom Instructions (Addresses 20-22)
        // Set initial data for custom ops
        UUT.data_memory[104] = 16'd40;
        UUT.instruction_memory[20] = {4'b1111, 3'd1, 12'd104}; // LD r1, 104
        UUT.instruction_memory[21] = {4'b0001, 3'd2, 3'd1, 9'b0}; // FFT r2, r1
        UUT.instruction_memory[22] = {4'b0010, 3'd3, 3'd1, 9'b0}; // ENC r3, r1
        UUT.instruction_memory[23] = {4'b0011, 3'd4, 3'd3, 9'b0}; // DEC r4, r3

        // Run simulation for 250ns to cover all instructions
        #250; 
        
        // End simulation
        $finish;
    end
    
endmodule

