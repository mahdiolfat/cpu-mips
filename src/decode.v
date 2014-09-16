`include "const.v"

/**** DECODE MODULE ****/
module decode (
clk_in, 
insn_in, pc_in, 
rs, rt, 
DX_PC, DX_IR,
s_immed, alu_funk, alu_in_1_sel, alu_in_2_sel,
wb_we, wb_reg_addr, 
mem_out_sel, mem_access_size, mem_rw,
xm_o_sel, d1_xm_o
);

// CPU clock
input 			clk_in;
// Receives the instruction word from memory
// associated with the supplied PC
input   [31:0]  insn_in;
// Receives the PC from fetch stage for the instruction 
// associated with the supplied instruction
input   [31:0]  pc_in;
// Pipeline register (D/X) holds PC value
output	[31:0]	DX_PC;
// Pipeline register (D/X) holds instruction value
output	[31:0]	DX_IR;	
// The address of the first source register
// Supplied to the register file
output	[4:0]	rs;
// The address of the second source register 
// Supplied to the register file
output	[4:0]	rt;
// The sign-extended immediate value from in the instruction
output	signed 	[31:0]	s_immed;

/***** Control Signals *****/
// The function of the ALU operation
output	[5:0]	alu_funk;
// The mux select line for first ALU input
output			alu_in_1_sel;
// The mux select line for second ALU input
output			alu_in_2_sel;
// Dictates whether the reg file will be written
output			wb_we;
// The address of the register to write to
output	[4:0]	wb_reg_addr;
// The mux select line for the memory stage output
// Chooses between ALU output and memory output
output			mem_out_sel;
// Memory access size
output	[1:0]	mem_access_size;
// Memory rw signal 
output			mem_rw;
// The mux select line for execute stage output
// Chooses between ALU output and special value to
// write to a register (e.g., for LUI and JAL insns)
output			xm_o_sel;
// This is used as a MUX input for the execute to
// memory stage output. It's used for writing special values
// to a register in the wb stage (e.g, for LUI and JAL insns)
output	[31:0]	d1_xm_o;

// Internal declarations used for decoding instruction
reg     	[5:0]   op;
reg     	[4:0]   rs;
reg     	[4:0]   rt;
reg     	[4:0]   rd;
reg     	[4:0]   sh;
reg     	[5:0]   func;
reg     	[15:0]  immed;
reg	signed 	[31:0] 	s_immed;

reg			[31:0]	DX_PC;
reg			[31:0]	DX_IR;
reg			[5:0]	alu_funk;
reg					wb_we;
reg					mem_out_sel;
reg			[4:0]	wb_reg_addr;
reg			[1:0]	mem_access_size;
reg					mem_rw;
reg					xm_o_sel;
reg			[31:0]	d1_xm_o;
reg					alu_in_1_sel;
reg					alu_in_2_sel;

// Select signal for inserting a nop bubble
reg					nop_sel;
// Hold the IR value in case of a nop bubble
reg 		[31:0]	DX_IR_reg;

// Set initial values for the decode stage
initial begin
	wb_we 			= 1'b0;
	mem_out_sel 	= 1'b1;
	mem_access_size = 2'b00;
	mem_rw			= 1'b1;
	xm_o_sel		= 1'b0;
	alu_in_1_sel 	= 1'b0;
	alu_in_2_sel	= 1'b0;
end

always @ (pc_in) begin
	if (nop_sel == 1'b0) begin
		DX_IR = insn_in;
		DX_PC = pc_in;
	end else begin
		// Insert a nop bubble
		DX_IR_reg = insn_in;
		DX_IR = `NOP;
		@ (posedge clk_in);
		DX_IR = DX_IR_reg;
	end
end

// Decode the instruction
always @ (DX_IR) begin
	mem_rw 		= 1'b1;
	xm_o_sel	= 1'b0;

	// Strip away all instruction bit-fields
	op      = DX_IR[`OP_e:`OP_s];
    rs      = DX_IR[`RS_e:`RS_s];
    rt      = DX_IR[`RT_e:`RT_s];
    rd      = DX_IR[`RD_e:`RD_s];
    sh      = DX_IR[`SH_e:`SH_s];
    func    = DX_IR[`FUNK_e:`FUNK_s];
    immed   = DX_IR[`I_e:`I_s];
	s_immed	= {{16{DX_IR[15]}}, DX_IR[15:0]};

	$display("TIME: %g DECODE PC = %h", $time, pc_in);
		
	// Decode instructions and set control signals accordingly
	case (op)
		`SPECIAL:
			case (func)
				`SLL: 	begin
					$display("%h\t\t sll %d, %d, %d", DX_IR, rd, rt, sh);
					wb_we			= 1'b1;
					mem_out_sel		= 1'b1;
					alu_in_1_sel	= 1'b1;
					alu_in_2_sel	= 1'b0;
					alu_funk		= `SLL;
					wb_reg_addr		= rd;
				end
				`SLT: 	begin
					$display("%h\t\t slt %d, %d, %d", DX_IR, rd, rs, rt);
					wb_we 			= 1'b1;
					mem_out_sel 	= 1'b1;
					alu_in_1_sel	= 1'b0;
					alu_in_2_sel	= 1'b0;
					alu_funk		= `SLT;
					wb_reg_addr 	= rd;
				end
				`SLTU: $display("%h\t\t sltu %d, %d, %d", DX_IR, rd, rs, rt);
				`SRA: $display("%h\t\t sra %d, %d, %d", DX_IR, rd, rt, sh);
				`SRL: $display("%h\t\t srl %d, %d, %d", DX_IR, rd, rt, sh);
				`SUB: 	begin
					$display("%h\t\t sub %d, %d, %d", DX_IR, rd, rs, rt);
					wb_we 			= 1'b1;
					mem_out_sel		= 1'b1;
					alu_in_1_sel	= 1'b0;
					alu_in_2_sel	= 1'b0;
					alu_funk		= `SUB;
					wb_reg_addr		= rd;
				end
				`SUBU: 	begin
					$display("%h\t\t subu %d, %d, %d", DX_IR, rd, rs, rt);
					wb_we 			= 1'b1;
					mem_out_sel		= 1'b1;
					alu_in_1_sel	= 1'b0;
					alu_in_2_sel	= 1'b0;
					alu_funk		= `SUB;
					wb_reg_addr		= rd;
				end
				`ADD: 	begin
					$display("%h\t\t add %d, %d, %d", DX_IR, rd, rs, rt);
					wb_we 			= 1'b1;
					mem_out_sel  	= 1'b1;
					alu_in_1_sel 	= 1'b0;
					alu_in_2_sel	= 1'b0;
					alu_funk		= `ADD;
					wb_reg_addr 	= rd;					
				end
				`ADDU:	begin 
					$display("%h\t\t addu %d, %d, %d", DX_IR, rd, rs, rt);
					wb_we 			= 1'b1;
					mem_out_sel  	= 1'b1;
					alu_in_1_sel 	= 1'b0;
					alu_in_2_sel	= 1'b0;
					alu_funk		= `ADD;
					wb_reg_addr 	= rd;					
				end
				`AND: $display("%h\t\t and %d, %d, %d", DX_IR, rd, rs, rt);
				`OR: $display("%h\t\t or %d, %d, %d", DX_IR, rd, rs, rt);
				`XOR: $display("%h\t\t xor %d, %d, %d", DX_IR, rd, rs, rt);
				`NOR: $display("%h\t\t nor, %d, %d, %d", DX_IR, rd, rs, rt);
				`JR:	begin
					$display("%h\t\t jr %d", DX_IR, rs);
					wb_we 			= 1'b0;
					mem_out_sel		= 1'b1;
					alu_in_1_sel 	= 1'b0;
					alu_in_1_sel	= 1'b0;
					alu_funk		= `JR;
				end
				default: $display("Instruction not found!");
			endcase
		`SPECIAL2:
			case(func)
				`MUL: 	begin
					$display("%h\t\t mul %d, %d, %d", DX_IR, rd, rs, rt);
					wb_we			= 1'b1;
					mem_out_sel		= 1'b1;
					alu_in_1_sel	= 1'b0;
					alu_in_2_sel	= 1'b0;
					alu_funk		= `MUL;
					wb_reg_addr		= rd;
				end
				default: $display("Instruction not found!");
			endcase
		`ADDI: 	begin
			$display("%h\t\t addi %d, %d, %d", DX_IR, rt, rs, s_immed);
			wb_we 			= 1'b1;
			mem_out_sel  	= 1'b1;
			alu_in_1_sel 	= 1'b0;
			alu_in_2_sel	= 1'b1;
			alu_funk		= `ADD;
			wb_reg_addr 	= rt;		
		end
		`ADDIU: begin
			$display("%h\t\t addiu %d, %d, %d", DX_IR, rt, rs, s_immed);
			wb_we 			= 1'b1;
			mem_out_sel  	= 1'b1;
			alu_in_1_sel 	= 1'b0;
			alu_in_2_sel	= 1'b1;
			alu_funk		= `ADD;
			wb_reg_addr 	= rt;
		end
		`ANDI: $display("%h\t\t andi %d, %d, %d", DX_IR, rt, rs, s_immed);
		`ORI: $display("%h\t\t ori %d, %d, %d", DX_IR, rt, rs, s_immed);

		`REGIMM: 
			case (rt)
				`BGEZ: $display("%h\t\t bgez %d, %h", DX_IR, rs, s_immed);
				`BGEZAL: $display("%h\t\t bgezal %d, %h", DX_IR, rs, s_immed);
				`BGEZALL: $display("%h\t\t bgezall %d, %h", DX_IR, rs, s_immed);
				`BGEZL: $display("%h\t\t bgezl %d, %h", DX_IR, rs, s_immed);
				`BLTZ: $display("%h\t\t bltz %d, %h", DX_IR, rs, s_immed);
				`BLTZAL: $display("%h\t\t bltzal %d, %h", DX_IR, rs, s_immed);
				`BLTZALL: $display("%h\t\t bltzall %d, %h", DX_IR, rs, s_immed);
				`BLTZL: $display("%h\t bltzl %d, %h", DX_IR, rs, s_immed);
				default: $display("Instrction not found!");
			endcase
		`BEQ:	begin 
			$display("%h\t\t beq %d, %d, %h", DX_IR, rs, rt, s_immed);
			wb_we 			= 1'b0;
			mem_out_sel 	= 1'b1;
			alu_in_1_sel 	= 1'b0;
			alu_in_2_sel	= 1'b0;
		end
		`BEQL: $display("%h\t\t beql %d, %d, %h", DX_IR, rs, rt, s_immed);
		`BGTZ: $display("%h\t\t bgtz %d, %h", DX_IR, rs, s_immed);
		`BGTZL: $display("%h\t\t bgtzl %d, %h", DX_IR, rs, s_immed);
		`BLEZ: $display("%h\t\t blez %d, %h", DX_IR, rs, s_immed);
		`BLEZL: $display("%h\t\t blezl %d, %h", DX_IR, rs, s_immed);
		`BNE: 	begin
			$display("%h\t\t bne %d, %d, %h", DX_IR, rs, rt, s_immed);
			wb_we			= 1'b0;
			mem_out_sel		= 1'b1;
		end
		`BNEL: $display("%h\t\t bnel %d, %d, %h", DX_IR, rs, rt, s_immed);
		`J: 	begin
			$display("%h\t\t j %h", DX_IR, {rs, rt, s_immed});
			wb_we 		= 1'b0;
			mem_out_sel	= 1'b1;
		end
		`JAL: 	begin
			$display("%h\t\t jal %h", DX_IR, {rs, rt, s_immed});
			wb_we			= 1'b1;
			mem_out_sel		= 1'b1;
			wb_reg_addr		= 31;
			d1_xm_o			= pc_in + 8;
			xm_o_sel		= 1'b1;
		end
		
		`LB: 	begin
			$display("%h\t\t lb %d, %d(%d)", DX_IR, rt, s_immed, rs);
			wb_we 			= 1'b1;
			mem_out_sel  	= 1'b0;
			mem_access_size	= 2'b00;
			mem_rw			= 1'b1;
			alu_in_1_sel 	= 1'b0;
			alu_in_2_sel	= 1'b1;
			alu_funk		= `ADD;
			wb_reg_addr 	= rt;
		end
		`LBU: 	begin
			$display("%h\t\t lbu %d, %d(%d)", DX_IR, rt, s_immed, rs);
			wb_we 			= 1'b1;
			mem_out_sel  	= 1'b0;
			mem_access_size	= 2'b00;
			mem_rw			= 1'b1;
			alu_in_1_sel 	= 1'b0;
			alu_in_2_sel	= 1'b1;
			alu_funk		= `ADD;
			wb_reg_addr 	= rt;
		end
		`LUI: 	begin
			$display("%h\t\t lui %d, %d", DX_IR, rt, immed);
			wb_we			= 1'b1;
			mem_out_sel		= 1'b1;
			wb_reg_addr		= rt;
			d1_xm_o			= (immed << 16) & 32'hFFFF0000;
			xm_o_sel		= 1'b1;					
		end
		`LW:	begin
			$display("%h\t\t lw %d, %d(%d)", DX_IR, rt, s_immed, rs);
			wb_we 			= 1'b1;
			mem_out_sel  	= 1'b0;
			mem_access_size	= 2'b10;
			mem_rw			= 1'b1;
			alu_in_1_sel 	= 1'b0;
			alu_in_2_sel	= 1'b1;
			alu_funk		= `ADD;
			wb_reg_addr 	= rt;
		end
		`SB: 	begin
			$display("%h\t\t sb %d, %d(%d)", DX_IR, rt, s_immed, rs);
			wb_we			= 1'b0;
			mem_out_sel		= 1'b0;
			mem_access_size	= 2'b00;
			mem_rw			= 1'b0;
			alu_in_1_sel	= 1'b1;
			alu_in_2_sel	= 1'b1;
			alu_funk		= `ADD;
			wb_reg_addr		= rt;		
		end
		`SLTI: 	begin
			$display("%h\t\t slti %d, %d, %d", DX_IR, rt, rs, s_immed);
			wb_we 			= 1'b1;
			mem_out_sel 	= 1'b1;
			alu_in_1_sel	= 1'b0;
			alu_in_2_sel	= 1'b1;
			alu_funk		= `SLT;
			wb_reg_addr 	= rt;			
		end
		`SLTIU: $display("%h\t\t sltiu %d, %d, %d", DX_IR, rt, rs, s_immed);
		`SW:	begin
			$display("%h\t\t sw %d, %d(%d)", DX_IR, rt, s_immed, rs);
			wb_we 			= 1'b0;
			mem_out_sel  	= 1'b0;
			mem_access_size	= 2'b10;
			mem_rw			= 1'b0;
			alu_in_1_sel 	= 1'b0;
			alu_in_2_sel	= 1'b1;
			alu_funk		= `ADD;
			wb_reg_addr 	= rt;
		end
		default: $display("Instruction not found!");
	endcase
end

endmodule
