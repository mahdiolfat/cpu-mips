`include "const.v"

/**** EXECUTE MODULE ****/
module exec(
clk_in, stall_in, 
DX_A, DX_B, DX_PC, DX_IR, XM_O, XM_B, XM_IR,
mem_address, mem_data_in, mem_access_size_in, 
mem_access_size_out, mem_rw_in, mem_rw_out,
mem_out_sel_in, mem_out_sel_out,
wb_we_in, wb_we_out, wb_reg_addr_in, wb_reg_addr_out,
pc_sel, pc_out,
alu_funk, alu_in_1_sel, alu_in_2_sel, s_immed,
xm_o_sel, d1_xm_o_in,
wb_data_bypass
);

// CPU clock
input			clk_in;
input			stall_in;
// Dictates whether the reg file will be written
input			wb_we_in;
// The address of the register to write to
input 	[4:0]	wb_reg_addr_in;
// Memory access size
input	[1:0]	mem_access_size_in;
// Memory rw signal 
input			mem_rw_in;
// The mux select line for first ALU input
input			alu_in_1_sel;
// The mux select line for second ALU input
input			alu_in_2_sel;

// Input pipeline registers
input			[31:0]	DX_A;
input			[31:0]	DX_B;
input			[31:0]	DX_PC;
input			[31:0]	DX_IR;
input signed	[31:0]	s_immed;

// The mux select line for the memory stage output
// Chooses between ALU output and memory output
input			mem_out_sel_in;
// The mux select line for execute stage output
// Chooses between ALU output and PC+8 (for the JAL insn)
input			xm_o_sel;
// This is used as a MUX input for the execute to
// memory stage output. It's used for writing special values
// to a register in the wb stage (e.g, for LUI and JAL insns)
input	[31:0]	d1_xm_o_in;
// WX BYPASS value of most-up-to-date register
input	[31:0]	wb_data_bypass;

// Output pipeline register
output	[31:0]	XM_O;
output	[31:0]	XM_B;
output	[31:0]	XM_IR;

// Registered control signal outputs
output			wb_we_out;
output	[4:0]	wb_reg_addr_out;

// MUX select signal for PC
// Set depending on branch taken/not taken.
output			pc_sel;
// New PC set on a taken branch
output	[31:0]	pc_out;

// Memory signals
output	[31:0]	mem_address;
output	[31:0]	mem_data_in;
output	[1:0]	mem_access_size_out;
output			mem_rw_out;
output			mem_out_sel_out;

reg				pc_sel;
reg		[31:0]	pc_out;

reg		[31:0]	XM_B;
reg		[31:0]	XM_IR;
reg		[31:0]	XM_O;

reg				wb_we_out;
reg		[4:0]	wb_reg_addr_out;

reg				mem_out_sel_out;
reg		[1:0]	mem_access_size_out;
reg				mem_rw_out;
reg		[31:0]	mem_data_in;

/********* THE ALU *********/
// Operation of the ALU
input 		[5:0]	alu_funk;
reg			[5:0]	alu_funk_reg;
reg 		[31:0]	alu_in_1_reg;
reg 		[31:0]	alu_in_2_reg;
reg 		[31:0]	alu_in_1;
reg signed 	[31:0]	alu_in_2;
// Output of the ALU
reg			[31:0]	alu_out;
always @ (alu_in_1 or alu_in_2 or alu_funk_reg) begin
	$display("alu_in_1 = %d", alu_in_1);
	$display("alu_in_2 = %d", alu_in_2);
	$display("alu_funk = %b", alu_funk_reg);
	case (alu_funk_reg)
		`SLL: 	alu_out = alu_in_2 << alu_in_1;
		`SLT: 	alu_out = (alu_in_1 < alu_in_2) ? 1 : 0;
		`SRA: 	alu_out = alu_in_2 >> alu_in_1;
		`SUB: 	alu_out = alu_in_1 - alu_in_2;
		`ADD:	alu_out = alu_in_1 + alu_in_2;
		`AND:	alu_out = alu_in_1 & alu_in_2;
		`OR:	alu_out = alu_in_1 | alu_in_2;
		`XOR:	alu_out = alu_in_1 ^ alu_in_2;
		`NOR:	alu_out = alu_in_1 ~^ alu_in_2;
		`MUL:	alu_out = alu_in_1 * alu_in_2;
		`JR:	alu_out = alu_in_1;
		default: alu_out = 32'hxxxxxxxx;
	endcase
	$display("%g alu_out = %d", $time, alu_out); 
end
/********* END OF ALUUUUU *********/

/***** MUX *****/
/* 
This MUX is used for input 1 of the ALU
 - (sel == 1'b0) output of bypass MUX for input 1 of ALU
 - (sel == 1'b1) sh (DX_IR[10:6]) portion of the instruction
*/
reg			[31:0]	alu_in_1_d0;
wire		[31:0]	alu_in_1_d1;
assign alu_in_1_d1 = DX_IR[`SH_e:`SH_s];
always @ (alu_in_1_sel or alu_in_1_d0 or alu_in_1_d1) begin
	if (alu_in_1_sel == 1'b0) begin
		alu_in_1_reg = alu_in_1_d0;
	end else begin
		alu_in_1_reg = alu_in_1_d1;
	end
end
/***** END OF MUX *****/

/***** MUX *****/
/* 
This MUX is used for input 2 of the ALU
 - (sel == 1'b0) output of bypass MUX for input 2 of ALU
 - (sel == 1'b1) sign-extended immediate portion of the instruction
*/
// MUX for input 2 of the ALU
reg	 	[31:0]	alu_in_2_d0;
wire	[31:0]	alu_in_2_d1;
assign alu_in_2_d1 = s_immed;
always @ (alu_in_2_sel or alu_in_2_d0 or alu_in_2_d1) begin
	if (alu_in_2_sel == 1'b0) begin
		alu_in_2_reg = alu_in_2_d0;
	end else begin
		alu_in_2_reg = alu_in_2_d1;
	end
end
/***** END OF MUX *****/

/***** MUX *****/
/*
This MUX is used for setting X/M output register value
It is used to choose from ALU output and value to write to a register
 - (sel == 1'b0) ALU output
 - (sel == 1'b1) special value to write to register (e.h., LUI and JAL insns)
*/
wire	[31:0]	xm_o_d0;
wire	[31:0]	xm_o_d1;
assign xm_o_d0 = alu_out;
assign xm_o_d1 = d1_xm_o_in;
always @ (xm_o_sel or xm_o_d0 or xm_o_d1) begin
	if (xm_o_sel == 1'b0) begin
		XM_O = xm_o_d0;
	end else begin
		XM_O = xm_o_d1;
	end
end
/***** END OF MUX *****/

/***** MUX *****/
/*
This MUX is used to set the new PC value 
depending on the branch/jump instruction
*/
reg		[1:0]	pc_out_sel;
wire	[31:0]	pc_out_d0;
wire	[31:0]	pc_out_d1;
wire	[31:0]	pc_out_d2;
assign	pc_out_d0 = (DX_PC[31:28] << 28) | (DX_IR[25:0] << 2);
assign 	pc_out_d1 = (DX_PC + 4) + (s_immed << 2);
assign 	pc_out_d2 = DX_A;
always @ (pc_out_sel or pc_out_d0 or pc_out_d1 or pc_out_d2) begin
	case (pc_out_sel)
		2'b00: 	 pc_out = pc_out_d0;
		2'b01: 	 pc_out = pc_out_d1;
		2'b10: 	 pc_out = pc_out_d2;
		default: pc_out = 32'hxxxxxxxxx;
	endcase
end
/***** END OF MUX *****/

/***** BYPASS MUX *****/
/*
This MUX is used for forwarding (bypass) to input 1 of the ALU
Forwarding is from Memory and WriteBack stage
The select signal is set in the cpu.v module
*/
reg		[1:0]	bypass_alu_in_1_sel;
wire	[31:0]	alu_in_1_b0;
wire	[31:0]	alu_in_1_b1;
wire	[31:0]	alu_in_1_b2;
assign alu_in_1_b0 = DX_A;
assign alu_in_1_b1 = alu_out;
assign alu_in_1_b2 = wb_data_bypass;

always @ (bypass_alu_in_1_sel or 
		  alu_in_1_b0 or alu_in_1_b1 or alu_in_1_b2)
begin
	case (bypass_alu_in_1_sel)
		2'b00: alu_in_1_d0 		= alu_in_1_b0;
		2'b01: alu_in_1_d0 		= alu_in_1_b1;
		2'b10: alu_in_1_d0 		= alu_in_1_b2;
		default: alu_in_1_d0 	= 32'hxxxxxxxx;
	endcase
end
/***** END OF MUX *****/

/***** BYPASS MUX *****/
/*
This MUX is used for forwarding (bypass) to input 2 of the ALU
Forwarding is from Memory and WriteBack stage
The select signal is set in the cpu.v module
*/
reg		[1:0]	bypass_alu_in_2_sel;
wire	[31:0]	alu_in_2_b0;
wire	[31:0]	alu_in_2_b1;
wire	[31:0]	alu_in_2_b2;
assign alu_in_2_b0 = DX_B;
assign alu_in_2_b1 = alu_out;
assign alu_in_2_b2 = wb_data_bypass;

always @ (bypass_alu_in_2_sel or
		  alu_in_2_b0 or alu_in_2_b1 or alu_in_2_b2) 
begin
	case (bypass_alu_in_2_sel)
		2'b00: alu_in_2_d0 		= alu_in_2_b0;
		2'b01: alu_in_2_d0 		= alu_in_2_b1;
		2'b10: alu_in_2_d0 		= alu_in_2_b2;
		default: alu_in_2_d0 	= 32'hxxxxxxxx;
	endcase
end
/***** END OF MUX *****/

// Address supplied to memory is always the ALU output
assign mem_address  = alu_out;

// Set initial values for the execute stage
initial begin
	pc_sel = 1'b0;
	bypass_alu_in_1_sel = 2'b00;
	bypass_alu_in_2_sel = 2'b00;
end

// Register the outputs to memory stage
always @ (posedge clk_in) begin
	XM_B  				<= DX_B;
	XM_IR 				<= DX_IR;
	mem_data_in 		<= DX_B;
	wb_we_out 			<= wb_we_in;
	mem_out_sel_out		<= mem_out_sel_in;
	wb_reg_addr_out		<= wb_reg_addr_in;
	mem_access_size_out	<= mem_access_size_in;
	mem_rw_out			<= mem_rw_in;
	alu_funk_reg		<= alu_funk;
	alu_in_1			<= alu_in_1_reg;
	alu_in_2			<= alu_in_2_reg;
end

always @ (posedge clk_in) begin
	if (stall_in == 1'b0) begin
		pc_sel = 1'b0;
		$display("TIME: %g EXECUTE PC = %h", $time, DX_PC);
		
		case (DX_IR[31:26])
			`SPECIAL:
				case (DX_IR[5:0])
					`JR:	begin
						pc_out_sel	= 2'b10;
						pc_sel 		= 1'b1;
					end
				endcase
			`BEQ:	begin
				if (DX_A == DX_B) begin
					$display("Branch taken");
					pc_out_sel	= 2'b01;
					pc_sel = 1'b1;
				end else begin
					$display("Branch NOT taken");
					pc_sel = 1'b0;
				end
			end
			`J:		begin
				pc_out_sel	= 2'b00;
				pc_sel 		= 1'b1;
			end
			`JAL:	begin
				pc_out_sel	= 2'b00;
				pc_sel		= 1'b1;
			end
			`BNE:	begin
				if (DX_A != DX_B) begin
					$display("Branch taken");
					pc_out_sel	= 2'b01;
					pc_sel 		= 1'b1;
				end else begin 
					$display("Branch not taken");
					pc_sel = 1'b0;
				end
			end
		endcase
	end
end
endmodule
