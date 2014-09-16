`include "const.v"

module mem_pipe(
clk, stall_in, 
address, data_in, access_size, rw,
XM_B, XM_IR, MW_IR, XM_O, 
wb_reg_addr_in, wb_reg_addr_out,
wb_reg_data_out, wb_we_in, wb_we_out,
sel_out,
wb_data_bypass
);

// CPU clock
input			clk;
input			stall_in;
// Memory address to read/write
input	[31:0]	address;
// Data to write
input	[31:0]	data_in;
// Memory access size
input	[1:0]	access_size;
// Read/Write signal
input			rw;
// MUX select signal for memory stage output to WB stage
input			sel_out;
// Pipeline registers
input	[31:0]	XM_B;
input	[31:0]	XM_O;
input	[31:0]	XM_IR;
// wb control signals
input			wb_we_in;
input	[4:0]	wb_reg_addr_in;
// WM BYPASS value of most-up-to-date register
input 	[31:0]	wb_data_bypass;

// wb control signals
output			wb_we_out;
output	[4:0]	wb_reg_addr_out;
output	[31:0]	wb_reg_data_out;

output	[31:0]	MW_IR;

reg		[31:0]	MW_IR;
reg		[31:0]	data_out;

reg				wb_we_out;
reg		[4:0]	wb_reg_addr_out;

// Memory array
reg		[7:0]	mem[0:`MEM_DEPTH];
reg		[31:0]	addr_i; // indexed memory address

reg		[31:0]	wb_reg_data_out;
reg		[31:0]	wb_reg_data_out_reg;

// Memory model registered values
reg		[31:0]	data;
reg				rw_reg;
reg		[31:0]	data_reg;
reg		[1:0]	access_size_reg;

// Register the outputs
always @ (posedge clk) begin
	MW_IR <= XM_IR;
	wb_we_out <= wb_we_in;
	wb_reg_addr_out <= wb_reg_addr_in;
	wb_reg_data_out <= wb_reg_data_out_reg;
end

/***** MUX *****/
/*
This MUX is used to set the output of the memory stage
This is either the output of memory or the output of
the execute stage.
 - (sel == 1'b0) memory output
 - (sel == 1'b1) execute output
*/
wire	[31:0]	d0;
wire	[31:0]	d1;
assign d0 = data;
assign d1 = XM_O;
always @ (sel_out or d0 or d1) begin
	if (sel_out == 1'b0) begin
		wb_reg_data_out_reg = d0;
	end else begin
		wb_reg_data_out_reg = d1;
	end
end
/***** END OF MUX *****/

/***** BYPASS MUX *****/
/*
This MUX is used for forwarding (bypass) to data_in of memory
Forwarding is from WriteBack to Memory stage
The select signal is set in the cpu.v module
*/
reg				data_sel;
wire	[31:0]	data_b0;
wire	[31:0]	data_b1;
assign data_b0 = data_in;
assign data_b1 = wb_data_bypass;

always @ (data_sel or data_b0 or data_b1) begin
	if (data_sel == 1'b0) begin
		data = data_b0;
	end else begin
		data = data_b1;
	end
end
/***** END OF MUX *****/

// Set initial values for the memory stage
initial begin
	for (addr_i = 0; addr_i < `MEM_DEPTH; addr_i = addr_i + 1) begin
		mem[addr_i] = 8'hFF;
	end
	data_sel = 1'b0;
end	

// Memory model
always @ (posedge clk) begin
	if (stall_in == 1'b0) begin
		addr_i 			= address - `ADDR_START;
		rw_reg 			= rw;
		data_reg 		= data;
		access_size_reg = access_size;
	end
end

// Read/Write at the negative clock edge
always @ (negedge clk) begin
	if (rw_reg === 1'b1) begin
		// read from memory
		case (access_size_reg)
			2'b00: data_out	= mem[addr_i];
			2'b01: data_out = {mem[addr_i], mem[addr_i+1]};
			2'b10: data_out = {mem[addr_i], mem[addr_i+1], 
						mem[addr_i+2], mem[addr_i+3]};
			default: data_out = 32'hxxxxxxxx;
		endcase
	end else begin
		// write to memory
		case (access_size_reg)
			2'b00: mem[addr_i] = data_reg[7:0];
			2'b01: begin
				// Big-Endian
				mem[addr_i] = data_reg[15:8];
				mem[addr_i+1] = data_reg[7:0];
				end
			2'b10: begin
				// Big-Endian
				mem[addr_i] = data_reg[31:24];
				mem[addr_i+1] = data_reg[23:16];
				mem[addr_i+2] = data_reg[15:8];
				mem[addr_i+3] = data_reg[7:0];
				end
			default: data_out = 32'hxxxxxxxx;
		endcase
	end
end
endmodule
