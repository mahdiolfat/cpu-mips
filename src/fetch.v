`include "const.v"

/**** Fetch Module ****/
module fetch (
clk_in, 
stall_in,
pc_out_new,
pc_sel, 
pc_out, 
pc_decode_out,  
rw_out,  
access_size_out
);

// CPU clock
input           clk_in;
// New PC provided by the execute stage upon a taken branch
input 	[31:0]	pc_out_new;
// MUX select signal for PC driven by the execute stage
input			pc_sel;
// Fetch stall signal. If asserted PC is NOT incremented
input           stall_in;

// Transmits the PC to the decode stage
output  [31:0]  pc_decode_out;
// rw signal to memory. Always set to read
output          rw_out;
// access signal to memory. Always set to access a word (4 bytes)
output  [1:0]   access_size_out;
// Supplies the PC to the memory to fetch insn
output  [31:0]  pc_out;

// Internal declarations
reg				rw_out;
reg		[31:0]	pc_out;
reg		[31:0]	pc_decode_out;
reg		[31:0]	pc_decode_out_reg;
reg		[1:0]	access_size_out;

/***** MUX *****/
/* 
This MUX is used to determine the value of the
next PC to fetch the instruction from.
New pc is either:
 - (sel == 1'b0) PC+4 
 - (sel == 1'b1) PC from execute stage for a taken branch 
*/
reg		[31:0]	d0;
wire	[31:0]	d1;
assign d1 = pc_out_new;
always @ (pc_sel or d0 or d1) begin
	if (pc_sel == 1'b0) begin
		pc_out = d0;
	end else begin
		pc_out = d1;
	end
end
/***** END OF MUX *****/

// Set inital values for the fetch stage
initial begin
    pc_out = `ADDR_START;
	d0 = pc_out;
    // Always a read
    rw_out = 1'b1;
    // Always read a word
    access_size_out = 2'b10;
end

// Register the output to the decode stage
always @ (posedge clk_in) begin
	if (stall_in == 1'b0)
		pc_decode_out <= pc_decode_out_reg;
end

// Increment PC by 4 if not stalled
always @ (posedge clk_in) begin
	if (stall_in == 1'b0) begin
		pc_decode_out_reg = pc_out;
		d0 = pc_out + 4;
		$display("TIME: %g FETCH PC = %h", $time, pc_out);
	end
end

endmodule
