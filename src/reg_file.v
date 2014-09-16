`include "const.v"

module reg_file (
clk_in, 
stall_in, 
rs_in, rt_in, 
rd_in, 
we, write_data, 
rs_out, rt_out
);

// CPU clock
input			clk_in;
input			stall_in;
// Source 1 register address
input	[4:0]	rs_in;
// Source 2 register address
input	[4:0]	rt_in;
// Destination register address
input	[4:0]	rd_in;
// Write enable signal
input			we;
// Data to write to desitnation register
input	[31:0]	write_data;

// Value of source 1 register
output	[31:0]	rs_out;
// Value of source 2 register
output	[31:0]	rt_out;

// MIPS architectural resister
reg		[31:0]	regs [0:31];

reg		[31:0]	rs_out;
reg		[31:0]	rt_out;
reg				we_reg;

always @ (clk_in or rs_in or rt_in or rd_in or we or write_data) begin
	we_reg = we;
	if (stall_in == 1'b0) begin
		//$display("TIME: %g REG FILE", $time);
		rs_out <= regs[rs_in];
		rt_out <= regs[rt_in];
	end
end

always @ (negedge clk_in) begin
	if (stall_in == 1'b0) begin
		if (we_reg) begin 
			regs[rd_in] <= write_data;
			$display("TIME: %g WRITE Reg %d = %d", $time, rd_in, write_data);
		end	
	end
end 

// Iterator for initializing architectural registers
reg		[31:0]	i;
initial begin
	// initial all register values
	for (i = 0; i < 32; i = i + 1) begin
		regs[i] = i;
	end
	// Assign the stack pointer to the highest byte in memory
	// Stack grows down
	regs[29] = `ADDR_START + `MEM_DEPTH;
end

endmodule
