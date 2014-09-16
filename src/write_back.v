module write_back (
clk, stall_in, 
MW_IR, 
write_data_in, write_data_out, 
write_addr_in, write_addr_out, 
wb_we_in, rf_we
);

// CPU clock
input			clk;
input			stall_in;

// Pipeline register
input	[31:0]	MW_IR;

// Destination register address and value
input	[4:0]	write_addr_in;
input	[31:0]	write_data_in;
// Write enable signal for the register
input			wb_we_in;

output	[4:0]	write_addr_out;
output	[31:0]	write_data_out;
output			rf_we;

reg		[4:0]	write_addr_out;
reg		[31:0]	write_data_out;
reg				rf_we;

initial begin
	rf_we = 1'b0;
end

// Update signals to the register file at positive edge
always @ (posedge clk) begin
	if (stall_in == 1'b0) begin
		$display("TIME: %g WRITE_BACK", $time);
		rf_we		   = wb_we_in;
		write_data_out = write_data_in;
		write_addr_out = write_addr_in;
	end
end

endmodule
