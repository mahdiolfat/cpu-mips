`include "const.v"

/**** MEMORY MODULE ****/
module memory_0 (clk, address, data_in, access_size, rw, FD_IR);

// Port declarations
input			clk; 
input	[31:0]	address; 
input	[31:0]	data_in; 
input	[1:0]	access_size; 
input			rw;
output	[31:0]	FD_IR;

// Port type declarations
reg 	[31:0] 	data_out;

reg 	[7:0] 	mem [0:`MEM_DEPTH];
reg 	[31:0] 	addr_i;	// indexed memory address
reg		[31:0]	FD_IR;

// Register memory output to F/D pipeline register
always @ (data_out) begin
	FD_IR = data_out;
end

reg			rw_reg;
reg	[31:0]	data_reg;
reg	[1:0]	access_size_reg;	

// Memory model
always @ (posedge clk) begin
	// Memory address translation
	addr_i = address - `ADDR_START;
	// Read/Write at negedge 
	rw_reg = rw;
	data_reg = data_in;
	access_size_reg = access_size;
end

// Read/Write at negative clock edge
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
