`include "memory_0.v"
`include "memory_1.v"
`include "fetch.v"
`include "decode.v"
`include "reg_file.v"
`include "execute.v"
`include "write_back.v"

module cpu;

// Internal decalarations for interacting with memory module
reg 		clk;
reg  [31:0] address;
reg  [31:0] data_in;
reg  [1:0] 	access_size;
reg 		rw;

// SREC Parser decalarations
integer srecFile;			// SREC file descriptor 
integer srecCnt;			// Number of bytes in a SREC data sequence
integer i;					// Iterator (used for for-loops)

integer writeCnt = 0;		// Count mumber of bytes written to memory 

reg [31:0]	srecAddr;		// Address read from SREC file
reg [8:0] 	srecChar;		// Byte read from SREC file
reg [7:0] 	c1, c2;			// 2-ascii characters read from SREC file
reg [3:0] 	LUT [48:70];	// Look-up-table for ASCII -> VALUE translation
reg 		doneParsing;	// Bit to indicate completion of SREC file parsing

// For the test cases assume that no more than 1K of data bytes
// This is more than enough for the sample SREC files
parameter 	MAX_DATA_CNT = 1024;
reg [31:0] 	addr_arr [0:MAX_DATA_CNT];
reg [7:0] 	data_arr [0:MAX_DATA_CNT];	// data written to memory

// Stall signals
reg			stall_f;
reg			stall_rf;
reg			stall_ex;
reg			stall_mem_1;
reg			stall_wb;

/***** MEMORY 0 MUX *****/
/*
Chooses between the parser and the fetch stage
*/
reg			sel;
wire 		d0_rw_0;
wire [31:0]	d0_address_0;
wire [1:0]	d0_access_size_0;
wire 		d1_rw_0;
wire [31:0]	d1_address_0;
wire [1:0]	d1_access_size_0;
reg 		mem_0_rw;
reg	[31:0]	mem_0_address;
reg	[1:0]	mem_0_access_size;

assign d0_rw_0 			= rw;
assign d0_address_0 	= address;
assign d0_access_size_0 = access_size;

always @ (sel or 
		  d0_address_0 or d0_rw_0 or d0_access_size_0 or 
		  d1_address_0 or d1_rw_0 or d1_access_size_0) begin
	if (sel == 1'b0) begin
		mem_0_address 		= d0_address_0;
		mem_0_rw 			= d0_rw_0;
		mem_0_access_size 	= d0_access_size_0;
	end else begin
		mem_0_address 		= d1_address_0;
		mem_0_rw 			= d1_rw_0;
		mem_0_access_size 	= d1_access_size_0;		
	end
end
/***** END OF MUX *****/

/***** MEMORY 1 MUX *****/
wire 		d0_rw_1;
wire [31:0]	d0_address_1;
wire [1:0]	d0_access_size_1;
wire [31:0]	d0_data_in_1;

wire 		d1_rw_1;
wire [31:0]	d1_address_1;
wire [1:0]	d1_access_size_1;
wire [31:0]	d1_data_in_1;

reg 		mem_1_rw;
reg	[31:0]	mem_1_address;
reg	[1:0]	mem_1_access_size;
reg	[31:0]	mem_1_data_in;		

assign d0_rw_1 			= rw;
assign d0_address_1 	= address;
assign d0_access_size_1 = access_size;
assign d0_data_in_1		= data_in;	

always @ (sel or 
		  d0_address_1 or d0_rw_1 or d0_access_size_1 or d0_data_in_1 or 
		  d1_address_1 or d1_rw_1 or d1_access_size_1 or d1_data_in_1) begin
	if (sel == 1'b0) begin
		mem_1_address 		= d0_address_1;
		mem_1_rw 			= d0_rw_1;
		mem_1_access_size 	= d0_access_size_1;
		mem_1_data_in		= d0_data_in_1;
	end else begin
		mem_1_address 		= d1_address_1;
		mem_1_rw 			= d1_rw_1;
		mem_1_access_size 	= d1_access_size_1;	
		mem_1_data_in		= d1_data_in_1;
	end
end
/***** END OF MUX *****/

/* CLOCK GENERATOR */
always #1 clk = ~clk;

// Main testbench block
// Waits for parser to finish parsin SREC file
// Write data to both memories and starts CPU
initial begin
	clk = 0;
	access_size = 2'b00;
	doneParsing = 1'b0;
	
	stall_f 		= 1'b1;
	stall_rf		= 1'b1;
	stall_ex		= 1'b1;
	stall_mem_1		= 1'b0;
	stall_wb		= 1'b1;
		
	sel = 1'b0;

	// Wait until SREC file has been parsed
	@ (doneParsing) begin
		for (i = 0; i < writeCnt; i = i + 1) begin
			@ (posedge clk) begin
				rw = 1'b0;
				address = addr_arr[i];
				data_in = data_arr[i];
			end
			@ (posedge clk) begin
				rw = 1'b1;
			end
			@ (posedge clk);
			@ (posedge clk) begin
				if (data_in[7:0] !== mem_0.data_out[7:0]) begin
					$display("MEM 0: Data written did not match data read!!!");
					$finish;
				end
				if (data_in[7:0] !== mem_1.data_out[7:0]) begin
					$display("MEM 1: Data written did not match data read!!!");
					$finish;
				end
			end
		end
	end
	
	$display("%g Memory written with instructions!\n", $time);
	
	// Parsing and writing instructions to memory is complete
	// Start cpu
	sel = 1'b1;
	stall_mem_1 		= 1'b1;

	$display("Initial Register Content");
	for (i = 0; i < 32; i = i + 1) begin
		$display("Register %d = %d (0x%h)", i, rf.regs[i], rf.regs[i]);
	end
	
	$display("");
	
	// Activate all stages
	stall_f 		= 1'b0;
	// These signals will not be used during processor execution
	stall_rf		= 1'b0;
	stall_ex		= 1'b0;
	stall_mem_1		= 1'b0;
	stall_wb		= 1'b0;
end

// SREC parser block
initial begin
	$display("Test SimpleAdd.srec sample file...");
	srecFile = $fopen("SampleBench429/SimpleAdd.srec", "r");
	if (!srecFile) begin
		$display("Could not open srec file!");
		$finish;
	end else begin
		$display("Opened SREC file!");
	end

	// Initialize ASCII->VALUE look-up-table
	LUT['h30] = 4'h0;
	LUT['h31] = 4'h1;
	LUT['h32] = 4'h2;
	LUT['h33] = 4'h3;
	LUT['h34] = 4'h4;
	LUT['h35] = 4'h5;
	LUT['h36] = 4'h6;
	LUT['h37] = 4'h7;
	LUT['h38] = 4'h8;	
	LUT['h39] = 4'h9;
	LUT['h41] = 4'hA;
	LUT['h42] = 4'hB;
	LUT['h43] = 4'hC;	
	LUT['h44] = 4'hD;
	LUT['h45] = 4'hE;
	LUT['h46] = 4'hF;
	
	// Loop each line until EOF (0x1FF)
	srecChar = $fgetc(srecFile);
	while (srecChar != 'h1ff) begin
		// S (0x53) indicates beginning of line
		if (srecChar != 'h53) begin
			$display("Not a valid SREC file");
			$fclose(srecFile);
			$finish;
		// Parse line
		end else begin
			srecChar = $fgetc(srecFile);
			// Looking for a 3 (0x33) after S for data sequence
			if (srecChar != 'h33) begin
				// Loop until an S is detected
				while (srecChar != 'hA) begin
					srecChar = $fgetc(srecFile);
				end
				srecChar = $fgetc(srecFile);
			end else begin
				// S3 was detected, read out address, size, and data to write
				c1 = $fgetc(srecFile);
				c2 = $fgetc(srecFile);
				// subtract size of address and checksum (5 bytes in total) from srecCnt
				// to retrive number of data bytes
				srecCnt = {LUT[c1], LUT[c2]} - 5;

				// Retrieve starting address for this data sequence
				c1 = $fgetc(srecFile);
				c2 = $fgetc(srecFile);
				srecAddr[31:24] = {LUT[c1], LUT[c2]};
				c1 = $fgetc(srecFile);
				c2 = $fgetc(srecFile);
				srecAddr[23:16] = {LUT[c1], LUT[c2]};
				c1 = $fgetc(srecFile);
				c2 = $fgetc(srecFile);
				srecAddr[15:8] = {LUT[c1], LUT[c2]};
				c1 = $fgetc(srecFile);
				c2 = $fgetc(srecFile);
				srecAddr[7:0] = {LUT[c1], LUT[c2]};
				
				// Read all data bytes in this data sequence
				repeat (srecCnt) begin
					c1 = $fgetc(srecFile);
					c2 = $fgetc(srecFile);
					data_arr[writeCnt] = {LUT[c1], LUT[c2]};
					addr_arr[writeCnt] = srecAddr;
					srecAddr = srecAddr + 1;
					writeCnt = writeCnt + 1;
				end
				
				// Skip over the checksum byte
				srecChar = $fgetc(srecFile);
				// Look for end of line char (0xA)
				while (srecChar != 'hA) begin
					srecChar = $fgetc(srecFile);
				end
				srecChar = $fgetc(srecFile);
			end
		end
	end
	$display("Parsed entire file successfully");
	doneParsing = 1'b1;
end

/* CONTROL LOGIC FOR FORWARDING, STALLS, AND END OF PROGRAM */
always @(posedge clk) begin
	ex.bypass_alu_in_1_sel 	 = 2'b00;
	ex.bypass_alu_in_2_sel 	 = 2'b00;
	mem_1.data_sel			 = 1'b0;
	if (sel == 1'b1) stall_f = 1'b0;
	d.nop_sel = 1'b0;
	
	if ((ex.DX_IR[`OP_e:`OP_s] == `SPECIAL) || 
		(ex.DX_IR[`OP_e:`OP_s] == `SPECIAL2)) 
	begin
		// rd(X) == rs(D) for REG-REG insn1
		if (ex.DX_IR[`RD_e:`RD_s] == d.insn_in[`RS_e:`RS_s]) begin
			ex.bypass_alu_in_1_sel = 2'b01;
		end
		
		if ((d.insn_in[`OP_e:`OP_s] == `SPECIAL) || 
			(d.insn_in[`OP_e:`OP_s] == `SPECIAL2)) 
		begin	
			// rd(X) == rt(D) for REG-REG insn2
			if (ex.DX_IR[`RD_e:`RD_s] == d.insn_in[`RT_e:`RT_s]) begin
				ex.bypass_alu_in_2_sel = 2'b01;
			end
		end
	end
	else
	begin
		if (ex.DX_IR[`RT_e:`RT_s] == d.insn_in[`RS_e:`RS_s]) begin
			ex.bypass_alu_in_1_sel = 2'b01;
		end
		
		if ((d.insn_in[`OP_e:`OP_s] == `SPECIAL) || 
			(d.insn_in[`OP_e:`OP_s] == `SPECIAL2)) 
		begin
			// rt(X) == rt(D) for REG-REG insn2
			if (ex.DX_IR[`RT_e:`RT_s] == d.insn_in[`RT_e:`RT_s]) begin
				ex.bypass_alu_in_2_sel = 2'b01;
			end			
		end
	end
	
	if ((mem_1.XM_IR[`OP_e:`OP_s] == `SPECIAL) || 
		(mem_1.XM_IR[`OP_e:`OP_s] == `SPECIAL2)) 
	begin
		// rd(X) == rs(D) for REG-REG insn1
		if (mem_1.XM_IR[`RD_e:`RD_s] == d.insn_in[`RS_e:`RS_s]) begin
			ex.bypass_alu_in_1_sel = 2'b10;
		end
		
		if (ex.DX_IR[`OP_e:`OP_s] == `SW) begin
			if (mem_1.XM_IR[`RD_e:`RD_s] == ex.DX_IR[`RT_e:`RT_s]) begin
				mem_1.data_sel = 1'b1;
			end
		end
		
		if ((d.insn_in[`OP_e:`OP_s] == `SPECIAL) || 
			(d.insn_in[`OP_e:`OP_s] == `SPECIAL2)) 
		begin	
			// rd(X) == rt(D) for REG-REG insn2
			if (mem_1.XM_IR[`RD_e:`RD_s] == d.insn_in[`RT_e:`RT_s]) begin
				ex.bypass_alu_in_2_sel = 2'b10;
			end
		end
	end
	else
	begin
		if (mem_1.XM_IR[`RT_e:`RT_s] == d.insn_in[`RS_e:`RS_s]) begin
			ex.bypass_alu_in_1_sel = 2'b10;
		end

		if (ex.DX_IR[`OP_e:`OP_s] == `SW) begin
			if (mem_1.XM_IR[`RT_e:`RT_s] == ex.DX_IR[`RT_e:`RT_s]) begin
				mem_1.data_sel = 1'b1;
			end
		end
		
		if ((d.insn_in[`OP_e:`OP_s] == `SPECIAL) || 
			(d.insn_in[`OP_e:`OP_s] == `SPECIAL2)) 
		begin
			if (mem_1.XM_IR[`RT_e:`RT_s] == d.insn_in[`RT_e:`RT_s]) begin
				ex.bypass_alu_in_2_sel = 2'b10;
			end			
		end
	end

	// Determine if a NOP is needed
	if (ex.DX_IR[`OP_e:`OP_s] == `LW)
	begin
		if ((d.insn_in[`OP_e:`OP_s] == `SPECIAL) || 
			(d.insn_in[`OP_e:`OP_s] == `SPECIAL2)) 
		begin 
			if ((ex.DX_IR[`RT_e:`RT_s] == d.insn_in[`RS_e:`RS_s]) ||
				(ex.DX_IR[`RT_e:`RT_s] == d.insn_in[`RT_e:`RT_s])) 
			begin
				stall_f = 1'b1;
				d.nop_sel = 1'b1;
				ex.bypass_alu_in_1_sel = 2'b00;
				ex.bypass_alu_in_2_sel = 2'b00;
			end
		end
		else begin
			if (ex.DX_IR[`RT_e:`RT_s] == d.insn_in[`RS_e:`RS_s]) begin
				stall_f = 1'b1;
				d.nop_sel = 1'b1;
				ex.bypass_alu_in_1_sel = 2'b00;
				ex.bypass_alu_in_2_sel = 2'b00;
			end
		end
	end
	
	// CHECK FOR END OF PROGRAM
	if ((ex.DX_IR[`OP_e:`OP_s] == `SPECIAL) && (ex.DX_IR[`FUNK_e:`FUNK_s] == `JR)) begin
		if (ex.DX_A == 31)
			$display("END OF PROGRAM");
			stall_f = 1'b1;
			d.nop_sel = 1'b1;
								
			stall_f 		= 1'b1;
			stall_rf		= 1'b1;
			stall_ex		= 1'b1;
			stall_mem_1		= 1'b1;
			stall_wb		= 1'b1;
			
			$display("Final Register Content");
			for (i = 0; i < 32; i = i + 1) begin
				$display("Register %d = %d\t(0x%h)", i, rf.regs[i], rf.regs[i]);
			end
			$finish;
	end
end

/* Instantiate all CPU modules */
memory_0 mem_0 (
	.clk 		 	(clk),
	.address 	 	(mem_0_address),
	.data_in 	 	(data_in),
	.access_size 	(mem_0_access_size),
	.rw 		 	(mem_0_rw),
	.FD_IR 			(d.insn_in)
);

mem_pipe mem_1 (
	.clk			(clk),
	.stall_in		(stall_mem_1),
	.address		(mem_1_address),
	.data_in		(mem_1_data_in),
	.access_size	(mem_1_access_size),
	.rw				(mem_1_rw),
	.XM_B			(ex.XM_B),
	.XM_IR			(ex.XM_IR),
	.MW_IR			(wb.MW_IR),
	.XM_O			(ex.XM_O),
	.wb_we_in		(ex.wb_we_out),
	.wb_we_out		(wb.wb_we_in),
	.wb_reg_addr_in	(ex.wb_reg_addr_out),
	.wb_reg_addr_out (wb.write_addr_in),
	.wb_reg_data_out (wb.write_data_in),
	.sel_out		(ex.mem_out_sel_out),
	.wb_data_bypass	(mem_1.wb_reg_data_out)
);

// Instantiate fetch module and connect ports
fetch f (
	.clk_in 		 (clk),
	.pc_out			 (d1_address_0),
	.pc_decode_out 	 (d.pc_in),
	.pc_out_new		 (ex.pc_out),
	.pc_sel			 (ex.pc_sel),
	.rw_out 		 (d1_rw_0),
	.stall_in 		 (stall_f),
	.access_size_out (d1_access_size_0)
);

// Instantiate decode module and connect ports
decode d (
	.clk_in			(clk),
	.insn_in 		(mem_0.FD_IR),
	.pc_in 			(f.pc_decode_out),
	.rs				(rf.rs_in),
	.rt				(rf.rt_in),
	.DX_PC			(ex.DX_PC),
	.DX_IR			(ex.DX_IR),
	.s_immed		(ex.s_immed),
	.alu_funk		(ex.alu_funk),
	.alu_in_1_sel	(ex.alu_in_1_sel),
	.alu_in_2_sel	(ex.alu_in_2_sel),
	.mem_out_sel	(ex.mem_out_sel_in),
	.wb_reg_addr	(ex.wb_reg_addr_in),
	.wb_we			(ex.wb_we_in),
	.mem_access_size (ex.mem_access_size_in),
	.mem_rw			(ex.mem_rw_in),
	.xm_o_sel		(ex.xm_o_sel),
	.d1_xm_o		(ex.d1_xm_o_in)
);

reg_file rf (
	.clk_in			(clk),
	.stall_in		(stall_rf),
	.rs_in			(d.rs),
	.rt_in			(d.rt),
	.rd_in			(wb.write_addr_out),
	.we				(wb.rf_we),	
	.write_data		(wb.write_data_out),
	.rs_out			(ex.DX_A),
	.rt_out			(ex.DX_B)
);

exec ex	(
	.clk_in				(clk),
	.stall_in			(stall_ex),
	.DX_A				(rf.rs_out),
	.DX_B				(rf.rt_out),
	.DX_PC				(d.DX_PC),
	.DX_IR				(d.DX_IR),
	.XM_O				(mem_1.XM_O),
	.XM_B				(mem_1.XM_B),
	.XM_IR				(mem_1.XM_IR),
	.mem_address		(d1_address_1),
	.mem_data_in		(d1_data_in_1),
	.mem_access_size_in	(d.mem_access_size),
	.mem_access_size_out (d1_access_size_1),
	.mem_rw_in			(d.mem_rw),
	.mem_rw_out			(d1_rw_1),
	.mem_out_sel_in		(d.mem_out_sel),
	.mem_out_sel_out 	(mem_1.sel_out),
	.wb_we_in			(d.wb_we),
	.wb_we_out			(mem_1.wb_we_in),
	.wb_reg_addr_in		(d.wb_reg_addr),
	.wb_reg_addr_out 	(mem_1.wb_reg_addr_in),
	.pc_sel				(f.pc_sel),
	.pc_out				(f.pc_out_new),
	.alu_funk			(d.alu_funk),
	.alu_in_1_sel		(d.alu_in_1_sel),
	.alu_in_2_sel		(d.alu_in_2_sel),
	.s_immed			(d.s_immed),
	.xm_o_sel			(d.xm_o_sel),
	.wb_data_bypass		(mem_1.wb_reg_data_out),
	.d1_xm_o_in			(d.d1_xm_o)
);

write_back wb (
	.clk			(clk),
	.stall_in		(stall_wb),
	.MW_IR			(mem_1.MW_IR),
	.rf_we			(rf.we),
	.write_data_in	(mem_1.wb_reg_data_out),
	.write_data_out (rf.write_data),
	.write_addr_in	(mem_1.wb_reg_addr_out),
	.write_addr_out (rf.rd_in),
	.wb_we_in		(mem_1.wb_we_out)
);

endmodule
