/**** MEMORY ****/
`define ADDR_START	32'h80020000
`define	MEM_DEPTH	20'hFFFFF

/**** GLOBAL CPU CONSTANTS ****/
`define	SPECIAL		6'b000000
	`define ADD		6'b100000
	`define ADDU	6'b100001
	`define AND		6'b100100
	`define JR		6'b001000
	`define NOR		6'b100111
	`define OR		6'b100101
	`define SLL		6'b000000
	`define SLT		6'b101010
	`define SLTU	6'b101011
	`define SRA		6'b000011
	`define SRL		6'b000010
	`define SUB		6'b100010
	`define SUBU	6'b100011
	`define XOR		6'b100110

`define SPECIAL2	6'b011100
	`define MUL		6'b000010

`define ADDI		6'b001000
`define ADDIU		6'b001001
`define ANDI		6'b001100
`define	ORI			6'b001101

`define REGIMM		6'b000001
	`define BGEZ	5'b00001
	`define BGEZAL	5'b10001
	`define BGEZALL	5'b10011
	`define BGEZL	5'b00011
	`define BLTZ	5'b00000
	`define BLTZAL	5'b10000
	`define	BLTZALL	5'b10010
	`define	BLTZL	5'b00010

`define BEQ			6'b000100
`define	BEQL		6'b010100
`define	BGTZ		6'b000111
`define	BGTZL		6'b010111
`define	BLEZ		6'b000110
`define	BLEZL		6'b010110
`define	BNE			6'b000101
`define	BNEL		6'b010101
`define	J			6'b000010
`define	JAL			6'b000011

`define	LB			6'b100000
`define	LBU			6'b100100
`define	LUI			6'b001111
`define	LW			6'b100011
`define	SB			6'b101000
`define	SLTI		6'b001010
`define	SLTIU		6'b001011
`define	SW			6'b101011

/**** Instruction Breakdown ****/
`define OP_e		31
`define OP_s		26
`define RS_e		25
`define RS_s		21
`define RT_e		20
`define RT_s		16
`define RD_e		15
`define RD_s		11
`define SH_e		10
`define SH_s		6
`define FUNK_e		5
`define FUNK_s		0
`define I_e			15
`define I_s			0

`define NOP			32'h00000000
