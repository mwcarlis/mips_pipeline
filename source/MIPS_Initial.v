//-----------------------------------------------------------------
// Module Name   : clk_gen
// Description   : Generate 4 second and 5KHz clock cycle from
//                 the 50MHz clock on the Nexsys2 board
//------------------------------------------------------------------
module clk_gen(
	input			clk50MHz, reset, 
	output reg		clksec );

	reg 			clk_5KHz;
	integer 		count, count1;
	
	
	
	always@(posedge clk50MHz) begin
		if(reset) begin
			count = 0;
			count1 = 0;
			clksec = 0;
			clk_5KHz =0;
		end else begin
			if (count == 50000000) begin
				// Just toggle after certain number of seconds
				clksec = ~clksec;
				count = 0;
			end
			if (count1 == 20000) begin
				clk_5KHz = ~clk_5KHz;
				count1 = 0;
			end
			count = count + 1;
			count1 = count1 + 1;
		end
	end
endmodule

//------------------------------------------------
// Source Code for a Single-cycle MIPS Processor (supports partial instruction)
// Developed by D. Hung, D. Herda and G. Gerken,
// based on the following source code provided by
// David_Harris@hmc.edu (9 November 2005):
//    mipstop.v
//    mipsmem.v
//    mips.v
//    mipsparts.v
//------------------------------------------------

// Main Decoder
module maindec(
	input	[ 5:0]	op, funct,
	output			memtoreg, memwrite, branch, alusrc, regdst, regwrite, jump,
	output	[ 1:0]	aluop, 
	output Lo_write, Hi_write, Hi_Lo, Result_Select, Jump_Register, jal_reg, Reg_Dst_jal);

	reg 	[ 15:0]	controls;

	assign {regwrite, regdst, alusrc, branch, memwrite, memtoreg, jump, aluop,
			  Lo_write, Hi_write, Hi_Lo, Result_Select, Jump_Register, jal_reg, Reg_Dst_jal } = controls;
	
	//if you need to write dont cares, including in aluop, just make them 0's
	always @(*)
		case(op) 														//the _ _ means its the 2bit aluop
			6'b000000:	 // Rtype
				case(funct)
					6'b011000: controls <= 16'b0000000_00_1100000; //Rtype MULT
					6'b010010: controls <= 16'b1100000_00_0011000; //Rtype MFLO
					6'b010000: controls <= 16'b1100000_00_0001000; //Rtype MFHI
					6'b001000: controls <= 16'b0000000_00_0000100; //Rtype JR
					default:   controls <= 16'b1100000_10_0000000; //This is the control signals for the rest of the Rtype
				endcase												  //instrcuctions, which they all have in common
							
			6'b100011: controls <= 16'b1010010_00_0000000; //LW
			6'b101011: controls <= 16'b0010100_00_0000000; //SW
			6'b000100: controls <= 16'b0001000_01_0000000; //BEQ
			6'b001000: controls <= 16'b1010000_00_0000000; //ADDI
			6'b000010: controls <= 16'b0000001_00_0000000; //J
			6'b000011: controls <= 16'b1000001_00_0000011; //JAL
			default:   controls <= 16'bxxxxxxxxxxxxxxxx; //???
		endcase
endmodule

// ALU Decoder
module aludec(
	input		[5:0]	funct,
	input		[1:0]	aluop,
	output reg	[2:0]	alucontrol);

	always @(*)
		case(aluop)
			2'b00: alucontrol <= 3'b010;  // add
			2'b01: alucontrol <= 3'b110;  // sub
			default: case(funct)          // RTYPE
				6'b100000: alucontrol <= 3'b010; // ADD
				6'b100010: alucontrol <= 3'b110; // SUB
				6'b100100: alucontrol <= 3'b000; // AND
				6'b100101: alucontrol <= 3'b001; // OR
				6'b101010: alucontrol <= 3'b111; // SLT
				default:   alucontrol <= 3'bxxx; // ???
			endcase
		endcase
endmodule
// ALU
module alu(
	input		[31:0]	a, b, 
	input		[ 2:0]	alucont, 
	output reg	[31:0]	result,
	output			zero );

	wire	[31:0]	b2, sum, slt;

	assign b2 = alucont[2] ? ~b:b; 
	assign sum = a + b2 + alucont[2];
	assign slt = sum[31];

	always@(*)
		case(alucont[1:0])
			2'b00: result <= a & b;
			2'b01: result <= a | b;
			2'b10: result <= sum;
			2'b11: result <= slt;
		endcase

	assign zero = (result == 32'b0);
endmodule

// Adder
module adder(
	input	[31:0]	a, b,
	output	[31:0]	y );

	assign y = a + b;
endmodule

// Two-bit left shifter
module sl2(
	input	[31:0]	a,
	output	[31:0]	y );

	// shift left by 2
	assign y = {a[29:0], 2'b00};
endmodule

// Sign Extension Unit
module signext(
	input	[15:0]	a,
	output	[31:0]	y );

	assign y = {{16{a[15]}}, a};
endmodule

// Parameterized Register
module flopr #(parameter WIDTH = 8) (
	input					clk, reset,
	input		[WIDTH-1:0]	d, 
	output reg	[WIDTH-1:0]	q);

	always @(posedge clk, posedge reset)
		if (reset) q <= 0;
		else       q <= d;
endmodule

// commented out since flopenr is not used
//module flopenr #(parameter WIDTH = 8) (
//	input					clk, reset,
//	input					en,
//	input		[WIDTH-1:0]	d, 
//	output reg	[WIDTH-1:0]	q);
//
//	always @(posedge clk, posedge reset)
//		if      (reset) q <= 0;
//		else if (en)    q <= d;
//endmodule

// Parameterized 2-to-1 MUX
module mux2 #(parameter WIDTH = 8) (
	input	[WIDTH-1:0]	d0, d1, 
	input				s, 
	output	[WIDTH-1:0]	y );

	assign y = s ? d1 : d0; 
endmodule

module mux4 #(parameter WIDTH = 8) (
	input [WIDTH-1:0] d0, d1, d2, d3,
	input	[1:0]		s,
	output reg [WIDTH-1:0] y );
	always @(s) begin
		if( s == 2'b00 ) begin
			y = d0;
		end	 else if( s == 2'b01 ) begin
			y = d1;
		end else if( s == 2'b10 ) begin
			y = d2;
		end else if( s == 2'b11 ) begin
			y = d3;
		end
	end
endmodule

// register file with one write port and three read ports
// the ra4(adress register) and rd4 are just ports that will be used to read data from a register in
	// FPGA implementation only
module regfile(	
	input			clk, 
	input			we3, 
	input 	[ 4:0]	ra1, ra2, wa3, 
	input	[31:0] 	wd3, 
	output 	[31:0] 	rd1, rd2,
	input	[ 4:0] 	ra4,
	output 	[31:0] 	rd4);

	reg		[31:0]	rf[31:0];
	integer			n;
	
	//initialize registers to all 0s
	initial 
		for (n=0; n<32; n=n+1) 
			rf[n] = 32'h00;
			
	//write first order, include logic to handle special case of $0
    always @(negedge clk)
        if (we3)
			if (~ wa3[4])
				rf[{0,wa3[3:0]}] <= wd3;
			else
				rf[{1,wa3[3:0]}] <= wd3;
		
			// this leads to 72 warnings
			//rf[wa3] <= wd3;
			
			// this leads to 8 warnings
			//if (~ wa3[4])
			//	rf[{0,wa3[3:0]}] <= wd3;
			//else
			//	rf[{1,wa3[3:0]}] <= wd3;
		
	assign rd1 = (ra1 != 0) ? rf[ra1[4:0]] : 0;
	assign rd2 = (ra2 != 0) ? rf[ra2[4:0]] : 0;
	assign rd4 = (ra4 != 0) ? rf[ra4[4:0]] : 0;
endmodule

// Control Unit
module controller(
	input	[5:0]	op, funct,
	output			memtoreg, memwrite, alusrc, regdst, regwrite, jump,
	output	[2:0]	alucontrol,
	output Lo_write, Hi_write, Hi_Lo, Result_Select, Jump_Register, jal_reg, Reg_Dst_jal, branch);

	wire	[1:0]	aluop;

	maindec	md(op,funct, memtoreg, memwrite, branch, alusrc, regdst, regwrite, jump, aluop, Lo_write, Hi_write,
						Hi_Lo, Result_Select, Jump_Register, jal_reg, Reg_Dst_jal);
	aludec	ad(funct, aluop, alucontrol);

endmodule

// Data Path (excluding the instruction and data memories)
module datapath(
	input			clk, reset, memtoreg, pcsrc, alusrc, regdst, regwrite, jump,
	input	[2:0]	alucontrol,
	output			zero,
	output	[31:0]	pc,
	input	[31:0]	instr,
	output	[31:0]	aluout, writedata,
	input	[31:0]	readdata,
	input	[ 4:0]	dispSel,
	output	[31:0]	dispDat,
	input Lo_write_E, Hi_write_E, Hi_Lo_M, Result_Select_W, Jump_Register, 
			jal_reg, Reg_Dst_jal_W, reset_Wreg, Reg_Dst_jal_M,
			regwrite_M, regwrite_W, Result_Select_M );

	wire [31:0] srcHi, srcLo, Hi_Lo_result, WD3_Result;
	//wires i created for the pipelined processor
	wire [31:0] pc_result_F, pcplus4_F, pcnextbranch_F, pcnext_F; 
	
	wire [31:0] pcplus4_E, signimm_E, signimmsh_E, pcbranch_E, srcb_temp_E, srcb_E, srca_E, alu_out_E; 
	wire [4:0]  rt_E, rd_E, rs_E, writewreg_E;
	wire [63:0]  multu_E;
	wire [25:0] instr_E;
	wire zero_E;
	
	wire [31:0] srca_D, srcb_D, signimm_D, pcplus4_D;
	
	wire [31:0] pcplus4_M, srca_M, pcbranch_M;
	wire [31:0] hi_lo;
	wire [4:0] writewreg_M, jal_A3_out;
	wire [25:0] instr_M;
	wire zero_M;
	
	wire [31:0] pcplus4_W, pcbranch_W, WD3_W, alu_out_W, result_W, srca_W;
	wire [4:0]  A3_final_W;
	wire [63:0] multu_W;
	wire [25:0] instr_W;

	wire [31:0] rdm1_out, rdm2_out;
	wire [1:0]  fw_AE, fw_BE;

	// next PC logic
	flopr #(32) pcreg(clk, reset, pc_result_F, pc); //done!!!
	adder       pcadd1(pc, 32'b100, pcplus4_F); //done!!! 
	sl2         immsh(signimm_E, signimmsh_E); //done!!!!
	adder       pcadd2(pcplus4_E, signimmsh_E, pcbranch_E); //done!!!
	mux2 #(32)  pcbrmux(pcplus4_F, pcbranch_W, pcsrc, pcnextbranch_F); //done!!!
	mux2 #(32)  pcmux(pcnextbranch_F, {pcplus4_W[31:28], instr_W, 2'b00}, jump, pcnext_F); //done!!

	// register file logic
	regfile		rf(clk, regwrite, instr[25:21], instr[20:16], A3_final_W, WD3_W, srca_D, srcb_D, dispSel, dispDat); //done!!!
	mux2 #(5)	wrmux(rt_E, rd_E, regdst, writewreg_E); //done!!!
	mux2 #(32)	resmux(alu_out_W, readdata, memtoreg, result_W); //done!!
	signext		se(instr[15:0], signimm_D); //done!!!!

	// ALU logic
	mux2 #(32)	srcbmux(rdm2_out, signimm_E, alusrc, srcb_E); //done!

	mux4 #(32) 	rdm1( srca_E, WD3_W, aluout, hi_lo, fw_AE, rdm1_out);
	mux4 #(32)  rdm2( srcb_temp_E, WD3_W, aluout, hi_lo, fw_BE, rdm2_out);

	alu			alu(rdm1_out, srcb_E, alucontrol, alu_out_E, zero_E); //done!!!
	
	//what I added below
	Multiply_Unsigned MULTU(rdm1_out, srcb_E, multu_E); //done

	//d_register Hi_register(multu_W[63:32], Hi_write_E, clk, srcHi); //done!!!!!
	//d_register Lo_register(multu_W[31:0], Lo_write_E, clk, srcLo); //done!!!
	//mux2 #(32) Hi_Lo_Select(srcHi, srcLo, Hi_Lo_M, Hi_Lo_result);  //done!!!

	d_register Hi_register(multu_E[63:32], Hi_write_E, clk, srcHi); // CTRL Sig
	d_register Lo_register(multu_E[31:0], Lo_write_E, clk, srcLo); // CTRL Sig
	mux2 #(32) Hi_Lo_Select(srcHi, srcLo, Hi_Lo_M, hi_lo);  // CTRL Sig

	mux2 #(32) Result_Select_mux (result_W, Hi_Lo_result, Result_Select_W, WD3_Result); /// done!!!!!
	mux2 #(32) PC_jr_mux(pcnext_F,srca_W, Jump_Register, pc_result_F); //done!!!!
	mux2 #(32) jal_WD3_mux(WD3_Result, pcplus4_W, jal_reg, WD3_W); //done!!!!
	//mux2 #(5) jal_A3_mux(writewreg_temp_W, 5'b11111, Reg_Dst_jal, A3_final_W); //done!!!
	mux2 #(5) jal_A3_mux(writewreg_M, 5'b11111, Reg_Dst_jal_M, jal_A3_out); //done!!!


	hazard_unit uzunit( regwrite_M, regwrite_W, Result_Select_M, rs_E, rt_E, jal_A3_out, A3_final_W, fw_AE, fw_BE);
	//module hazard_unit( input regwrite_M, regwrite_W, Result_Select_M,
	//				input [4:0] RsE, RtE, writereg_M, writereg_W, 
	//				output [1:0] ForwardAE, ForwardBE );
	  
	
	//pipeline registers below
	
	Dreg_dp Decodereg_dp  (clk, pcplus4_F, pcplus4_D); //done!!
	Ereg_dp Executereg_dp (srca_D, srcb_D, instr[25:0], pcplus4_D, instr[20:16], instr[15:11], instr[25:21], signimm_D, clk,
									srca_E, srcb_temp_E, instr_E, pcplus4_E, rt_E, rd_E, rs_E, signimm_E ); //done!!!!
	Mreg_dp Memoryreg_dp  (zero_E, alu_out_E, rdm2_out, instr_E, pcplus4_E, writewreg_E, srca_E, pcbranch_E, clk,
									zero_M, aluout, writedata, instr_M, pcplus4_M, writewreg_M, srca_M, pcbranch_M );//done!!!!
	Wreg_dp Writebackreg_dp	 ( zero_M, aluout, instr_M, pcplus4_M, jal_A3_out, srca_M, hi_lo, pcbranch_M, clk,
										zero, alu_out_W, instr_W, pcplus4_W, A3_final_W, srca_W, Hi_Lo_result, pcbranch_W, reset_Wreg);	//done!!!!							
	
	
	
endmodule

// The MIPS (excluding the instruction and data memories)
module mips(
	input        	clk, reset,
	output	[31:0]	pc,
	input 	[31:0]	instr,
	output			memwrite,
	output	[31:0]	aluout, writedata,
	input 	[31:0]	readdata,
	input		[4:0]	dispSel,
	output	[31:0]	dispDat, 
	input reset_Wreg); //I only added this reset input for the Wreg_c amd Wreg_dp for simulation

	//wires I added for pipelined processor
	wire memtoreg_D, memwrite_D, alusrc_D, regdst_D, regwrite_D;
	wire jump_D, Lo_write_D, Hi_write_D, Hi_Lo_D;
	wire Result_Select_D, Jump_Register_D, jal_reg_D, Reg_Dst_jal_D, branch_D;
	wire [2:0] alucont_D;
	wire [31:0] instr_D;
	
	
	
	wire memtoreg_E, memwrite_E, alusrc_E, regdst_E, regwrite_E;
	wire jump_E, Lo_write_E, Hi_write_E, Hi_Lo_E;
	wire Result_Select_E, Jump_Register_E, jal_reg_E, Reg_Dst_jal_E, branch_E;
	wire [2:0] alucont_E;
	
	wire memtoreg_M, regwrite_M;
	wire jump_M, Lo_write_M, Hi_write_M, Hi_Lo_M;
	wire Result_Select_M, Jump_Register_M, jal_reg_M, Reg_Dst_jal_M, branch_M;
	
	wire memtoreg_W, regwrite_W;
	wire jump_W, Lo_write_W, Hi_write_W, Hi_Lo_W;
	wire Result_Select_W, Jump_Register_W, jal_reg_W, Reg_Dst_jal_W, branch_W;
	wire zero_W, pcsrc_W;
 
 
	controller c(instr_D[31:26], instr_D[5:0] ,
				memtoreg_D, memwrite_D, alusrc_D, regdst_D, regwrite_D, jump_D,
				alucont_D, Lo_write_D, Hi_write_D, Hi_Lo_D, 
				Result_Select_D, Jump_Register_D, jal_reg_D, Reg_Dst_jal_D, branch_D ); //done!!!
				
	//datapath dp(clk, reset, memtoreg_W, pcsrc_W,
	//			alusrc_E, regdst_E, regwrite_W, jump_W,
	//			alucont_E, zero_W, pc, instr_D, aluout, 
	//			writedata, readdata, dispSel, dispDat,
	//			Lo_write_W, Hi_write_W, Hi_Lo_W, Result_Select_W, 
	//			Jump_Register_W, jal_reg_W, Reg_Dst_jal_W, reset_Wreg,
	//			Reg_Dst_jal_M, regwrite_M, regwrite_W); //done!!!!


	datapath dp(clk, reset, memtoreg_W, pcsrc_W,
				alusrc_E, regdst_E, regwrite_W, jump_W,
				alucont_E, zero_W, pc, instr_D, aluout, 
				writedata, readdata, dispSel, dispDat,
				Lo_write_E, Hi_write_E, Hi_Lo_M, Result_Select_W, 
				Jump_Register_W, jal_reg_W, Reg_Dst_jal_W, reset_Wreg,
				Reg_Dst_jal_M, regwrite_M, regwrite_W, Result_Select_M); //done!!!!
			
	//pipeline registers I added
	Dreg_c Decodereg_c (instr , clk, instr_D); //done!!!!
	
	Ereg_c Executereg_c (memtoreg_D, memwrite_D, alusrc_D, regdst_D, jump_D, Lo_write_D, Hi_write_D, Hi_Lo_D, Result_Select_D,
								 Jump_Register_D, jal_reg_D, Reg_Dst_jal_D, alucont_D, regwrite_D, clk,
								   memtoreg_E, memwrite_E, alusrc_E, regdst_E, jump_E, Lo_write_E, Hi_write_E, Hi_Lo_E, Result_Select_E,
									Jump_Register_E, jal_reg_E, Reg_Dst_jal_E, alucont_E, regwrite_E, branch_D, branch_E); //done!!!!
									
	Mreg_c Memoryreg_c (memtoreg_E, memwrite_E, jump_E, Lo_write_E, Hi_write_E, Hi_Lo_E, Result_Select_E, Jump_Register_E,
								jal_reg_E, Reg_Dst_jal_E, regwrite_E, clk,
								memtoreg_M, memwrite, jump_M, Lo_write_M, Hi_write_M, Hi_Lo_M, Result_Select_M, Jump_Register_M,
								jal_reg_M, Reg_Dst_jal_M, regwrite_M, branch_E, branch_M); //done!!!!
								
	Wreg_c Writebackreg_c (memtoreg_M, jump_M, Lo_write_M, Hi_write_M, Hi_Lo_M, Result_Select_M, Jump_Register_M, jal_reg_M,
								   Reg_Dst_jal_M, regwrite_M, clk, 
									memtoreg_W, jump_W, Lo_write_W, Hi_write_W, Hi_Lo_W, Result_Select_W, Jump_Register_W, jal_reg_W,
									Reg_Dst_jal_W, regwrite_W, reset_Wreg, branch_M, branch_W );
									
	assign pcsrc_W = branch_W & zero_W; 
	
	
endmodule

// Instruction Memory
module imem (
	input	[ 5:0]	a,
	output 	[31:0]	dOut );
	
	reg		[31:0]	rom [0:63]; //this means you will have 64 word locations(which mean each memory location rom[a] conatins a word (4bytes))
	 
	
	//VERY IMPORTANT. YOU NEED TO INITIALIZE ALL THE MEMRY LOCATIONS IN THE ROM BELOW. SO TO DO THIS, YOU MUST COMPLETELY FILL UP
	//THE MEMFILE_S.DAT FILE(i.e you must fill the remaining instructions in the memfile_s.dat with dummy instruction
	//code. SO YOU MUST HAVE INSTRUCTIONS IN YOUR MEMFILE_S.DAT FILE)
	//
	//initialize rom from memfile_s.dat
	initial 
		$readmemh("memfile_s.dat", rom);
	    
	//simple rom
    assign dOut = rom[a];
endmodule
 
// Data Memory 
module dmem (
	input			clk,
	input			we,
	input	[31:0]	addr,
	input	[31:0]	dIn,
	output 	[31:0]	dOut );
	
	reg		[31:0]	ram[63:0];
	integer			n;
	
	//initialize ram to all FFs
	initial 
		for (n=0; n<64; n=n+1)
			ram[n] = 8'hFF;
		
	assign dOut = ram[addr[31:2]];
				
	always @(posedge clk)
		if (we) 
			ram[addr[31:2]] = dIn; 
endmodule


module hazard_unit( input regwrite_M, regwrite_W, Result_Select_M,
					input [4:0] RsE, RtE, writereg_M, writereg_W, 
					output [1:0] ForwardAE, ForwardBE );
	reg [1:0] FAE, FBE;
	always @(*)
	begin
		if((RsE != 5'b00000) & (RsE == writereg_M) & regwrite_M & Result_Select_M) begin
			FAE = 2'b11;
		end
		else if ((RsE != 5'b00000) & (RsE == writereg_M) & regwrite_M & ~Result_Select_M) begin
			FAE = 2'b10;
		end 
		else if ((RsE != 5'b00000) & (RsE == writereg_W) & regwrite_W) begin  
			FAE = 2'b01;
		end 
		else begin
			FAE = 2'b00;
		end


		if((RtE != 5'b00000) & (RtE == writereg_M) & regwrite_M & Result_Select_M) begin
			FBE = 2'b11;
		end
		else if ((RtE != 5'b00000) & (RtE == writereg_M) & regwrite_M & ~Result_Select_M) begin
			FBE = 2'b10;
		end 
		else if ((RtE != 5'b00000) & (RtE == writereg_W) & regwrite_W) begin  
			FBE = 2'b01;
		end 
		else begin
			FBE = 2'b00;
		end
	end

	assign ForwardAE = FAE;
	assign ForwardBE = FBE;


endmodule


//I added the following modules for my extended functionality

module Multiply_Unsigned ( input [31:0] a, b, output [63:0] y);

	assign y = a * b;

endmodule


module d_register ( input [31:0] d, input we, clk, output reg [31:0] q); //will be used for my Hi and Lo registers

always @ (posedge clk)
	begin
		if (we)
			begin
				q <= d;
			end
		else
			begin
				q <= q;
			end
	end
endmodule

 // things added below are for the pipelined extension
module Dreg_dp (input clk, input [31:0] pcplus4_in, output reg [31:0] pcplus4_out);

always @ (posedge clk)
	begin
		pcplus4_out <= pcplus4_in;
	end
	
endmodule



module Ereg_dp (input [31:0] srca_in, srcb_in, input [25:0] instr_in, input [31:0] pcplus4_in, input [4:0] rt_in,
						rd_in, rs_in, input [31:0]  signimm_in, input clk,
					 output reg [31:0] srca_out, srcb_out, output reg [25:0] instr_out, output reg [31:0] pcplus4_out, 
					   output reg [4:0]  rt_out, rd_out, rs_out, output reg [31:0] signimm_out );
						
//the 26 bit input instr_in and 26 bit output intr_out will be used for the jumo instructions					 
always @(posedge clk)
	begin
		srca_out	<= srca_in;
		srcb_out	<= srcb_in;
		instr_out	<= instr_in;
		pcplus4_out	<= pcplus4_in;
		rt_out		<= rt_in;
		rd_out		<= rd_in;
		rs_out 		<= rs_in;
		signimm_out	<= signimm_in;
	end
							 
endmodule

module Mreg_dp (input zero_in, input [31:0] alu_in, writedata_in, input [25:0] instr_in, input [31:0] pcplus4_in,
						input [4:0] writewreg_in, input [31:0] srca_in, input [31:0] pcbranch_in,
						input clk,
					 output reg zero_out, output reg [31:0] alu_out, writedata_out, output reg [25:0] instr_out, 
					  output reg [31:0] pcplus4_out, output reg [4:0] writewreg_out, output reg [31:0] srca_out, 
					  output reg [31:0] pcbranch_out);
					  
//the 26 bit input instr_in and 26 bit output intr_out will be used for the jumo instructions					 					 
always @ (posedge clk)
	begin
		zero_out			<= zero_in;
		alu_out			<= alu_in;
		writedata_out	<= writedata_in;
		instr_out		<= instr_in;
		pcplus4_out		<= pcplus4_in;
		writewreg_out	<= writewreg_in;
		srca_out			<= srca_in;
		pcbranch_out	<= pcbranch_in;
	end

endmodule

module Wreg_dp (input zero_in, input [31:0] alu_in, input [25:0] instr_in, input [31:0] pcplus4_in, 
						input [4:0] writewreg_in, input [31:0] srca_in, input [31:0] hi_lo_in, 
						input [31:0] pcbranch_in, input clk,
					 output reg zero_out, output reg [31:0] alu_out, output reg [25:0] instr_out, 
					   output reg [31:0] pcplus4_out, output reg [4:0] writewreg_out, output reg [31:0] srca_out, 
						output reg [31:0] hi_lo_out, output reg [31:0] pcbranch_out, input reset_Wreg);
						
//the 26 bit input instr_in and 26 bit output intr_out will be used for the jumo instructions					 		 			 
always @ (posedge clk, posedge reset_Wreg)
	if(reset_Wreg)
		begin
			zero_out			<= 0;
			alu_out			<= 0;
			instr_out		<= 0;
			pcplus4_out		<= 0;
			writewreg_out	<= 0;
			srca_out			<= 0;
			hi_lo_out		<= 0;
			pcbranch_out	<= 0;
		end
	else
		begin
			zero_out			<= zero_in;
			alu_out			<= alu_in;
			instr_out		<= instr_in;
			pcplus4_out		<= pcplus4_in;
			writewreg_out	<= writewreg_in;
			srca_out			<= srca_in;
			hi_lo_out		<= hi_lo_in;
			pcbranch_out	<= pcbranch_in;
		end

endmodule


//belowe are the pipelined registers for my controller

module Ereg_c (input memtoreg_in, memwrite_in, alusrc_in, regdst_in, jump_in, Lo_write_in, Hi_write_in, Hi_Lo_in, Result_Select_in,
			Jump_Register_in, jal_reg_in, Reg_Dst_jal_in, input [2:0] alucont_in, input  regwrite_in, clk, 
			output reg memtoreg_out, memwrite_out, alusrc_out, regdst_out, jump_out, Lo_write_out, Hi_write_out, Hi_Lo_out, 
			Result_Select_out, Jump_Register_out, jal_reg_out, Reg_Dst_jal_out, output reg [2:0] alucont_out, output reg regwrite_out,
			input branch_in, output reg branch_out);
			
always @ (posedge clk)
	begin
		memtoreg_out			<= memtoreg_in;
		memwrite_out			<= memwrite_in;
		alusrc_out				<= alusrc_in;
		regdst_out				<= regdst_in;
		jump_out					<= jump_in;
		Lo_write_out			<= Lo_write_in;
		Hi_write_out			<= Hi_write_in;
		Hi_Lo_out 				<= Hi_Lo_in;
		Result_Select_out		<= Result_Select_in;
		Jump_Register_out		<= Jump_Register_in;
		jal_reg_out				<= jal_reg_in;
		Reg_Dst_jal_out		<= Reg_Dst_jal_in;
		alucont_out				<= alucont_in;
		regwrite_out			<= regwrite_in;
		branch_out				<= branch_in;
	end
			 
endmodule

module Mreg_c (input memtoreg_in, memwrite_in, jump_in, Lo_write_in, Hi_write_in, Hi_Lo_in, Result_Select_in,
			Jump_Register_in, jal_reg_in, Reg_Dst_jal_in, input regwrite_in, clk, 
			output reg memtoreg_out, memwrite_out, jump_out, Lo_write_out, Hi_write_out, Hi_Lo_out, 
			Result_Select_out, Jump_Register_out, jal_reg_out, Reg_Dst_jal_out, regwrite_out,
			input branch_in, output reg branch_out);
			
always @ (posedge clk)
	begin
		memtoreg_out			<= memtoreg_in;
		memwrite_out			<= memwrite_in;
		jump_out					<= jump_in;
		Lo_write_out			<= Lo_write_in;
		Hi_write_out			<= Hi_write_in;
		Hi_Lo_out 				<= Hi_Lo_in;
		Result_Select_out		<= Result_Select_in;
		Jump_Register_out		<= Jump_Register_in;
		jal_reg_out				<= jal_reg_in;
		Reg_Dst_jal_out		<= Reg_Dst_jal_in;
		regwrite_out			<= regwrite_in;
		branch_out				<= branch_in;
	end
			

endmodule


module Wreg_c (input memtoreg_in, jump_in, Lo_write_in, Hi_write_in, Hi_Lo_in, Result_Select_in,
			Jump_Register_in, jal_reg_in, Reg_Dst_jal_in, input regwrite_in, clk, 
			output reg memtoreg_out, jump_out, Lo_write_out, Hi_write_out, Hi_Lo_out, 
			Result_Select_out, Jump_Register_out, jal_reg_out, Reg_Dst_jal_out, regwrite_out, input reset_Wreg,
			input branch_in, output reg branch_out);
			
always @ (posedge clk, posedge reset_Wreg)
	if(reset_Wreg) //this is used for testbench simulation so this is necessary
		begin
			branch_out				<= 0;
			memtoreg_out			<= 0;
			jump_out					<= 0;
			Lo_write_out			<= 0;
			Hi_write_out			<= 0;
			Hi_Lo_out 				<= 0;
			Result_Select_out		<= 0;
			Jump_Register_out		<= 0;
			jal_reg_out				<= 0;
			Reg_Dst_jal_out		<= 0;
			regwrite_out			<= 0;
		
		end
	else
		begin
			branch_out				<= branch_in;
			memtoreg_out			<= memtoreg_in;
			jump_out					<= jump_in;
			Lo_write_out			<= Lo_write_in;
			Hi_write_out			<= Hi_write_in;
			Hi_Lo_out 				<= Hi_Lo_in;
			Result_Select_out		<= Result_Select_in;
			Jump_Register_out		<= Jump_Register_in;
			jal_reg_out				<= jal_reg_in;
			Reg_Dst_jal_out		<= Reg_Dst_jal_in;
			regwrite_out			<= regwrite_in;
		end
			

endmodule

module Dreg_c ( input [31:0] instr_in, input clk, output reg [31:0] instr_out);
 
always @( posedge clk)
	begin
		instr_out <= instr_in;
	end

endmodule

