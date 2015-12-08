
//////////////////////////////////////////////////////////////////////////////////
// CMPE 140, CMPE Department, San Jose State University 
// Authors: Donald Hung and Hoan Nguyen, Matthew Carlis
// 
// Create Date:    08:36:48 02/25/2010 
// Design Name: 
// Module Name:    mips_top 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//  
//////////////////////////////////////////////////////////////////////////////////
// MIPS Top-level Module (including the memories, clock,
// and the display module for prototyping)
module mips_top(
	input					clk, reset, 
	output				memwrite,
	output	[ 3:0] 	top_an, 
	output 	[ 7:0]	top_sseg,
	input		[ 7:0]	switches,
	output				sinkBit,
	input reset_Wreg);

	wire		[31:0] 	pc, instr, dataadr, writedata, dispDat;
	wire 					clksec;
	reg		[ 7:0] 	reg_hex1, reg_hex0; 

	wire [31:0] readsdata_M, readsdata_W;

	// Clock (1 second) to slow down the running of the instructions
	clk_gen top_clk(.clk50MHz(clk), .reset(reset), .clksec(clksec));
      
	// Instantiate processor and memories	
	mips 	mips	(clksec, reset, pc, instr, 
					memwrite, dataadr, writedata, readsdata_W, switches[4:0], dispDat, reset_Wreg); //dataadr is the same as aluout_M in my diagrams
									
	imem 	imem	(pc[7:2], instr);
	dmem	dmem	(clksec, memwrite, dataadr, writedata, readsdata_M); ////changed clk to clksec


	//Wreg pipeline register I inserted for the data memroy
	
	
	//changed clk to clksec
	Wreg_M Writebackreg_M (readsdata_M, clksec, readsdata_W);
	
	//////


	// Instantiate 7-seg LED display module
	disp_hex_mux disp_unit (.clk(clk), .reset(1'b0), 
		.hex3(reg_hex1[7:4]), .hex2(reg_hex1[3:0]), 
		.hex1(reg_hex0[7:4]), .hex0(reg_hex0[3:0]),  
		.dp_in(4'b1111), .an(top_an), .sseg(top_sseg));

	// contents displayed on the 7 segment LEDs depending on DIP switches 7:5
	//   	7:5 = 000: display PC & LSB of register selected by DIP switches 4:0
	//   	7:5 = 001: display PC & LSB of instr
	//   	7:5 = 010: display PC & LSB of dataadr
	//   	7:5 = 011: display PC & LSB of writedata
	// 	7:5 = 100: display PC & instr byte 0
	// 	7:5 = 101: display PC & instr byte 1
	// 	7:5 = 110: display PC & instr byte 2
	//  	7:5 = 111: display PC & instr byte 3
	always @ (posedge clk) begin
		reg_hex1 = pc[7:0];
		case ({switches[7],switches[6], switches[5]})
			3'b000:	begin
					reg_hex0 = dispDat[ 7:0];
					end
			3'b001:	begin
					reg_hex0 = instr[ 7:0];
					end
			3'b010:	begin
					reg_hex0 = dataadr[ 7:0];
					end
			3'b011:	begin
					reg_hex0 = writedata[ 7:0];
					end
			3'b100:	begin
					reg_hex0 = instr[ 7:0];
					end
			3'b101:	begin
					reg_hex0 = instr[ 15:8];
					end
			3'b110:	begin
					reg_hex0 = instr[ 23:16];
					end
			3'b111:	begin
					reg_hex0 = instr[ 31:24];
					end
			endcase
	end		

	//sink unused bit(s) to knock down the number of warning messages
	assign sinkBit = (pc > 0) ^ (instr > 0) ^ (dataadr > 0) ^ (writedata > 0) ^ 
					 (readsdata_W > 0) ^ (dispDat > 0);
endmodule



// the register below is the pipeline register Wreg that is located after the Data Memory

module Wreg_M (input [31:0] readdata_in, input clk, output reg [31:0] readdata_out );

	always @(posedge clk)
		begin
			readdata_out <= readdata_in;
		end

endmodule


