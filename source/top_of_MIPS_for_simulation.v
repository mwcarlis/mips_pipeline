


module top_of_MIPS_for_simulation( input clk, reset, input [4:0] dispSel, output [31:0] pc, dispDat,
												dataaddr, output memwrite, output [31:0] writedata, input reset_Wreg);

	wire [31:0] instr,readdata_M, readdata_W;
	
	imem imem_d (pc[7:2], instr[31:0]);
	mips mips_d (clk, reset, pc, instr[31:0], memwrite, dataaddr, writedata, readdata_W, dispSel,dispDat, 
						reset_Wreg);
	dmem deme_d ( clk, memwrite, dataaddr, writedata, readdata_M);
	Wreg_M Writebackreg_d(readdata_M, clk, readdata_W);
	
	
	//dataaddr is my aluout_M in my diagrams on visio
endmodule
