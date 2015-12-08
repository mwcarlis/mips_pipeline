`timescale 1ns/1ps
 
module MIPS_testbench();

	reg	clk_tb;
	reg 	reset_tb;
	reg reset_Wreg_c_tb;
	reg	[4:0] dispSel_tb;
	
	wire [31:0] pc_tb, dispDat_tb, writedata_tb, dataaddr_tb;
	wire memwrite_tb;
	
	top_of_MIPS_for_simulation DUT(clk_tb, reset_tb, dispSel_tb, pc_tb, dispDat_tb, dataaddr_tb, memwrite_tb, writedata_tb, reset_Wreg_c_tb);
	 
	// initialize test
	initial
		begin
			reset_tb   <= 1; #5; 
			reset_tb   <= 0;
			dispSel_tb = 5'b10000; //make dispSel_tb=10000 so you can check if register $s0(which is register 16) has the correct								  //result below in your test case
		end
	
	initial
		begin
			reset_Wreg_c_tb <= 1; #35; //hold reset for 35 seconds for the Wreg
			reset_Wreg_c_tb <= 0;
		end
		
	// generate clock to sequence tests
	always
		begin
			clk_tb <= 1; # 5; 
			clk_tb <= 0; # 5;
		end

	// checks to see if 4! (which is x18) is written in $s0 after your assembly program is done executing
	always@(negedge clk_tb)
		begin
			if(pc_tb === 32'h30) //so when pc reaches pc = x30, you should have x18 in $s0(register adress 16)
				begin
					if(dispDat_tb == 32'h18) 
						begin
							$display("Simulation succeeded"); #5;
							$stop;
						end 
					else 
						begin
							$display("Simulation failed"); #5;
							$stop;
						end
				end
		end

endmodule


/*


//------------------------------------------------
// mipstest.v
// David_Harris@hmc.edu 23 October 2005
// Testbench for MIPS processor
//------------------------------------------------
`timescale 1ns/1ps

module testbench();

  reg         clk;
  reg         reset;
 
  wire [31:0] writedata, dataadr, readdata_tb, instr_top;
  wire memwrite;

  // instantiate device to be tested
  top dut(clk, reset, writedata, readdata_tb, dataadr, instr_top, pc_top, memwrite);
   
  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check that 0x41 gets written to address Mem[0x30]
  always@(negedge clk)
    begin
      if(memwrite) 
			begin
				if(dataadr === 32'h30 & writedata === 32'h41) 
					begin
						$display("Simulation succeeded"); #5;
						$stop;
					end 
				else 
					if (dataadr === 32'h30 & writedata !== 32'h41) 
						begin
							$display("Simulation failed"); #5;
							$stop;
						end
			end
    end
endmodule

*/

