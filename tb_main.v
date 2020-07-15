`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   21:43:29 03/09/2020
// Design Name:   main
// Module Name:   /home/utkarsh/Desktop/Processor/processor/tb_main.v
// Project Name:  processor
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: main
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_main;

	// Inputs
	reg clk;

	// Outputs
	wire [31:0]ans;
	wire [31:0]Instruction;
	wire [31:0]ALUoutput;
	wire [31:0]output1;
	wire [31:0]busA;
	wire [31:0]busB;
	wire [4:0]ra;
	wire [4:0]rb;
	wire [31:0]pipeline_reg;
	wire ExtOp;
	wire AluSrc;
	wire zero;
	wire [15:0] beq_offset;
   wire branch;
	// Instantiate the Unit Under Test (UUT)
	main uut (
		.clk(clk), 
		.ans(ans),
		.Instruction(Instruction),
		.ALUoutput(ALUoutput),
		.output1(output1),
		.busA(busA),
		.busB(busB),
		.ra(ra),
		.rb(rb),
		.pipeline_reg(pipeline_reg),
	   .ExtOp(ExtOp),
		.AluSrc(AluSrc),
		.zero(zero),
		.beq_offset(beq_offset),
		.branch(branch)
	);

	initial begin
		// Initialize Inputs
		clk = 1;

		// Wait 100 ns for global reset to finish
	
        
		// Add stimulus here

	end
	always #17 clk = !clk;      
endmodule

