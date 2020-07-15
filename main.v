`timescale 1ns / 1ps

module ALU(output reg signed [31:0]ans,input signed[3:0]sel,input signed[31:0]x,input signed[31:0]y,output zero,output reg overflow);
integer i; 
reg [31:0]signal_x;
reg [31:0]signal_y;
		
initial begin
	assign signal_x = $unsigned(x);
	assign signal_y = $unsigned(y);
end

always@(sel or x or y)
begin
	overflow = 1'b0;
	if(sel==4'b0000)
	begin
	   {overflow,ans[31:0]} = x + y;
	end
	else if(sel==4'b0001)
	begin
		ans = x - y;
	end
	else if(sel==4'b0010)
	begin
		ans= x >> y;
	end
	else if(sel==4'b0011)
	begin
		ans = x << y;
	end
	else if(sel==4'b0100)
	begin
		ans = x & y;
	end
	else if(sel==4'b0101)
	begin
		ans = x|y;
	end
	else if(sel==4'b0110)
	begin
		ans = ~(x|y);
	end
	else if(sel==4'b0111)
	begin
		ans = x^y;
	end
	
	else if(sel==4'b1000)
	begin
		if(x <= y)
			begin
				ans = 1'b0;
			end
		else
			begin
				ans = 1'b1;
			end
			
	end
	
	else if(sel==4'b1001)
	begin
		ans = x >>> y;
	end
	
	else if(sel==4'b1010)
	begin
		ans = y;
		ans[31:0] = $signed(ans) << x; 
	end

	else if(sel==4'b1011)
	begin
	   ans = signal_x - signal_y;            ////making unsigned and then subtracting 
	end
	
	else if(sel==4'b1100)
	begin
		if(x < y)
			begin
					ans = 32'b00000000000000000000000000000001;
			end
		else
			begin
					ans = 32'b00000000000000000000000000000000;
			end
	end
	
	else if(sel==4'b1101)
	begin
		if(signal_x < signal_y)
		begin
				ans = 32'b00000000000000000000000000000001;
		end
		else
		begin
				ans = 32'b00000000000000000000000000000000;
		end
	end	
	
	else if(sel==4'b1110)
	begin
	   if(x > 0)
			begin
				 ans = 32'd0;
			end
		else
			begin
				ans = 32'd1;
			end
	end
	else if(sel==4'b1111)
	begin
		if(x!=y)
			begin
				ans = 32'd0;
			end
		else
			begin
				ans = 32'd1;
			end
	end
end

assign zero = (ans==32'd0) ? 1'b1 : 1'b0;

endmodule
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module controlpath(
    input [31:0]Instruction,
	 output reg RegDst,
    output reg AluSrc,
    output reg MemtoReg,
    output reg RegWrite,
    output reg MemWrite,
    output reg Branch,
    output reg Jump,
    output reg ExtOp,
	 output reg [3:0]Aluctr,
	 output reg [4:0]ra,
	 output reg [4:0]rb,
	 output reg [4:0]rw,
	 output reg [15:0]beq_offset,
	 output reg [25:0]target_addr
	 );

always@(Instruction)
begin

	if(Instruction[31:25]==6'b001011) 
		begin
			ra = Instruction[20:16];
			rb = Instruction[15:11];
			AluSrc = 1;
			RegDst = 0;
			Aluctr[3:0] = 4'b1101;       ///SLTIU (comparision)          
			MemtoReg = 0;
			RegWrite = 1;
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			ExtOp = 1;
	end
	
	
	else if(Instruction[31:21]== 11'b00000000000 && Instruction[5:0] == 6'b000011)
	begin
			ra = Instruction[20:16];
			rb = Instruction[15:11];
	     	AluSrc = 1;
			RegDst = 0;
			Aluctr[3:0] = 4'b1001;      ///SRA                 
			MemtoReg = 0;
			RegWrite = 1;
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			ExtOp = 0;
	end
	
	else if(Instruction[31:21]== 11'b00000000000 && Instruction[5:0] == 6'b000000)
	begin
			ra = Instruction[20:16];
			rb = Instruction[15:11];
			AluSrc = 1;
			RegDst = 0;
			Aluctr[3:0] = 4'b0011;           ///////SLL          
			MemtoReg = 0;
			RegWrite = 1;
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			ExtOp = 0;
	end

   else if(Instruction[31:21]== 11'b00000000000 && Instruction[5:0] == 6'b000010) 
		begin                                
			ra = Instruction[20:16];
			rb = Instruction[15:11];
			AluSrc = 1;
			RegDst = 0;
			Aluctr[3:0] = 4'b0010;			   ////////SRL          
			MemtoReg = 0;
			RegWrite = 1;
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			ExtOp = 0;     
		end

	  else if(Instruction[31:26]== 6'b000000 && Instruction[10:0] == 11'b00000000110) 
		begin
			ra = Instruction[20:16];
			rb = Instruction[25:21];
			rw = Instruction[15:11];
			AluSrc = 0;
			RegDst = 1;
			Aluctr[3:0] = 4'b0010;     		//////SRLV                                          
			MemtoReg = 0;
			RegWrite = 1;
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			ExtOp = 0;     
		end

	else if(Instruction[31:26] == 6'b001110)
		begin
		   ra = Instruction[25:21];
			rb = Instruction[20:16];
			AluSrc = 1;
			RegDst = 0;
			Aluctr[3:0] = 4'b0111;      ////XORI
			MemtoReg = 0;
			RegWrite = 1;
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			ExtOp =0;			
		end
		
	else if(Instruction[31:26] == 6'b001101)
		begin
			ra = Instruction[25:21];
			rb = Instruction[20:16];
			AluSrc = 1;
			RegDst = 0;
			Aluctr[3:0] = 4'b0101;      ////ORI
			MemtoReg = 0;
			RegWrite = 1;
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			ExtOp =0;			
		end
	
	else if(Instruction[31:26] == 6'b001000)
	begin
			ra = Instruction[25:21];
			rb = Instruction[20:16];
			AluSrc = 1;
			RegDst = 0;
			Aluctr[3:0] = 4'b0000;      ////ADDI
			MemtoReg = 0;        
			RegWrite = 1;
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			ExtOp = 1;				
	end
	
	else if(Instruction[31:26] == 6'b001000)
	begin
		   ra = Instruction[25:21];
			rb = Instruction[20:16];
			AluSrc = 1;
			RegDst = 0;
			Aluctr[3:0] = 4'b0100;      ////ANDI
			MemtoReg = 0;
			RegWrite = 1;
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			ExtOp = 0;
	end	

	else if(Instruction[31:26] == 6'b001010)
	begin
		   ra = Instruction[25:21];
			rb = Instruction[20:16];
			AluSrc = 1;
			RegDst = 0;
			Aluctr[3:0] = 4'b1100;      ////SLTI(straight comparision)
			MemtoReg = 0;
			RegWrite = 1;
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			ExtOp = 1;
	end

	else if(Instruction[31:26] == 6'b000100)
		begin
			ra = Instruction[25:21];
			rb = Instruction[20:16];
			AluSrc = 0;
			Aluctr[3:0] = 4'b0001;     //////BEQ
			RegWrite = 0;		
			MemWrite = 0;		
			Branch = 1;
			Jump = 0;
			beq_offset = Instruction[15:0];
		end
	
	else if(Instruction[31:26] == 6'b100000)    
		begin		
			MemtoReg = 1;	
			rw = Instruction[20:16];
			ra = Instruction[25:21];
			RegDst = 1;										
			RegWrite = 1;                          ///LB;
			AluSrc = 1;										
			ExtOp = 1;									
			Jump =0;										
			Branch= 0;										
			Aluctr[3:0] = 4'b0000;						
		end
	
	else if(Instruction[31:26] == 6'b101000)       
		begin
			MemtoReg = 0;
			ra = Instruction[25:21];
         rb = Instruction[20:16];		  
			AluSrc = 1;
			MemWrite = 1;
			RegWrite = 0;									///SB							
			ExtOp = 1;											
			Jump = 0;										
			Branch = 0;											
			Aluctr[3:0] = 4'b0000;						
		end
	
	else if(Instruction[31:26] == 6'b000011)		
		begin
			rw = 5'b11111;
			RegDst = 1;
			RegWrite = 1;									
			AluSrc = 0;											////JAL
         Jump = 1;										
			Branch = 0;
			Aluctr[3:0] = 4'b0000;
			target_addr = Instruction[25:0];
		end													
	
	else if(Instruction[31:26] == 6'b000010)
		begin
			RegWrite =0;
			MemWrite = 0;                      			////Jump
			Branch = 0;
			Jump = 1;
			target_addr = Instruction[25:0];
		end	
	
	else if(Instruction[31:26] == 6'b000111)
		begin
			ra = Instruction[25:21];
			rb = Instruction[20:16];
			AluSrc = 0;
			RegWrite = 0;
			MemWrite = 0;
			Jump = 0;
			Branch = 1;
			Aluctr[3:0] = 4'b1110;							//BGTZ
			beq_offset = Instruction[15:0];
		end

	else if(Instruction[31:26] == 6'b000101)  
		begin 
			ra = Instruction[25:21];
			rb = Instruction[20:16];
			AluSrc = 0;
			Aluctr[3:0] = 4'b1111;    					 ///BNE
			RegWrite = 0;		
			MemWrite = 0;		
			Branch = 1;
			Jump = 0;
			beq_offset = Instruction[15:0];
	   end
	
	else if(Instruction[31:26] == 6'b000110)
		begin
			ra = Instruction[25:21];
			rb = 5'b00000;
			RegWrite = 0;
         MemWrite = 0;
			AluSrc = 0;
			Jump = 0;										//BLEZ
			Branch = 1;
			Aluctr[3:0] = 4'b1000;
			beq_offset = Instruction[15:0];
     end
	  
	else if(Instruction[31:26] == 6'b00000)
		begin
			AluSrc = 0;
			MemtoReg = 0;
			RegWrite = 1;	
			MemWrite = 0;
			Branch = 0;
			Jump = 0;
			RegDst = 1;
			ra = Instruction[25:21];
			rb = Instruction[20:16];
			rw = Instruction[15:11];
			
			if(Instruction[10:0]==11'b00000100000)
				begin
				Aluctr[3:0] = 4'b0000;         ////ADD
				end
				
			else if(Instruction[10:0]==11'b00000100010)
				begin	
				Aluctr[3:0] = 4'b0001;        /////SUB
				end
			else if(Instruction[10:0]==11'b00000100100)
				begin	
				Aluctr[3:0] = 4'b0100;        //////AND
				end

			else if(Instruction[10:0]==11'b00000100101)
				begin	
				Aluctr[3:0] = 4'b0101;         /////OR
				end

			else if(Instruction[10:0]==11'b00000100111)
				begin	
				Aluctr[3:0] = 4'b0110;         /////NOR
				end

			else if(Instruction[10:0]==11'b00000100110)
				begin	
				Aluctr[3:0] = 4'b0111;         /////XOR
				end			
			
			else if(Instruction[10:0]==11'b00000000111)
			begin
				ra = Instruction[20:16];
				rb = Instruction[25:21];
				Aluctr[3:0] = 4'b1001;          ///SRAV right shift
			end
			
			else if(Instruction[10:0]==11'b00000101010)
			begin
				Aluctr[3:0] = 4'b1100;         ///SLT (comparision)
			end
			
			else if(Instruction[10:0]==11'b00000101011)
			begin
				Aluctr[3:0] = 4'b1101;         ///SLTU (comparision) 
			end
			
			else if(Instruction[10:0]==11'b00000000100)
			begin
				ra = Instruction[20:16];
				rb = Instruction[25:21];
				Aluctr[3:0] = 4'b0011;        //// SLLV (arithmetic left shift)
			end
		
		end
end

endmodule

///// All checked

module main(
    input clk,
	 output reg[31:0]ans,
	 output wire[31:0]Instruction,
	 output wire[31:0]ALUoutput,
	 output reg[31:0]output1,
	 output wire [31:0]busA,
	 output wire [31:0]busB,
	 output wire [4:0]ra,
	 output wire [4:0]rb,
	 output reg [31:0]pipeline_reg,
	 output wire ExtOp,
	 output wire AluSrc,
	 output wire zero,
	 output wire [15:0]beq_offset,
	 output wire branch,
	 output wire [25:0]target_addr
	 );
	
	reg [7:0] instr_mem [0:127];
	reg [31:0] data_memory [0:31];
	reg [31:0] reg_memory [0:31];
	reg [31:0] PC;

	// INSTRUCTION MEMORY AND INITIALIZATION
	initial begin

	instr_mem[0] = 8'b00100000;
	instr_mem[1] = 8'b01010000;   //ADD
	instr_mem[2] = 8'b10100110;
	instr_mem[3] = 8'b00000000;

	instr_mem[4] = 8'b00000100;
	instr_mem[5] = 8'b00000000;  	//ADDI
	instr_mem[6] = 8'b01000111;    
	instr_mem[7] = 8'b00100001;

	instr_mem[8] = 8'b00100100;
	instr_mem[9] = 8'b01000000;    //AND
	instr_mem[10] = 8'b10000110;
	instr_mem[11] = 8'b00000000;

	instr_mem[12] = 8'b00000100;
	instr_mem[13] = 8'b00000000;   //ANDI
 	instr_mem[14] = 8'b10000111;
	instr_mem[15] = 8'b00110000;

	instr_mem[16] = 8'b00100111;
	instr_mem[17] = 8'b00111000;   //NOR
	instr_mem[18] = 8'b01001100;
	instr_mem[19] = 8'b00000001;
	
	instr_mem[20] = 8'b00100101;
	instr_mem[21] = 8'b00111000;   //OR
	instr_mem[22] = 8'b01001100;
	instr_mem[23] = 8'b00000001;

	instr_mem[24] = 8'b00000101;
	instr_mem[25] = 8'b00000000;
	instr_mem[26] = 8'b01100101;
	instr_mem[27] = 8'b00110100;	 //ORI	

	instr_mem[28] = 8'b00000100;
	instr_mem[29] = 8'b00101000;   //SLLV
	instr_mem[30] = 8'b00100111;
	instr_mem[31] = 8'b00000000;	
	
	instr_mem[32] = 8'b11000000;
	instr_mem[33] = 8'b01010000;
	instr_mem[34] = 8'b00001111;   //SLL
	instr_mem[35] = 8'b00000000;	

	instr_mem[36] = 8'b01000011;
	instr_mem[37] = 8'b00001000;   //SRA
	instr_mem[38] = 8'b00001001;
	instr_mem[39] = 8'b00000000;				
			
			
	instr_mem[40] = 8'b00000111;
	instr_mem[41] = 8'b01111000;   //SRAV
	instr_mem[42] = 8'b01001000;
	instr_mem[43] = 8'b00000000;

	instr_mem[44] = 8'b10000010;   //SRL
	instr_mem[45] = 8'b00101000;
	instr_mem[46] = 8'b00000100;
	instr_mem[47] = 8'b00000000;
	
	instr_mem[48] = 8'b00000110;
	instr_mem[49] = 8'b00111000;
	instr_mem[50] = 8'b01000110;
	instr_mem[51] = 8'b00000000; 	///SRLV
	
	instr_mem[52] = 8'b00100010;
	instr_mem[53] = 8'b01100000;
	instr_mem[54] = 8'b01001011;  //SUB
	instr_mem[55] = 8'b00000001;
	
	instr_mem[56] = 8'b00100110;
	instr_mem[57] = 8'b01010000; ////XOR
	instr_mem[58] = 8'b10001011;
	instr_mem[59] = 8'b00000001;
				
	instr_mem[60] = 8'b00000101; ///XORI
	instr_mem[61] = 8'b00000000;
	instr_mem[62] = 8'b01101101;
	instr_mem[63] = 8'b00111010;
	
	instr_mem[64] = 8'b00101010;
	instr_mem[65] = 8'b01111000;
	instr_mem[66] = 8'b10100111;
	instr_mem[67] = 8'b00000000;  //SLT
	
	instr_mem[68] = 8'b00101011;
	instr_mem[69] = 8'b00101000; 	//SLTU
	instr_mem[70] = 8'b01110111;
	instr_mem[71] = 8'b00000011;
 
	instr_mem[72] = 8'b11110000;
	instr_mem[73] = 8'b11111111;
	instr_mem[74] = 8'b11101110;	//SLTI
	instr_mem[75] = 8'b00101011;
	
	instr_mem[76] = 8'b11111101;
	instr_mem[77] = 8'b11111111;
	instr_mem[78] = 8'b00100100; 	//BEQ
	instr_mem[79] = 8'b00010000;
	
	instr_mem[80] = 8'b00100000;
	instr_mem[81] = 8'b00000000;  //ADD
	instr_mem[82] = 8'b00000000;
	instr_mem[83] = 8'b00000000;
	
	reg_memory[0] = 0;
	reg_memory[1] = 1;
	reg_memory[2] = 2;
	reg_memory[3] = 3;
	reg_memory[4] = 4;
	reg_memory[5] = 5;
	reg_memory[6] = 6;
	reg_memory[7] = 7;
	reg_memory[8] = 8;
	reg_memory[9] = 9;
	reg_memory[10] = 10;
	reg_memory[11] = 11;       ///////////Initializing registers
	reg_memory[12] = 12;
	reg_memory[13] = 13;
	reg_memory[14] = 14;
	reg_memory[15] = 15;
	reg_memory[16] = 16;
	reg_memory[17] = 17;
	reg_memory[18] = 18;
	reg_memory[19] = 19;
	reg_memory[20] = 20;
	reg_memory[21] = 21;
	reg_memory[22] = 22;
	reg_memory[23] = 23;
	reg_memory[24] = 24;
	reg_memory[25] = 25;
	reg_memory[26] = 26;
	reg_memory[27] = 27;
	reg_memory[28] = 28;
	reg_memory[29] = 29;
	reg_memory[30] = 30;
	reg_memory[31] = 31;
	

	data_memory[0] = 0;
	data_memory[1] = 10;
	data_memory[2] = 20;         ////////////////////data memory
	data_memory[3] = 30;
	data_memory[4] = 40;

	pipeline_reg = 32'bx;
	PC = 32'b0;
	
	end

	wire [29:0] PC_add,PC_jump;
	reg [29:0] PC_1,PC_2;
	wire jump;

	//FETCH UNIT
	assign PC_jump = {PC[31:28],target_addr[25:0]};
	assign PC_add = PC[31:2] + 30'd1;

	always @(PC_add or branch or zero)
		begin
			if(branch == 1'b1 & zero == 1'b1)
				begin
					PC_1 = PC_add + {{14{beq_offset[15]}},beq_offset[15:0]} - 30'd1;
				end
			else
				begin
					PC_1 = PC_add;
				end
		end

	always @(PC_jump or PC_1 or jump)
			begin
				if(jump==1'b1)
					begin
						PC_2=PC_jump;
					end
				else
					begin
						PC_2=PC_1;
					end
			end

	always @(negedge clk)
		begin
			pipeline_reg<={instr_mem[PC+3],instr_mem[PC+2],instr_mem[PC+1],instr_mem[PC]};
			PC [31:0] <={PC_2,{2{1'b0}}};
		end

	assign Instruction = pipeline_reg;
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

wire RegDst;
wire MemtoReg;
wire RegWrite;
wire MemWrite;
wire Jump;
wire [3:0]Aluctr;
wire overflow;
wire [4:0]rw;
wire [4:0]ru;

controlpath module2(.Instruction(pipeline_reg), .RegDst(RegDst), .AluSrc(AluSrc), .MemtoReg(MemtoReg), .RegWrite(RegWrite), .MemWrite(MemWrite), .Branch(branch), .Jump(Jump), .ExtOp(ExtOp), .Aluctr(Aluctr) , .ra(ra), .rb(rb), .rw(rw) , .beq_offset(beq_offset) , .target_addr(target_addr));


reg [31:0]extended;
reg [31:0]finalOutput;
assign busA[31:0] = reg_memory[ra][31:0];
assign busB[31:0] = reg_memory[rb][31:0];

//	if(pipeline_reg[31:26] == 6'b000011)
//		begin
//			assign busA = PC;                                                                      /////////////JAL store
///			assign busB = {{6{pipeline_reg[25]}},pipeline_reg[25:0]} + 1;
//		end	

always@(pipeline_reg or ExtOp)
	begin
		
	if(pipeline_reg[31:21]== 11'b00000000000 && pipeline_reg[5:0] == 6'b000011)	
		begin
			extended[31:0] = {{27{0}} , pipeline_reg[10:6]};         ///SRA shifting
		end
	
	else if(pipeline_reg[31:21]== 11'b00000000000 && pipeline_reg[5:0] == 6'b000000)
		begin
			extended[31:0] ={{27{0}} , pipeline_reg[10:6]};       	////SLL
		end
	
	else if(pipeline_reg[31:21]== 11'b00000000000 && pipeline_reg[5:0] == 6'b000010)
		begin
			extended[31:0] ={{27{0}} , pipeline_reg[10:6]};       	////SRL
		end
	
	else
		begin
			if(ExtOp==1)
				begin
					extended[31:0] = {{16{pipeline_reg[15]}},pipeline_reg[15:0]};
				end
			else
				begin
					extended[31:0] = {{16{1'b0}},pipeline_reg[15:0]};	  
				end
		end	  		
end

always@(busB or extended or AluSrc)
begin
		if(AluSrc==0)
			begin
				output1[31:0] = busB[31:0];
			end
		else
			 begin 
				output1[31:0] = extended[31:0];
			 end
end
	
ALU module3(ALUoutput,Aluctr,busA,output1,zero,overflow);

always@(negedge clk) 
	begin
		if(MemWrite==1)
			begin
				data_memory[ALUoutput] =  reg_memory[rb][7:0];               /////////last 8 bits of the register
			end
	end

always@(ALUoutput or MemtoReg)
	begin		
		if(MemtoReg==0)
			begin
				 finalOutput[31:0] = ALUoutput[31:0];
			end
		else
			begin
				 finalOutput = {{26{data_memory[ALUoutput][7]}},{data_memory[ALUoutput]}};   /////Sign extension in LB
			end
	end

assign ru = ({5{(!RegDst)}}& rb) | ({5{RegDst}} & rw); 

always@(negedge clk)
begin
	if(RegWrite == 1)
		begin	 
			reg_memory[ru][31:0] = ALUoutput[31:0];
		   ans[31:0] = reg_memory[ru][31:0];
		end	
end	
		
endmodule

