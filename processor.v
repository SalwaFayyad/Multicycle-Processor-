// ENCS4370| Computer Architecture
// Second Semester, 2023/2024
// Project No. 2
// Group Of :
// Sami Moqbel 1200751
// Lama Abdelmuhsen 1201138
// Salwa Fayyad 1200430


`define  IF 3'b000  //Fetch 
`define  ID 3'b001	//Decode  
`define  EX 3'b010	//Execute
`define  MEM 3'b011	//Memory
`define  WB 3'b100	//WriteBack	


///////Define the ALU operation codes
`define  AND 2'b00  //AND Operation 
`define  ADD 2'b01	//ADD Operatoion  
`define  SUB 2'b10	//SUB Operation


///Multi-Cycle Processor
module MultiCycleProcessor (clk, reset, zeroFlag, overflowFlag, negativeFlag, ALUOut, dataOut, WBData);  
		// input to our processor (just we need clock and reset to reintiate the values)
		input clk, reset;
		
		// output from our processor which used for waveform
		output reg zeroFlag, overflowFlag, negativeFlag; // output flags from ALU (which needed to check branch taken or not taken)
		output reg signed [15:0] ALUOut;// ALU output
		
		// all needed signals for main control unit
		reg RegW, ALUsrc, ExtOp, MemR, MemW, DataWB, ExtLoad, memOut, ExtImm ;
		
		// signal for ALU control unit
		reg [1:0]ALUOp; // two signals for define the ALU operation --> +, - or &
			
		reg signed [15:0] Data_Memory[0:127]; // 16-bit address memory, also this size is temporary and can be changed 
		
		reg signed [15:0] fullData; 
		reg signed [8:0] byteData; 
			
		// define the Instruction Memory
		// the actual Instruction Memory
		reg [15:0] Memory[0:127]; // 16-bit address memory, its size = 128 byte and it can be more or less (as computer spec)
		
		//define the special purpose registers --> PC & SP
		reg [15:0] PC;
		reg [15:0] NPC;// save next PC used for call instructon this is used for CALL instruction (variable to save the next value of PC because the original value will be updated)
	
			
		reg [2:0] PState, NState;// states used in FSM (SHOULD ASK ABOUT)
		
		
		
		reg [15:0] IR; // instruction register used to save the readed instruction obtained from instruction memory
	
		
		/////Define the addresses of the registers being used in Register File
		reg [2:0] Rdr;    //Destination address For R-Type
		reg [2:0] Rdi;    //Destination address For I-Type		
		reg [2:0] Rd;    //Destination address
		reg [2:0] Rs1r;	 //Source1 R-Type address
		reg [2:0] Rs1i;	 //Source1 I-Type address
		reg [2:0] Rs2;	 //Source2 address	
		reg [4:0] immI; // I-Type Immediate
		reg [11:0] immJ; // J-Type Immediate
		reg [11:0] jOffset; // J-Type Offset
		reg [7:0] immS; // S-Type Immediate
		
		
		reg signed [15:0]extendedImm; // the output for EXtender 
		
		// Define the Register File
		reg signed [15:0] Registers[0:7]; // 8 16-bit general purpose registers 
		reg signed [15:0] BusA, BusB;// output form Register File
		reg signed [15:0] BusW, BusWA;// Write buses for Register File
			
		// variable to save address used in memory, data in and data out from memory	
		reg signed [7:0] Memaddress; // ( SHOULD BE CHECKED )
		reg signed [15:0] dataIn;
		output reg signed [15:0] dataOut;
		output reg signed [15:0] WBData; // Data be written to Rigster file (last MUX)
		
	
		integer i; // used to reinitiate the values saved in registers when there is reset input
		initial 
		begin
			PC = 15'b0; // the first address for PC starting from address zero 
			NState = `IF; // the first cycle for program is fetch of course 
			for (i = 0; i <= 7; i = i + 1) // reset all values for registers 
		 		Registers[i] = 15'h0000;
			
			$display("\n****************MultiCycle Procceser****************\n");
			
						// program instrution Memory
				Memory[0] = 16'b0000011001010000; // AND R-Type (R[3] = R[1] & R[2]) 
				Memory[1] = 16'b0001011001010000; // ADD R-Type (R[3] = R[1] + R[2])
				Memory[2] = 16'b0010011001010000; // SUB R-Type (R[3] = R[1] - R[2])

				Memory[3] = 16'b0100001100100001; // ANDI I-Type (R[3] = R[1] & 1)
				Memory[4] = 16'b0011001100100001; // ADDI I-Type (R[3] = R[1] + 1)
				Memory[5] = 16'b0101010000100001; // LW I-Type 	(R[4] = Mem(R[1] + 1)
	
				Memory[6] = 16'b0110010000100010; // LBu I-Type  (R[4] = Mem(R[1] + 2)	MODE!!
				Memory[7] = 16'b0110110000100010; // LBs I-Type  (R[4] = Mem(R[1] + 2)	MODE!!
	
				Memory[8] = 16'b0111001100100011; // SW I-Type 	 Mem(R[1] + 3) = R[3]
	
				Memory[9] =  16'b1000001100100010; // BGT I-Type 	R[3] > R[1] MODE!	 if Yes -> Pc = Pc + Imm	else Pc = PC + 1
				Memory[11] = 16'b1000101100100010; // BGTZ I-Type 	 R[3] > R[0] MODE! 	 if Yes -> Pc = Pc + Imm	else Pc = PC + 1

				Memory[13] = 16'b1001001111000010; // BLT I-Type 	R[3] < R[6] MODE!	 if Yes -> Pc = Pc + Imm	else Pc = PC + 1
				Memory[15] = 16'b1001101111000010; // BLTZ I-Type 	R[3] < R[0] MODE!	 if Yes -> Pc = Pc + Imm	else Pc = PC + 1
	
				Memory[16] = 16'b1010001100100010; // BEQ I-Type 	R[3] == R[1] MODE!	 if Yes -> Pc = Pc + Imm	else Pc = PC + 1
				Memory[17] = 16'b1010101100100010; // BEQZ I-Type 	R[3] == R[0] MODE!	 if Yes -> Pc = Pc + Imm	else Pc = PC + 1
	
				Memory[18] = 16'b1011001100100010;	// BNE I-Type 	R[3] != R[1] MODE!	 if Yes -> Pc = Pc + Imm	else Pc = PC + 1
				Memory[20] = 16'b1011101100100010; // BNEZ I-Type 	R[3] < R[0] MODE!	 if Yes -> Pc = Pc + Imm	else Pc = PC + 1
	
				Memory[22] = 16'b1100000000001101; // JMP J-Type  PC = {PC[15,12], Imm * 2 in Binary}
				Memory[26] = 16'b1101000000010001; // CALL J-Type  PC = {PC[15,12], Imm * 2 in Binary} , R[7] = PC + 1 
				Memory[27] = 16'b1111001000000100; // Sv S-Type Memory[1] = 2
				Memory[28] = 16'b0000011001010000; // AND R-Type (R[3] = R[1] & R[2]) 
				Memory[34] = 16'b0001000001010000; // ADD R-Type (R[3] = R[1] + R[2])
				Memory[35] = 16'b1110000000000001; // RET J-Type PC= R[7]
	
							   
				//Data
				Data_Memory[1] = 16'b0000000000000101; /// 5 in decimal  
				Data_Memory[2] = 16'b0000000000001001;	//// 9 in decimal		 
				Data_Memory[3] = 16'b0000000110000001;	//// 385 in decimal	
				Data_Memory[4] = 16'b0000000000001101;	//// 13 in decimal	
				Registers[0] = 16'd0;  // MUST BE SET TO 0
				Registers[1] = 16'd1;
				Registers[2] = 16'd1;
				Registers[5] = 16'd0;
				Registers[6] = 16'd10;

				
		
		end	   
		
		
		always @(posedge clk, posedge reset) begin
			if (reset)	
				begin //empty all register values and make state to IF stage  
					$display("\n------------------Reset Program------------------\n");
					NState = `IF; ///Initialize the next state
					PC = 15'h0000;
					Rs1r = 15'd0; 
					Rs1i = 15'd0; 
					Rs2 = 15'd0;
					Rdr = 15'd0;
					Rdi = 15'd0;  
					Rd = 15'd0;
					
					for (i = 0; i <= 7; i = i + 1)
		 			 	Registers[i] = 15'h0000;
				end
			else
				begin	 
					
							PState = NState;
							
							case(PState)
				//------------------------------------IF-----------------------------------------
								`IF: 
									begin
										$display("\n------------------Fetch Cycle------------------\n");
										$display("PC=%d\t", PC);
										InstructionMemory(PC,IR); // get instruction from instruction memory
										NState=`ID; // change the state instruction
										  
										
									end
									
				//------------------------------------ID-----------------------------------------
								`ID:
									begin
										$display("\n------------------Decode Cycle------------------\n");
										// Generate Main Control signals
										MainControl(IR[15:12]);	  
										
										$display("Signals:\n RegW = %b,  ALUsrc = %b, ExtOp = %b,   MemR = %b, MemW = %b, DataWB = %b, memOut = %b, ExtLoad = %b, ExtImm = %b\n", RegW,  ALUsrc, ExtOp,  MemR, MemW, DataWB, memOut, ExtLoad, ExtImm);
							
										//Case Of R-Type
										Rdr = IR[11:9]; // It is used as Rd for R-Type, and Could be used as Rs for S-Type
										Rs1r = IR[8:6];
										Rs2 = IR[5:3];
										
										//Case Of I-Type
										Rdi = IR[10:8];
										Rs1i = IR[7:5];
										immI = IR[4:0];
										
										
										//Case of J-Type
										immJ = IR[11:0];
										jOffset = immJ << 1;
										
										//Case of S-Type
										immS = IR[8:1];
										
										
										if(IR[15:12] == 4'b1111)	 //Then its S-Type (ExtImm Signal)
											begin  
												// Extend S-Imm	
												Extender2(immS, ExtOp, extendedImm);
												$display("immS, extended S Imm = %b", extendedImm);
											end
										else
											begin
												// Extend I-Imm	
												Extender1(immI, ExtOp, extendedImm);
											end
											
										// Read from Register File According to Its Type
										
										if(IR[15:12] ==4'b0000 || IR[15:12] ==4'b0001 || IR[15:12] ==4'b0010)
											// R-type signals (AND, ADD, SUB)
											begin
												RegsiterFileRead(Rs1r,Rs2, BusA, BusB);
											end
										else if(IR[15:12] ==4'b0011 || IR[15:12] ==4'b0100 || IR[15:12] ==4'b0101 || 
													IR[15:12] ==4'b0110 || IR[15:12] ==4'b0111 || IR[15:12] ==4'b1000 || 
													IR[15:12] ==4'b1001 || IR[15:12] ==4'b1010 || IR[15:12] ==4'b1011)
											begin			 
												RegsiterFileRead(Rdi,Rs1i, BusA, BusB);
											end
										else
											begin
												RegsiterFileRead(Rs1i,Rs2, BusA, BusB);
											end
										
										
											
										// ******* Instruction dont need to go to execute stage (J-type)
										if(IR[15:12] == 4'b1100) // JMP instruction need 2 cycles
											begin
												
												PC = {PC[15:12], jOffset}; //JTA then to fetch stage
												NState=`IF;	
											end
										else if(IR[15:12] == 4'b1101) // CALL instruction need to go to EX stage
											begin
												NPC = PC + 1;
												PC = {PC[15:12], jOffset};
												NState=`EX;	
											end
										else if(IR[15:12] == 4'b1110) // RET instruction need to go to Memory stage
											NState=`EX;
										else // other instrucations need to go to Execute stage
											NState = `EX;		
									end
				//------------------------------------EX-----------------------------------------			
								`EX:
									begin			
										$display("\n------------------Execute Cycle------------------\n");
										//$display("Extended immediate=%2d, BusA=%2d, BusB=%2d\n", extendedImm, BusA, BusB);
										if (ALUsrc == 1'b0)
											ALU(ALUOp, BusA, BusB, ALUOut, zeroFlag, negativeFlag, overflowFlag);
										else
											if(IR[15:12] == 4'b0111) // Sw
												begin
													ALU(ALUOp, BusB, extendedImm, ALUOut, zeroFlag, negativeFlag, overflowFlag);
												end	
											else
												begin
													ALU(ALUOp, BusA, extendedImm, ALUOut, zeroFlag, negativeFlag, overflowFlag);
												end
										
										
										
										if (IR[15:12] <= 4'b0100) // R-Type, ADDI, ANDI instructions
											NState = `WB;
										else if (IR[15:12] == 4'b0101 || IR[15:12] == 4'b0110 || IR[15:12] == 4'b0111) // LW,LBu, LBs SW instructions
											NState = `MEM;
										else if (IR[15:12] >= 4'b1000 && IR[15:12] <= 4'b1011) // Branch instructions
											NState = `IF;
										else if (IR[15:12] == 4'b1101) // CALL
											begin
												ALUOut = NPC;
												NState = `WB;
											end
										else if (IR[15:12] == 4'b1110) // RET 
											begin
												PC = NPC;
												NState = `IF;
											end
										else if (IR[15:12] == 4'b1111 ) // Sv
											begin
												NState = `MEM;
											end
												
										
										// check conditoins for branch instructions
										// BEQ instruction
										if (IR[15:12] == 4'b1010) 
											begin
												if (zeroFlag == 1'b1) 
													PC = PC + extendedImm; //branch taken --> PC = BTA
												else
													PC = PC + 1;
											end	  
										
										// BNE instruciton
										else if (IR[15:12] == 4'b1011) 
											begin
												if (zeroFlag == 1'b0) 
													PC = PC + extendedImm; //branch taken --> PC = BTA
												else
													PC = PC + 1; //branch not taken --> PC = PC + 1
											end
										
										// BGT instruction
										else if (IR[15:12] == 4'b1000) 
											begin
												if ((negativeFlag != 1'b1) && (zeroFlag != 1'b1))
													PC = PC + extendedImm; //branch taken --> PC = BTA
												else
													PC = PC + 1;
											end
										
										// BLT instruction
										else if (IR[15:12] == 4'b1001) 
											begin
												if ((negativeFlag == 1'b1)) // Rd < Rs1 same as --> Rs1 > Rd because the first operand in ALU remain Rs1 
													PC = PC + extendedImm;							   
												else
													PC = PC + 1;
											end 
									end // end EX
				//------------------------------------MEM-----------------------------------------				
								`MEM: 
									begin
										$display("\n------------------Memory Cycle------------------\n");
										
										Memaddress = ALUOut;
										
										if(IR[15:12] == 4'b0111)
											begin			   
												dataIn = BusA; 
											end
										else
											begin
												dataIn = BusB; 
											end
											
										
															
										$display("ALUOut= %d, Memaddress= %d, dataIn = %d\n",ALUOut, Memaddress, dataIn);
										
										if(IR[15:12] == 4'b1111)  //Sv
											begin  
												dataMem(Rdr, extendedImm, MemR, MemW, dataOut);	
											end
										else if(IR[15:12] == 4'b0111)	 //Sw
											begin
												dataMem(Memaddress, dataIn, MemR, MemW, dataOut);	
											end
										else			 // Lw
											begin
												dataMem(Memaddress, extendedImm, MemR, MemW, dataOut);	
											end
											
										
										
									
										if(IR[15:12] == 4'b0111 || IR[15:12] == 4'b1111 ) // SW, Sv need to go to fetch
											begin
												PC = PC + 1;
												NState = `IF;
											end
										else //LW, LBu, LBs -> WB
											NState = `WB;			
									end
									
				//------------------------------------WB-----------------------------------------
								`WB: 
									begin
										$display("\n------------------Write Back Cycle------------------\n");
										
										if(DataWB==0)
											WBData = ALUOut;
										else
											WBData = dataOut;
										
										if(IR[15:12] == 4'b0010 || IR[15:12] == 4'b0001 || IR[15:12] == 4'b0000) // R-Type	 
											begin
												Rd = Rdr;
											end
										else if(IR[15:12] == 4'b1101) 
											begin
												Rd = 3'b111;
											end     
										else	
											begin
												Rd = Rdi;  
											end
										if (IR[15:12] == 4'b1101)	   
											begin
												RegsiterFileWrite(Rd, RegW, NPC);	
						
										        NState = `IF;
											end	  
									else
										begin
										RegsiterFileWrite(Rd, RegW, WBData);
										PC = PC + 1;
										NState = `IF;
										end												  
											
									end	// end WB 
							endcase
						end // end else
					end // end always			
				
				
	// -------------------- InstructionMemory --------------------------
	task InstructionMemory(input [15:0] instAdd, output reg [15:0] instOut);
		
		instOut = Memory[instAdd];
		
		$display("Instruction = %b\n", instOut);
		$display("R0 = %d, R1 = %d, R2 = %d\n", Registers[0], Registers[1], Registers[2]);
		$display("R3 = %d, R4 = %d, R7 = %d\n", Registers[3], Registers[4], Registers[7]);
		$display("M1 = %d, M4 = %d\n", Data_Memory[1], Data_Memory[4]);
	endtask			
				
	
	// -------------------- Main Control --------------------------
	task MainControl(input [3:0] opcode);
		if(opcode==4'b0000 || opcode==4'b0001 || opcode==4'b0010)
			// R-type signals (AND, ADD, SUB)
			begin
				RegW = 1'b1;
				ALUsrc = 1'b0;
				MemR = 1'b0;
				MemW = 1'b0;
				DataWB = 'b0;
				if(opcode==4'b0000) // AND
					ALUOp = `AND;
				else if(opcode==4'b0001) // ADD
					ALUOp = `ADD;
				else // SUB
					ALUOp = `SUB;
			end
		// I-type signals
		else if(opcode==4'b0011 || opcode==4'b0100 || opcode==4'b0101 || 
				opcode==4'b0110 || opcode==4'b0111 || opcode==4'b1000 || 
				opcode==4'b1001 || opcode==4'b1010 || opcode==4'b1011)
				begin
				ExtImm = 1'b0;
					if(opcode==4'b0011) // ADDI
						begin
							RegW = 1'b1;
							ALUsrc = 1'b1;
							ExtOp = 1'b1;
							MemR = 1'b0;
							MemW = 1'b0;
							DataWB = 'b0;
							ALUOp = `ADD;
						end
					else if(opcode==4'b0100) // ANDI
						begin
							RegW = 1'b1;
							ALUsrc = 1'b1;
							ExtOp = 1'b0;
							MemR = 1'b0;
							MemW = 1'b0;
							DataWB = 'b0;
							ALUOp = `AND;
						end
					else if(opcode==4'b0101 || opcode==4'b0110) // LW, LBu, LBs
						begin
							RegW = 1'b1;
							ALUsrc = 1'b1;
							MemR = 1'b1;
							MemW = 1'b0;
							DataWB = 'b1;
							ALUOp = `ADD;
							
							if(opcode==4'b0101) // Normal LW
								begin
									ExtOp = 1'b1;
									memOut = 1'b0;
								end
							else //LBu, LBs
								begin
									memOut = 1'b1;
									if(!IR[11])// if  mode = 0 then it is LBu
										begin
											ExtLoad = 1'b0;
										end
									else // Then It is LBs
										begin
											ExtLoad = 1'b1;
										end	
								end
							
							
						end
					else if(opcode==4'b0111) // SW
						begin
							RegW = 1'b0;
							ALUsrc = 1'b1;
							ExtOp = 1'b1;
							MemR = 1'b0;
							MemW = 1'b1;
							ALUOp = `ADD;
						end
					else // Branch instructions
						begin
							RegW = 1'b0;
							ALUsrc = 1'b0;
							ExtOp = 1'b1;
							MemR = 1'b0;
							MemW = 1'b0;
							ALUOp = `SUB;
						end	
				end
		// J-type signals
		else if(opcode==4'b1100 || opcode==4'b1101 || opcode==4'b1110)
			begin
				// common signals among all J-type instructions
				MemR = 1'b0;
				MemW = 1'b0;
				
				if(opcode==4'b1100) // JMP
					begin
						RegW = 1'b0;
					end
				else if(opcode==4'b1101) // CALL
					begin
						RegW = 1'b1;
					end 
				else // RET
					begin
						RegW = 1'b0;
						ALUsrc = 1'b0;	
					end			
			end
		// S-type signals
		else if(opcode==4'b1111)
			begin
				ExtImm = 1'b1;
				RegW = 1'b0;
				ALUsrc = 1'b1;
				ExtOp = 1'b1;
				MemR = 1'b0;
				MemW = 1'b1;
			end	
		
		
	endtask	
	
	
	///---------------------5 -> 16-bit EXTENDER---------------
	task Extender1(input [4:0]imm, input ext_op, output signed [15:0]outExt); 
		if(ext_op)
			begin
				if(imm[4]==1'b1)
					outExt={11'b11111111111,imm}; // the sign extended is ones
				else
					outExt={11'b0,imm};
			end
		else
			outExt={11'b0,imm};
		
	endtask			
	
	///---------------------8 -> 16-bit EXTENDER---------------
	task Extender2(input [7:0]imm, input ext_op, output signed [15:0]outExt); 
		if(ext_op)
			begin
				if(imm[7]==1'b1)
					outExt={8'b11111111,imm}; // the sign extended is ones
				else
					outExt={8'b0,imm};
			end
		else
			outExt={8'b0,imm};
		
	endtask					
	
	// -------------------- Register Read --------------------------			
	task RegsiterFileRead(input [2:0] Rs1, Rs2, output signed [15:0] BusA, BusB);
		BusA = Registers[Rs1];
		
		if (IR[15:12] >= 4'b1000 && IR[15:12] <= 4'b1011) // Branch instructions
			begin
				if(!IR[11])// if  mode = 0 then Compare With RS2
					begin
						BusB = Registers[Rs2];
					end
				else // Compare With R0
					begin
						BusB = Registers[0];
					end	
			end
		else if (IR[15:12] == 4'b0011 || IR[15:12] == 4'b0100 || IR[15:12] == 4'b0101 || 
				IR[15:12] == 4'b0110 )
			begin  
				BusA = Registers[Rs2];
			end
		else if (IR[15:12] == 4'b0111 )
			begin  
				BusB = Registers[Rs2];
			end			  
		else 
			begin BusB = Registers[Rs2]; end
		
		$display("Rs1 = %d, Rs2 = %d, BusA = %d, \t BusB = %d\n", Rs1, Rs2 , BusA, BusB );
			
	endtask			
				
				
				
	task ALU (input [1:0] ALUOp, input signed [15:0] op1, input signed [15:0] op2, output reg signed [15:0]result, output reg z, n, v);
	
		// compute result
		case(ALUOp)
			`AND: begin result = op1 & op2; end
			`ADD: begin result = op1 + op2; end
			`SUB: begin result = op1 - op2; end
		endcase
		
		// set flags
		z = result == 16'd0;
		n = result < 0;
		v = (op1[15] != op2[15]) && (result[15] != op1[15]);
	   
		if(IR[15:12] == 4'b1111)
			begin  
				result = op2;	
			end
		$display("op1= %2d, op2=%2d\n", op1, op2);
		$display("ALU result= %2d, z=%b, n=%b, v=%b\n", result, z, n, v);
	endtask				
				
				
				
	// -------------------- Data Memory --------------------------	
	task dataMem(input [15:0] Address, input signed [15:0] Data_in, input MemR, MemW, output signed [15:0] Data_out);
		
		if(IR[15:12] == 4'b0101 || IR[15:12] == 4'b0111 || IR[15:12] == 4'b1111 ) // LW, SW and Sv
			begin
				if(MemR==1'b1 && MemW==1'b0)
					Data_out=Data_Memory[Address];	
				else if(MemR==1'b0 && MemW==1'b1)
					Data_Memory[Address]=Data_in;
			end
		else //Lbu , LBs
			begin
				fullData = Data_Memory[Address];
				byteData = fullData[7:0];
				byteExtender(byteData, ExtLoad, Data_out);
			end
		
	endtask			
				
	task byteExtender(input [7:0] byteOut, input ExtLoad, output signed [15:0] loadOut);
	
		if(ExtLoad) // LBs
			begin
				if(byteOut[7]==1'b1)
					loadOut={8'b11111111,byteOut}; // the sign extended is ones
				else
					loadOut={8'b0,byteOut};
			end
		else // LBu
			loadOut={8'b0,byteOut};
		
	endtask					
				
	// -------------------- Register Write --------------------------
	task RegsiterFileWrite(input [2:0] Rd,  input RegW, input signed [15:0] BusW);
	
		if(RegW)
			Registers[Rd] = BusW;
		
		if(RegW) 	
			$display("The register R%0d updated new value = %2d\n", Rd, Registers[Rd]);
	endtask		


endmodule	 


module TestBench; // test bench for the final modul 	
 	//Declarations of test inputs and outputs 
 
	reg clk;// every component in our data path synchronous by clock signal (sequantial circite )(when positive edge comes all component sensetive for clk signal)
	reg reset;
	
	wire zeroFlag, overflowFlag, negativeFlag;
	wire [15:0] ALUOut, dataOut, WBData;
	
	
	MultiCycleProcessor MCP(clk, reset, zeroFlag, overflowFlag, negativeFlag, ALUOut, dataOut, WBData);	  


	initial clk = 0;	
	always #10 clk = ~clk;
	initial #2000 $finish; 			
endmodule
