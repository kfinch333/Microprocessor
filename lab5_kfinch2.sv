
//physical validation
module lab5_pv(input clk, SW0, SW1, KEY0, SW2, SW3, SW4, output logic [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0, output logic LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7 );
	logic clkout;
	logic [3:0] OPCODE;
	logic [7:0] PC, alu_out, W_REG, disp0, disp1, disp2, disp3, disp4, disp5;

	pmcntr #(25) p1 (clk, SW0, 25'd500000, count, clkout);
	lab5 mcu1 (clkout, SW0, SW1, KEY0, SW2, SW3, SW4, OPCODE, PC, alu_out, W_REG);

	ASCII27Seg d0 (disp0, HEX0);
	ASCII27Seg d1 (disp1, HEX1);
	ASCII27Seg d2 (disp2, HEX2);
	ASCII27Seg d3 (disp3, HEX3);
	ASCII27Seg d4 (disp4, HEX4);
	ASCII27Seg d5 (disp5, HEX5);

	always_ff @ (posedge clk) begin
		if (~SW4 & ~SW3 & ~SW2) begin
			disp5 = "f"; 
			disp4 = "i";
			disp3 = "n";
			disp2 = "c";
			disp1 = "h";
			disp0 = "_";
		end
		else if (SW4 & SW3 & ~SW2) begin
			disp5 = "_"; 
			disp4 = "_";
			disp3 = "_";
			disp2 = "_";
			disp1 = {4'd0,PC[7:4]};
			disp0 = {4'd0,PC[3:0]};
		end
		else if (SW4 & ~SW3 & SW2) begin
			disp5 = "_"; 
			disp4 = "_";
			disp3 = "_";
			disp2 = "_";
			disp1 = {4'd0,W_REG[7:4]};
			disp0 = {4'd0,W_REG[3:0]};
		end
		else if (~SW4 & SW3 & SW2) begin
			disp5 = "_"; 
			disp4 = "_";
			disp3 = "_";
			disp2 = "_";
			disp1 = {4'd0,alu_out[7:4]};
			disp0 = {4'd0,alu_out[3:0]};
		end
		else if (SW4 & SW3 & SW2) begin
			disp5 = "_"; 
			disp4 = "_";
			disp3 = {4'd0,OPCODE[3]};
			disp2 = {4'd0,OPCODE[2]};
			disp1 = {4'd0,OPCODE[1]};
			disp0 = {4'd0,OPCODE[0]};
		end
		else begin
			disp5 = "_"; 
			disp4 = "_";
			disp3 = "_";
			disp2 = "_";
			disp1 = "_";
			disp0 = "_";
		end
	end
	
	assign LED0 = PC[0];
	assign LED1 = PC[1];
	assign LED2 = PC[2];
	assign LED3 = PC[3];
	assign LED4 = PC[4];
	assign LED5 = PC[5];
	assign LED6 = PC[6];
	assign LED7 = PC[7];
endmodule	

////////////////////////////////////////////////////////////////////////

//TEST BENCH
module lab5_tb();
	logic clk, SW0, SW1, KEY0, SW2, SW3, SW4;
	logic [3:0] OPCODE;
	logic [7:0] PC, alu_out, W_REG;
	lab5 mcu1 (clk, SW0, SW1, KEY0, SW2, SW3, SW4, OPCODE, PC, alu_out, W_REG);
	int fd;

	initial begin
//		fd = $fopen("output.csv");
//		$fwrite(fd, "PC, IR, OPCODE, RA, RB, RD, A, B, RF[RD]\n");
		clk = 0; SW0 = 1; SW1 = 0; #5;
		SW0 = 0; #5;
		repeat (800) begin
			clk = ~clk; #5;
		end
//		$fclose(fd);
	end
endmodule

////////////////////////////////////////////////////////////////////////

//LAB 5
module lab5(input clk, SW0, SW1, KEY0, SW2, SW3, SW4, output logic [3:0] OPCODE, output logic [7:0] PC, alu_out, W_REG);
	logic [2:0] state;
	logic CLK;

	always_ff @ (*) begin
		if (~SW1)
			CLK <= clk;
		else begin
			if (~KEY0)
				CLK = 1'b1;
			else
				CLK = 1'b0;
		end
	end

	control c1 (CLK, SW0, OPCODE, state, PC, alu_out, W_REG);

endmodule

////////////////////////////////////////////////////////////////////////

//CONTROL
module control (input clk, reset, output logic [3:0] OPCODE, output logic [2:0] state, output logic [7:0] PC, alu_out, W_REG);
	logic [2:0] next_state;
	logic [3:0] RA, RB, RD;
	logic [7:0] A, B, temp_data, Rem;
	logic [15:0] IR;
	int fd;

	parameter IF = 3'b001;
	parameter ID = 3'b010;
	parameter FD = 3'b011;
	parameter EX = 3'b100;
	parameter RWB = 3'b101;
	parameter HLT = 3'b110;


	always_ff @ (posedge clk or posedge reset) begin
		if (reset) begin
			state <= 3'd1;
			PC <= 8'd0;
		end
		else begin
			if (state == RWB) begin
				if (OPCODE == 4'hd) begin
					PC <= temp_data;
				end
				else if (OPCODE == 4'he) begin
					if (A >= B) begin
						PC <= PC + RD;
					end
					else begin
						PC <= PC + 1;
					end
				end
				else begin
					if (PC == 8'h14) begin
						PC <= PC;
					end
					else
						PC <= PC + 1;
				end
			end 
			state <= next_state;
		end
	end

		always_comb begin
			next_state = 3'd0;
			case (state)
				IF: begin
					next_state = ID;
				end
				ID: begin
					next_state = FD;
				end
				FD: begin
					next_state = EX;
				end
				EX: begin
					next_state = RWB;
//					fd = $fopen("output.csv");
//					$fwrite(fd, "%h, %h, %h, %h, %h, %h, %h, %h, %h\n", PC, IR, OPCODE, RA, RB, RD, A, B, alu_out);
//					$fclose(fd);
				end
				RWB: begin
					if (OPCODE == 4'd0)
						next_state = HLT;
					else
						next_state = IF;
				end
				HLT: begin
					next_state = HLT;
				end
				default: begin
					next_state = state;
				end
			endcase
		end
	
	ROM rom1 (PC, IR);
	assign OPCODE = IR[15:12];
	assign RA = IR[12:8];
	assign RB = IR[7:4];
	assign RD = IR[3:0];
	assign temp_data = {RA,RB};

	RegFile rf1 (clk, reset, RA, RB, RD, OPCODE, state, W_REG, A, B);

	ALU alu1 (OPCODE, RA, RB, A, B, alu_out);

	wreg w1 (clk, reset, alu_out, W_REG);
endmodule

////////////////////////////////////////////////////////////////////////

//Instruction Register
module RegFile (input clk, reset, input [3:0] RA, RB, RD, OPCODE, input [2:0] current_state, input [7:0] RF_data_in, output logic [7:0] RF_data_out0, RF_data_out1);
		logic [7:0] RF [15:0];
		
		parameter IF = 3'b001;
		parameter ID = 3'b010;
		parameter FD = 3'b011;
		parameter EX = 3'b100;
		parameter RWB = 3'b101;
		parameter HLT = 3'b110;

		always_ff @ (posedge clk or posedge reset) begin
			if (reset) begin
				RF[0] <= 8'd0;
				RF[1] <= 8'd0;
				RF[2] <= 8'd0;
				RF[3] <= 8'd0;
				RF[4] <= 8'd0;
				RF[5] <= 8'd0;
				RF[6] <= 8'd0;
				RF[7] <= 8'd0;
				RF[8] <= 8'd0;
				RF[9] <= 8'd0;
				RF[10] <= 8'd0;
				RF[11] <= 8'd0;
				RF[12] <= 8'd0;
				RF[13] <= 8'd0;
				RF[14] <= 8'd0;
				RF[15] <= 8'd0;
			end
			else if ((current_state == RWB) && ~((OPCODE == 4'd13) || (OPCODE == 4'd14) || (OPCODE == 4'd15))) begin
					RF[RD] <= RF_data_in; 
	 		end
	 	end

		assign RF_data_out0 = RF[RA];
		assign RF_data_out1 = RF[RB];
endmodule

////////////////////////////////////////////////////////////////////////

//Instruction Memory
module ROM (input [7:0] PC, output [15:0] data);
	logic [15:0] mem [20:0];

	assign mem[0] = 16'h1000;
	assign mem[1] = 16'h1011;	
	assign mem[2] = 16'h1002;
	assign mem[3] = 16'h10A3;
	assign mem[4] = 16'hE236;
	assign mem[5] = 16'h2014;
	assign mem[6] = 16'h3100;
	assign mem[7] = 16'h3401;
	assign mem[8] = 16'h7022;
	assign mem[9] = 16'hD040;
	assign mem[10] = 16'h3405;
	assign mem[11] = 16'h6536;
	assign mem[12] = 16'h5637;
	assign mem[13] = 16'h4578;
	assign mem[14] = 16'h3828;
	assign mem[15] = 16'h8089;
	assign mem[16] = 16'h809A;
	assign mem[17] = 16'hB89B;
	assign mem[18] = 16'hA9AC;
	assign mem[19] = 16'hC0CD;
	assign mem[20] = 16'h0000;
	
	mux21a mux1 (mem, PC, data);
endmodule

//21:1 MUX
module mux21a (input [15:0] a [20:0], input [7:0] s, output [15:0] z);
	assign z = a[s];
endmodule

////////////////////////////////////////////////////////////////////////

//ALU
module ALU (input [3:0] OPCODE, RA, RB, input logic [7:0] A, B, output logic [7:0] alu_out);
	logic Cout, OF, Cinlastbit, Cout_mul, halt, neg;
	logic [7:0] Rem;

	always_comb begin
	alu_out = 8'd0; halt = 1'b0; Cout = 1'b0; OF = 1'b0; Cinlastbit = 1'b0; Cout_mul = 1'b0; Rem = 8'd0; neg = 1'b0;
		case (OPCODE)
//halt
			4'h0: halt = 1;
//load
			4'h1: alu_out = {RA,RB};
//add
			4'h2: begin
				alu_out = A + B;
/*
				{Cout,alu_out} = {1'b0,A} + {1'b0,B};
				Cinlastbit = (~A[7]&B[7] | A[7]&~B[7])&~alu_out[7] | (~A[7]&~B[7] | A[7]&B[7])&alu_out[7];
				OF = Cout^Cinlastbit;
*/
			end
//add im
			4'h3: begin
				alu_out = A + RB;
/*
				{Cout,alu_out} = {1'b0,A} + {1'b0,{4'd0,RB}};
				Cinlastbit = (~A[7]&B[7] | A[7]&~B[7])&~alu_out[7] | (~A[7]&~B[7] | A[7]&B[7])&alu_out[7];
				OF = Cout^Cinlastbit;
*/
			end
//sub
			4'h4: begin
				alu_out = A - B;
/*
				{Cout,alu_out} = {1'b0,A} + {1'b0,-B};
				Cinlastbit = (~A[7]&B[7] | A[7]&~B[7])&~alu_out[7] | (~A[7]&~B[7] | A[7]&B[7])&alu_out[7];
				OF = Cout^Cinlastbit;
*/
			end
//mul
			4'h5: begin
				alu_out = A*B;
/*
				{Cout_mul,alu_out} = abs(A) + abs(B);
				Cout = {Cout_mul,alu_out[7]} && 1'b1;
				OF = Cout;
				neg = isneg(A) ^ isneg(B);
				if (neg && !Cout) alu_out = ~alu_out;
*/
			end
//div
			4'h6: begin
				alu_out = A/B;
/*
				alu_out = abs(A) / abs(B);
				Rem  = abs(A) % abs(B);
				neg = isneg(A) ^ isneg(B);
				if (neg) alu_out = ~alu_out;
*/
			end
//inc
			4'h7: alu_out = B + 1;
//dec
			4'h8: alu_out = B - 1;
//nor
			4'h9: alu_out = ~(A | B);
//nand
			4'ha: alu_out = ~(A&B);
//xor
			4'hb: alu_out = A^B;
//comp
			4'hc: alu_out = ~B;
//jump
			4'hd: alu_out = alu_out;
//cmpl
			4'he: alu_out = alu_out;
//nop
//			4'hf: alu_out = alu_out;
			default: alu_out = 8'd0;
		endcase
	end

//isneg
	function logic isneg;
		input [7:0] v;
		isneg = v[7];
	endfunction
//abs
	function [7:0] abs;
		input [7:0] v;
		if (isneg(v)) abs = comp2(v);
		else abs = v;
	endfunction
//2's comp
	function [7:0] comp2;
		input [7:0] v;
		comp2 = ~v + 1'b1;
	endfunction
endmodule

////////////////////////////////////////////////////////////////////////

//W REG
module wreg (input clk, reset, input [7:0] data_in, output logic [7:0] data_out);
	
	always_ff @ (posedge clk or posedge reset) begin
		if (reset)
			data_out <= 8'd0;
		else 
			data_out <= data_in;
	end
endmodule

////////////////////////////////////////////////////////////////////////

// parameterized counter, frequency divider
module pmcntr #(parameter siz=25) (input clk, reset, input [siz-1:0] count_max, output logic [siz-1:0] count, output logic clkout);
	always_ff @ (posedge clk or posedge reset)
		if (reset) begin
			count <= {siz{1'b0}};
			clkout <= 1'b0;
		end
		else if (count<count_max)
			count <= count + {{(siz-1){1'b0}},1'b1};
		else begin
			count <= {siz{1'b0}};
			clkout <= ~clkout;
		end
endmodule

////////////////////////////////////////////////////////////////////////

//ASCII
module ASCII27Seg (input [7:0] AsciiCode, output logic [6:0] HexSeg);
	always_ff @ (*) begin
		HexSeg = 8'd0;
		$display ("AsciiCode %d", AsciiCode);
		case (AsciiCode)
//			0
			8'h0 : HexSeg[6] = 1;
//			1
			8'h1 : begin
				HexSeg[0] = 1; HexSeg[3] = 1; HexSeg[4] = 1; HexSeg[5] = 1; HexSeg[6] = 1;
			end
//			2
			8'h2 : begin
				HexSeg[2] = 1; HexSeg[5] = 1;
			end
//			3
			8'h3 : begin
				HexSeg[4] = 1; HexSeg[5] = 1;
			end
//			4
			8'h4 : begin
				HexSeg[0] = 1; HexSeg[3] = 1; HexSeg[4] = 1;
			end
//			5
			8'h5 : begin
				HexSeg[1] = 1; HexSeg[4] = 1;
			end
//			6
			8'h6 : HexSeg[1] = 1;
//			7
			8'h7 : begin
				HexSeg[3] = 1; HexSeg[4] = 1; HexSeg[5] = 1; HexSeg[6] = 1;
			end
//			8
			8'h8 : HexSeg = 8'd0;
//			9
			8'h9 : HexSeg[4] = 1;
//			A
			8'b00001010 : HexSeg[3] = 1;
//			a
			8'h61 : HexSeg[3] = 1;
//			B
			8'b00001011 : begin
				HexSeg[0] = 1; HexSeg[1] = 1;
			end
//			b
			8'h62 : begin
				HexSeg[0] = 1; HexSeg[1] = 1;
			end
//			C
			8'b00001100 : begin
				HexSeg[1] = 1; HexSeg[2] = 1; HexSeg[6] = 1;
			end
//			c
			8'h63 : begin
				HexSeg[1] = 1; HexSeg[2] = 1; HexSeg[6] = 1;
			end
//			D
			8'b00001101 : begin
				HexSeg[0] = 1; HexSeg[5] = 1;
			end
//			d
			8'h64 : begin
				HexSeg[0] = 1; HexSeg[5] = 1;
			end
//			E
			8'b00001110 : begin
				HexSeg[1] = 1; HexSeg[2] = 1;
			end
//			e
			8'h65 : begin
				HexSeg[1] = 1; HexSeg[2] = 1;
			end
//			F
			8'b00001111 : begin
				HexSeg[1] = 1; HexSeg[2] = 1; HexSeg[3] = 1;
			end
//			f
			8'h66 : begin
				HexSeg[1] = 1; HexSeg[2] = 1; HexSeg[3] = 1;
			end
//			H
			8'h48 : begin
				HexSeg[0] = 1; HexSeg[3] = 1;
			end
//			h
			8'h68 : begin
				HexSeg[0] = 1; HexSeg[3] = 1;
			end
//			I
			8'h49 : begin
				HexSeg[0] = 1; HexSeg[1] = 1; HexSeg[2] = 1; HexSeg[3] = 1; HexSeg[6] = 1;
			end
//			i
			8'h69 : begin
				HexSeg[0] = 1; HexSeg[1] = 1; HexSeg[2] = 1; HexSeg[3] = 1; HexSeg[6] = 1;
			end
//			N
			8'h4E : begin
				HexSeg[1] = 1; HexSeg[3] = 1; HexSeg[5] = 1; HexSeg[0] = 1;
			end
//			n
			8'h6E : begin
				HexSeg[1] = 1; HexSeg[3] = 1; HexSeg[5] = 1; HexSeg[0] = 1;
			end
//			_
			8'h42 : begin
				HexSeg[0] = 1; HexSeg[1] = 1; HexSeg[2] = 1; HexSeg[3] = 1; HexSeg[4] = 1; HexSeg[5] = 1; HexSeg[6] = 1;
			end
			default : HexSeg = 8'b11111111;
		endcase
	end
endmodule	
