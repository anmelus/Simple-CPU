module proj1(clk, rst, MemRW_IO, MemAddr_IO, MemD_IO);
	input clk;
    	input rst;
	output MemRW_IO;
	output [7:0]MemAddr_IO;
        output [15:0]MemD_IO;

	wire mload, done, zflag, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, MemRW;
	wire [1:0] opALU;
	wire [7:0] opcode, MemAddr;
	wire [15:0] MemQ, MemD;

	ctr ctr_ins(clk, rst, zflag, opcode, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, opALU, MemRW, done, mload);

	datapath data_ins(clk, rst, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, opALU, zflag, opcode, MemAddr, MemD, MemQ, done, mload);

	ram ram_ins(MemRW, MemD, MemQ, MemAddr);
	
	assign MemAddr_IO = MemAddr;
 	assign MemD_IO = MemD;
 	assign MemRW_IO = MemRW;
endmodule

module ram(we, d, q, addr);
	input we;
	input [15:0] d;
	output reg [15:0] q;
	input [7:0] addr;
	reg [15:0] mem256x16 [0:255]; 
	
	always @ (*)
	begin
	if (we == 1)
		begin
		mem256x16[addr] = d;
		q = 0;
		end
	else 
		q = mem256x16[addr];
	end
endmodule

//Declare the ports of Half adder module
module half_adder(A, B, S, C);
    //what are the input ports.
    input A;
    input B;
    //What are the output ports.
    output S;
    output C;
    //reg S,C; NOT NEEDED will cause error
     //Implement the Sum and Carry equations using Structural verilog
     xor x1(S,A,B); //XOR operation (S is output not a input)
     and a1(C,A,B); //AND operation 
 endmodule

//Declare the ports for the full adder module
module full_adder( A, B, Cin, S, Cout); // (Port names)

    //what are the input ports.
    input A;
    input B;
    input Cin;

    //What are the output ports.
    output S;
    output Cout;
    //reg  A,B,Cin; //(Doesn't need to be reg, it will cause an error
    wire Sa1, Ca1, Ca2;    
     //Two instances of half adders used to make a full adder
     half_adder a1(.A(A),.B(B),.S(Sa1),.C(Ca1)); //S to Sa1 to match diagram
     half_adder a2(.A(Sa1),.B(Cin),.S(S),.C(Ca2));
     or  o1(Cout,Ca1,Ca2);
 endmodule

module my16bitadder(output [15:0] O, output c, input [15:0] A, B, input S);
//wires for connecting output to input
	wire Cout0in1, Cout1in2, Cout2in3, Cout3in4, Cout4in5, Cout5in6, Cout6in7, Cout7in8,
 	Cout8in9, Cout9in10, Cout10in11, Cout11in12, Cout12in13, Cout13in14, Cout14in15;

	full_adder Add0 ( A[0], B[0], S, O[0], Cout0in1); 
	full_adder Add1 ( A[1], B[1], Cout0in1, O[1], Cout1in2); 
	full_adder Add2 ( A[2], B[2], Cout1in2, O[2], Cout2in3); 
	full_adder Add3 ( A[3], B[3], Cout2in3, O[3], Cout3in4); 
	full_adder Add4 ( A[4], B[4], Cout3in4, O[4], Cout4in5); 
	full_adder Add5 ( A[5], B[5], Cout4in5, O[5], Cout5in6); 
	full_adder Add6 ( A[6], B[6], Cout5in6, O[6], Cout6in7); 
	full_adder Add7 ( A[7], B[7], Cout6in7, O[7], Cout7in8); 
	full_adder Add8 ( A[8], B[8], Cout7in8, O[8], Cout8in9); 
	full_adder Add9 ( A[9], B[9], Cout8in9, O[9], Cout9in10); 
	full_adder Add10 ( A[10], B[10], Cout9in10, O[10], Cout10in11); 
	full_adder Add11 ( A[11], B[11], Cout10in11, O[11], Cout11in12);
	full_adder Add12 ( A[12], B[12], Cout11in12, O[12], Cout12in13); 
	full_adder Add13 ( A[13], B[13], Cout12in13, O[13], Cout13in14); 
	full_adder Add14 ( A[14], B[14], Cout13in14, O[14], Cout14in15);  
	full_adder Add15 ( A[15], B[15], Cout14in15, O[15], c);
endmodule

module my8bitmultiplier(output reg[15:0] O, output reg Done, output Cout, input [7:0] A, B, input Load,Clk,Reset,Cin);
reg[1:0] state;
reg[15:0] A_reg, B_reg, A_temp, O_reg, B_temp;
wire[15:0] O_temp;

my16bitadder dut1(O_temp, Cout, A_temp, B_temp, Cin);

always@(posedge Clk)
begin
if(Reset)
state <= 0;
else
case(state)
	0: if(Load)begin
		A_reg <= A; 
		B_reg <= B; 
		O_reg <= A; 
		state <= 1;
                Done <= 0;
                O <= 0;
		end
	1: begin
		A_temp <= A_reg; 
		B_temp <= O_reg; 
		B_reg <= B_reg - 1; 
		state <= 2;
		end
	2: begin
		O_reg <= O_temp;
		if(B_reg>1) state <= 1;
		else begin
			state <= 3; 
			Done <= 1;
			O <= O_temp;
			end
		end
	3: begin
		Done <= 0; 
		state <= 0;
		end

endcase
end
endmodule


module alu(A, B, opALU, Rout, done, mload, clk, rst);
input [15:0] A;
input [15:0] B;
input [1:0] opALU;
output reg [15:0] Rout;
input wire clk, rst, mload;
reg mclk, mrst, multiplyload;

reg[7:0] areg, breg;
input wire done;
wire cout, acout;
wire [15:0] out, aout;

my8bitmultiplier multiplydut(out, done, cout, areg, breg, multiplyload, mclk, rst, 1'b0);
my16bitadder adderdut(aout, acout, A, B, 1'b0);

always @(*)
begin
	case(opALU)
	0: Rout = A | B;
	1: Rout = aout;
	2: begin
		multiplyload = mload;
		areg = A[7:0];
		breg = B[7:0];
		Rout = out;
	end
	3: Rout = ~A;
	endcase
	mclk = clk;
	mrst = rst;
end
endmodule
	
module ctr (clk, rst, zflag, opcode, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, opALU, MemRW, done, mload);
        input clk;
        input rst;
        input zflag;
        input [7:0]opcode;
	input done;
        output reg muxPC;
        output reg muxMAR;
        output reg muxACC;
        output reg loadMAR;
        output reg loadPC;
        output reg loadACC;
        output reg loadMDR;
        output reg loadIR;
        output reg [1:0] opALU;
        output reg MemRW;
	output reg mload;
	
	parameter Fetch_1 = 5'b00000; 	// 0
	parameter Fetch_2 = 5'b00001;	// 1
	parameter Fetch_3 = 5'b00010;	// 2
	parameter Decode = 5'b00011;	// 3
	parameter ExecAdd_1 = 5'b00100;	// 4
	parameter ExecAdd_2 = 5'b00101;	// 5
	parameter ExecOr_1 = 5'b00110;	// 6
	parameter ExecOr_2 = 5'b00111;	// 7
	parameter ExecLoad_1 = 5'b01000;	// 8
	parameter ExecLoad_2 = 5'b01001;	// 9
	parameter ExecStore = 5'b01010;	// 10
	parameter ExecJump = 5'b01011;	// 11
	parameter ExecMult_1 = 5'b01100;	// 12
	parameter ExecMult_2 = 5'b01101;	// 13
	parameter ExecMult_3 = 5'b01110;	// 14
	parameter ExecMult_4 = 5'b01111;	// 15
	parameter ExecMult_5 = 5'b10000;	// 16
	parameter ExecNegate = 5'b10001;	// 17

	parameter op_add=8'b001;
	parameter op_or= 8'b010;
	parameter op_jump=8'b011;
	parameter op_jumpz=8'b100;
	parameter op_load=8'b101;
	parameter op_store=8'b110;
	parameter op_mull=8'b1001;
	parameter op_neg=8'b1010;

	reg[4:0] state;
	reg[4:0] state_next;

	always@(posedge clk)
	begin
	if(rst)
		state = Fetch_1;
	else
		state = state_next;
	end
	
	always @ (*)
	begin
	case(state)
		Fetch_1: state_next = Fetch_2;
		Fetch_2: state_next = Fetch_3;
		Fetch_3: state_next = Decode;
		Decode:
			case(opcode)
				op_add: state_next = ExecAdd_1;
				op_or: state_next = ExecOr_1;
				op_jump: state_next = ExecJump;
				op_jumpz:
					if(zflag == 1)
						state_next = ExecJump;
					else
						state_next = Fetch_1;
				op_load: state_next = ExecLoad_1;
				op_store: state_next = ExecStore;
				op_mull: state_next = ExecMult_1;
				op_neg: state_next = ExecNegate;
	
			endcase
		ExecAdd_1: state_next = ExecAdd_2;
		ExecAdd_2: state_next = Fetch_1;
		ExecOr_1: state_next = ExecOr_2;
		ExecOr_2: state_next = Fetch_1;
		ExecLoad_1: state_next = ExecLoad_2;  
      		ExecLoad_2: state_next = Fetch_1;  
		ExecStore: state_next = Fetch_1;  
		ExecJump: state_next = Fetch_1;  
		ExecMult_1: state_next = ExecMult_2;
		ExecMult_2: state_next = ExecMult_3;
		ExecMult_3: state_next = ExecMult_4;
		ExecMult_4: state_next = ExecMult_5;
		ExecMult_5:  
		if(done)
			state_next = Fetch_1;
		else
			state_next = ExecMult_5;
		ExecNegate: state_next = Fetch_1;
		default: state_next = state;
    	endcase
	end

	always @ (state)
  	begin
	case (state)
      		Fetch_1:	 
	  	begin
			muxPC = 1'b0; //
    			muxMAR = 1'b0; //
    			muxACC = 1'b0; 
    			loadMAR = 1'b1;  //
    			loadPC = 1'b1; //
    			loadACC = 1'b0; 
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b0;

	   	end
		  
      		Fetch_2:	
	  	begin
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b0; 
    			loadMDR = 1'b1; //
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b0; //
		end
		  
      		Fetch_3:	 
	  	begin
    			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b0; 
    			loadMDR = 1'b0; 
    			loadIR = 1'b1; //
    			opALU = 2'b0; 
    			MemRW = 1'b0;  
		end  

		Decode:	 
	  	begin
    			muxPC = 1'b0; 
    			muxMAR = 1'b1; //
    			muxACC = 1'b0; 
    			loadMAR = 1'b1; //
    			loadPC = 1'b0; 
    			loadACC = 1'b0; 
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b0;  
      		end
		  
      		ExecAdd_1:	
      		begin	  
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b0; 
    			loadMDR = 1'b1; //
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b0; //
		end
		  
      		ExecAdd_2:	 
	  	begin
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; //
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b1; //
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b1; //
    			MemRW = 1'b0;
		end

      		ExecOr_1:	 
	  	begin
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b0; 
    			loadMDR = 1'b1; //
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b0; //
		  
		end

		ExecOr_2:
        	begin		 
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; //
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b1; //
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b0; //
    			MemRW = 1'b0;  
         	end
 
      		ExecLoad_1:	
         	begin	  
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b0; 
    			loadMDR = 1'b1; //
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b0;  //
         	end		  
		  
      		ExecLoad_2:
        	begin	  
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b1; //
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b1; //
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b0;
        	end		  
		  
      		ExecStore:
        	begin	  
 			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b0; 
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b1; //
		end

      		ExecJump:
        	begin	  
			muxPC = 1'b1; //
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b1; //
    			loadACC = 1'b0; 
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b0; 
		end

		ExecMult_1:	
      		begin	  
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b0; 
    			loadMDR = 1'b1; //
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b0; //
		end
		  
      		ExecMult_2:	 
	  	begin
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; //
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b0; //
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b10; //
    			MemRW = 1'b0;
		end
	
		ExecMult_3:	 
	  	begin
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b0; 
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b10; //
    			MemRW = 1'b0;
			mload = 1'b1; //
		end
		
		ExecMult_4:	 
	  	begin
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b0; 
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b10; //
    			MemRW = 1'b0;
			mload = 1'b0;
		end

		ExecMult_5:	 
	  	begin
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0;  //
    			loadMAR = 1'b0; 
    			loadPC = 1'b0; 
    			loadACC = 1'b1;  //
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b10; //
    			MemRW = 2'b0;
		end

		ExecNegate:
        	begin	  
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; //
    			loadMAR = 1'b0; 
    			loadPC = 1'b0;
    			loadACC = 1'b1; //
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b11; //
    			MemRW = 1'b0; 
		end
		default:
		begin
			muxPC = 1'b0; 
    			muxMAR = 1'b0; 
    			muxACC = 1'b0; 
    			loadMAR = 1'b0; 
    			loadPC = 1'b0;
    			loadACC = 1'b0; 
    			loadMDR = 1'b0; 
    			loadIR = 1'b0; 
    			opALU = 2'b0; 
    			MemRW = 1'b0; 
		end
   	endcase  
   	end		
endmodule

module registers(clk, rst, PC_reg, PC_next, IR_reg, IR_next, ACC_reg, ACC_next, MDR_reg, MDR_next, MAR_reg, MAR_next, Zflag_reg, zflag_next);
	input wire clk;
	input wire rst;
	output reg  [7:0]PC_reg; 
	input wire  [7:0]PC_next;
 
	output reg  [15:0]IR_reg;  
	input wire  [15:0]IR_next;  

	output reg  [15:0]ACC_reg;  
	input wire  [15:0]ACC_next;  

	output reg  [15:0]MDR_reg;  
	input wire  [15:0]MDR_next;  

	output reg  [7:0]MAR_reg;  
	input wire  [7:0]MAR_next;  

	output reg Zflag_reg;
	input wire zflag_next;

	always @(posedge clk)
	begin
	if (rst) begin
		PC_reg <= 0;
		IR_reg <= 0;
		ACC_reg <= 0;
		MDR_reg <= 0;
		MAR_reg <= 0;
	end
	else 
	begin 
		PC_reg <= PC_next;
		IR_reg <= IR_next;
   		ACC_reg <= ACC_next;
   		MDR_reg <= MDR_next;
   		MAR_reg <= MAR_next;
   		Zflag_reg <= zflag_next;
  	end
	end
endmodule

module datapath(clk, rst, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, opALU, zflag, opcode, MemAddr, MemD, MemQ, done, mload);
	input  clk;
	input  rst;
	input  muxPC;
	input  muxMAR;
	input  muxACC;
	input  loadMAR;
	input  loadPC;
	input  loadACC;
	input  loadMDR;
	input  loadIR;
	input[1:0] opALU; 
	output reg zflag;
	output reg[7:0]opcode;
	output reg[7:0]MemAddr;
	output reg[15:0]MemD;
	input[15:0]MemQ;

	reg  [7:0]PC_next;
	reg  [15:0]IR_next; // wire?
	reg  [15:0]ACC_next;
	reg  [15:0]MDR_next; // wire?
	reg  [7:0]MAR_next;
	reg zflag_next;

	wire  [7:0]PC_reg;
	wire  [15:0]IR_reg;
	wire  [15:0]ACC_reg;
	wire  [15:0]MDR_reg;
	wire  [7:0]MAR_reg;
	wire zflag_reg;
	wire  [15:0]ALU_out;

	output done;
	input mload; 

	
 	alu out(ACC_reg, MDR_reg, opALU, ALU_out, done, mload, clk, rst);
	registers out2(clk, rst, PC_reg, PC_next, IR_reg, IR_next, ACC_reg, ACC_next, MDR_reg, MDR_next, MAR_reg, MAR_next, Zflag_reg, zflag_next);

	always @(*)
	begin
		if(loadPC)begin
			case(muxPC)
				0: PC_next = PC_reg + 1;
				1: PC_next = IR_reg[15:8]; 
		endcase
		end
		else 
			PC_next = PC_reg;

		if(loadIR)
			IR_next = MDR_reg;
		else
			IR_next = IR_reg;

		if(loadACC)begin
			case(muxACC)
				0: ACC_next = ALU_out;
				1: ACC_next = MDR_reg;
			endcase
		end
		else ACC_next = ACC_reg;

		if(loadMDR)
			MDR_next = MemQ;
		else
			MDR_next = MDR_reg;

			if(loadMAR)begin
				case(muxMAR)
					0: MAR_next = PC_reg;
					1: MAR_next = IR_reg[15:8]; 
			endcase
			end
			else MAR_next = MAR_reg;

		if(ACC_reg == 0)
			zflag_next = 1;
		else
			zflag_next = 0;

	
	zflag = zflag_reg;
	opcode = IR_reg[7:0];
	MemAddr = MAR_reg;
	MemD = ACC_reg;
end
endmodule

module proj1_tb;
    reg clk;
    reg rst;
    wire MemRW_IO;
    wire [7:0]MemAddr_IO;
    wire [15:0]MemD_IO;

proj1 dut(clk, rst, MemRW_IO, MemAddr_IO, MemD_IO);

always 
      #5  clk =  !clk; 

initial begin
	clk=1'b0;
	rst=1'b1;
	$readmemh("memory.list", proj1_tb.dut.ram_ins.mem256x16);
	#20 rst=1'b0;
	#1000 
	$display("Final value\n");
	$display("0x000e %h\n",proj1_tb.dut.ram_ins.mem256x16[16'h000e]);
	$finish;
end

endmodule


		


	
				


	


