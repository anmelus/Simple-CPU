//Declare the ports of Half adder module
module half_adder(A, B, S, C);

    input A;
    input B;
 
    output S;
    output C;

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

module myexp(output reg[15:0] O, output reg Done, output Cout, input [15:0] A, B, input Load,Clk,Reset,Cin);
reg[1:0] state;			
reg[15:0] A_reg, B_reg, A_temp, O_reg, B_temp;
wire[15:0] O_temp;
reg[15:0] exponent;

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
		exponent <= A;
                Done <= 0;
                O <= 0;
		end
	1: begin
		A_temp <= A_reg; 
		B_temp <= O_reg; 
		exponent <= exponent - 1; 
		state <= 2;
		end
	2: begin
		O_reg <= O_temp;
		if(exponent > 1) state <= 1;
		else begin
			state <= 3; 
			exponent <= A;
			A_reg <= O_temp;
			B_reg <= B_reg - 1;
			end
		end
	3: begin
		if(B_reg > 1) state <= 1;
		else
		begin
			state <= 4;
			Done <= 1;
			O <= O_temp;

		end
	   end
	4: begin
		Done <= 0; 
		state <= 0;
		end

endcase
end
endmodule

module myexp_tb;
reg clk, reset, load, cin;
reg [15:0] a, b;
wire done, cout;
wire[15:0] out;

myexp dut(out, done, cout, a, b, load, clk, reset, cin);

always #5 clk = ~clk;
initial
begin
clk = 0;
cin = 0;
reset = 1;
load = 1;
a = 8'd9; 
b = 8'd3;
#10 reset = 0;
#10 load = 0;
#800 $display (" A = %d, B = %d, O = %d", a, b, out);

#10 $finish;
end
endmodule

module fullsubtractor(input [15:0] x, input [15:0] y, output [15:0] O);
 
assign O =   y -  x;
 
endmodule

module mymodfunc(output reg[15:0] O, output reg Done, input [15:0] A, B, input Load,Clk,Reset,Cin);
reg[1:0] state;
reg[15:0] A_reg, B_reg, A_temp, O_reg, B_temp;
wire[15:0] O_temp;

fullsubtractor dut1(A_temp, B_temp, O_temp);

always@(posedge Clk)
begin
if(Reset)
state <= 0;
else
case(state)
	0: if(Load)begin
		A_reg <= B; 
		B_reg <= B; 
		O_reg <= A; 
		state <= 1;
                Done <= 0;
                O <= 0;
		end
	1: begin
		A_temp <= A_reg; 
		B_temp <= O_reg;  
		state <= 2;
		end
	2: begin
		O_reg <= O_temp;
		if(O_reg >= B_reg) state <= 1;
		else begin
			state <= 3; 
			Done <= 1;
			O <= O_reg;
			end
		end
	3: begin
		Done <= 0; 
		state <= 0;
		end

endcase
end
endmodule

module mymodfunc_tb;
reg clk, reset, load, cin;
reg [15:0] a, b;
wire done;
wire[15:0] out;

mymodfunc dut(out, done, a, b, load, clk, reset, cin);

always #5 clk = ~clk;
initial
begin
clk = 0;
cin = 0;
reset = 1;
load = 1;
a = 16'd1456; 
b = 16'd9;
#10 reset = 0;
#10 load = 0;
#9000 $display (" A = %d, B = %d, O = %d", a, b, out);

#10 $finish;
end
endmodule

module RSA(private_key, public_key, message_val, clk, Rst, Start, Cal_val, Cal_done);

input clk, Rst, Start;
input[15:0] private_key, public_key, message_val;

output reg[15:0] Cal_val;
output reg Cal_done;

reg Load_exp, Load_mod;
wire Done_exp, Done_mod, Cout;
reg Cin = 0;

reg [15:0] private_temp, private_reg, public_temp, public_reg, message_reg, message_temp;
wire[15:0] O_temp, last_val;

reg[2:0] state;
parameter Capture_state = 3'b000, Exponent_state1 = 3'b001, Exponent_state2 = 3'b010, Exponent_state3 = 3'b011;
parameter Mod_state1 = 3'b100, Mod_state2 = 3'b101, Mod_state3 = 3'b110, Cal_done_state = 3'b111;

myexp exp1(.O(O_temp), .Done(Done_exp), .Cout(Cout), .A(message_temp), .B(private_temp), .Load(Load_exp), .Clk(clk), .Reset(Rst), .Cin(Cin));
mymodfunc mod1(.O(last_val), .Done(Done_mod), .A(O_temp), .B(public_temp), .Load(Load_mod), .Clk(clk), .Reset(Rst), .Cin(Cin));

always@(posedge clk) begin
	if (Rst) 
		state <= 0; 
	else begin
		case(state) 
			Capture_state: if(Start) begin
						Load_exp <= 0;
						Load_mod <= 0;
						Cal_val <= 0;
						Cal_done <= 0;
						private_reg <= private_key;
						public_reg <= public_key;
						message_reg <= message_val;
						state <= Exponent_state1; end
			Exponent_state1: begin
						Load_exp <= 1;
						private_temp <= private_reg;
						message_temp <= message_reg;
						state <= Exponent_state2; end
			Exponent_state2: begin
						Load_exp <= 0;
						state <= Exponent_state3; end
			Exponent_state3: begin 
					if (Done_exp) state <= Mod_state1;			
				    	else state <= Exponent_state2; end

			Mod_state1: begin
					Load_mod <= 1;
					public_temp <= public_reg;
					state <= Mod_state2; end
			Mod_state2: begin
					Load_mod <= 0;
					state <= Mod_state3; end
			Mod_state3: begin 
					if (Done_mod) state <= Cal_done_state;
				    	else state <= Mod_state2; end
			Cal_done_state: begin
						Cal_done <= 1;
						Cal_val <= last_val;
						state <= Capture_state; end
		endcase
	end
end
endmodule

module proj2(private_key, public_key, message_val, clk, Rst, Start, Cal_val, Cal_done);
input clk, Rst, Start;
input[15:0] private_key, public_key, message_val;

output[15:0] Cal_val;
output Cal_done;

RSA main1(private_key, public_key, message_val, clk, Rst, Start, Cal_val, Cal_done);

endmodule


module proj2_tb;
reg clk, Rst, Start;
reg[15:0] private_key, public_key, message_val;

wire[15:0] Cal_val;
wire Cal_done;

proj2 dut(private_key, public_key, message_val, clk, Rst, Start, Cal_val, Cal_done);

always #5 clk = ~clk;

always@(*)
	case(proj2_tb.dut.main1.state)
		proj2_tb.dut.main1.Capture_state: $display("State = Capture State");
		proj2_tb.dut.main1.Exponent_state1: $display("State = Exponent State 1");
		proj2_tb.dut.main1.Exponent_state2: $display("State = Exponent State 2");
		proj2_tb.dut.main1.Exponent_state3: $display("State = Exponent State 3");
		proj2_tb.dut.main1.Mod_state1: $display("State = Mod State 1");
		proj2_tb.dut.main1.Mod_state2: $display("State = Mod State 2");
		proj2_tb.dut.main1.Mod_state3: $display("State = Mod State 3");
		proj2_tb.dut.main1.Cal_done_state: $display("State = Cal Done State");
	endcase

initial
begin
clk = 0;
Rst = 1;
Start = 1;
message_val = 16'd9;
private_key = 16'd3;
public_key = 16'd33;
#10 Rst = 0;
#10 Start = 0;

#2000 $display (" Message Value = %d, Private Key = %d, Public Key = %d, New Encrypted Value = %d, Cal_Done = %d", message_val, private_key, public_key, dut.Cal_val, dut.Cal_done);

#10 $finish;
end
endmodule

