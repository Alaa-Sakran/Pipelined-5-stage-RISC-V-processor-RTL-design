module RV32I_Pipelined(input clk,reset);

logic [31:0] Instruction;
logic [3:0] ALUOP;
logic [2:0] EXTDEC_CODE;
logic WEM,MUX3INPUT,M1,J,WER,zero,LessThan,LUI,stall,enable,M1_Q3,flush_f5,flush_f2,enable_f2,M7,flush_f3,M8;
logic [1:0] M2,M3,M2_Q4,M5,M6,M2_Q3,lui_select1,lui_select2;
logic [4:0] A1_Q3,A2_Q3,A3_Q4,A3_Q5,A3_Q3;

datapath 		dp(lui_select1,lui_select2,M7,flush_f2,enable_f2,enable,M5,M6,LUI,clk,reset,Instruction,LessThan,zero,M2, WEM,MUX3INPUT,M1,WER,EXTDEC_CODE,ALUOP,A1_Q3,A2_Q3,A3_Q4,A3_Q5,A3_Q3,M1_Q3); 

controller 		control_unit(flush_f3,M8,enable,clk,reset,Instruction,LUI,WEM,MUX3INPUT,M1,WER,M2,ALUOP,EXTDEC_CODE,zero,LessThan,M2_Q4,M2_Q3);
hazard_unit_forwarding 		 H1( A1_Q3,A2_Q3,A3_Q4,A3_Q5,M1_Q3,M2_Q4,M5,M6,lui_select1,lui_select2);//forwards from stage 4 ,5 to stage 3
hazard_unit_stall     		 H2(clk,A3_Q3,Instruction[19:15],Instruction[24:20],M1,M2_Q3,stall,enable);
branch_flush                     Flush(clk,MUX3INPUT,flush_f2,enable_f2,M7,flush_f3,M8); //flushes 3 instructions after branches and jumps
//flushes current instructions and disables the F2D for one cycle so that unwanted instruction doesn't get decoded

endmodule 

module controller(input logic  flush_f3,M8,enable,input logic clk,reset,input logic [31:0] Instruction,output logic LUI_Q5,WEM_Q4,MUX3INPUT,M1,WER_Q5, output logic [1:0] M2_Q5, output logic [3:0] ALUOP_Q3, output logic [2:0] EXTDEC_CODE,
input logic zero,LessThan, output logic [1:0] M2_Q4,M2_Q3);

logic  WEM_Q3, MUX3INPUT_Q3, MUX3INPUT_Q4 ,J_Q3, J_Q4, WER_Q3, WER_Q4, WEM, J, WER, MUX8OUTPUT;
logic [1:0] M3_Q2 ,M2, M3_Q5 ,M3_Q4,M3,M3_Q3;
logic [3:0] ALUOP;

logic [2:0] ALUOPDEC;
MUX2TO1 #(1)  MUX8(reset,flush_f3,M8,MUX8OUTPUT);

main_dec U1(LUI,Instruction[6:0],M2,WER,WEM,M1,J,EXTDEC_CODE,ALUOPDEC);
ALUDEC aludec(ALUOPDEC,Instruction[14:12], Instruction[31:25],ALUOP,M3);
assign MUX3INPUT=(zero&(M3_Q3==2'b01))|(LessThan&(M3_Q3==2'b10))|(J_Q3);
flop_enabled #(12) F3(clk,MUX8OUTPUT,enable, {WEM,J,WER,M2,M3,ALUOP,LUI}, {WEM_Q3,J_Q3,WER_Q3,M2_Q3,M3_Q3,ALUOP_Q3,LUI_Q3});
flop_enabled #(5) F4(clk,reset,enable, {WEM_Q3,WER_Q3,M2_Q3,LUI_Q3}, {WEM_Q4,WER_Q4,M2_Q4,LUI_Q4} );
flop #(4) F5(clk,reset, {WER_Q4,M2_Q4,LUI_Q4}, {WER_Q5,M2_Q5,LUI_Q5});
endmodule

module datapath(input logic [1:0] lui_select1,lui_select2,input logic M7,flush_f2,enable_f2,enable,input logic [1:0] M5,M6,input logic LUI,clk,reset, output logic [31:0] Instruction_Q2, output logic LessThan,zero,
input logic [1:0] M2,
input logic WEM,MUX3INPUT,M1,WER, input logic [2:0] EXTDEC_CODE, input logic [3:0] ALUOP, output logic [4:0] A1_Q3,A2_Q3,A3_Q4,A3_Q5,A3_Q3, output logic M1_Q3);

logic [31:0] Instruction,PCNEXT_Q2,RDA2_Q3,RDA1_Q3,MUX1OUTPUT_Q3,PCNEXT_Q3,RDA2_Q4, ALUOUT_Q4,ALUTOUT_Q4_2, PCNEXT_Q4, ALUOUT_Q5,RDB1_Q5,PCNEXT_Q5;
logic [31:0] RDA1,RDA2,MUX1OUTPUT,EXTOUT,BranchAddress,RDB1,MUX3OUTPUT;
logic [31:0] PCNEXT,PC,ALUOUT,MUX2OUTPUT,MUX4OUTPUT,EXTOUT_Q2,PC_Q2,BranchAddress_Q2,EXTOUT_Q3,EXTOUT_Q4,EXTOUT_Q5,MUX5OUTPUT,MUX6OUTPUT,
HZMUX1OUTPUT,HZMUX2OUTPUT;
logic M4_Q3,M4_Q4,M4_Q5,MUX7OUTPUT;

instruction_memory IM1(PC,Instruction);
REGFILE regfile(LUI,Instruction_Q2[19:15], Instruction_Q2[24:20], A3_Q5, WER,clk,reset,MUX2OUTPUT,RDA1,RDA2);
Extendor E1(EXTDEC_CODE,Instruction_Q2[31:7],EXTOUT);
SimpleAdder A1(PC,32'd4,PCNEXT);

ALU alu(HZMUX1OUTPUT,HZMUX2OUTPUT,ALUOP,ALUOUT,zero,LessThan);
SimpleAdder A2(EXTOUT,PC_Q2,BranchAddress);
data_memory DM(ALUTOUT_Q4_2 ,RDA2_Q4 ,WEM,clk,RDB1);
MUX2TO1 #(32) MUX1(RDA2,EXTOUT,M1,MUX1OUTPUT);
MUX4TO1 #(32) MUX2(ALUOUT_Q5, RDB1_Q5 ,PCNEXT_Q5 ,EXTOUT_Q5,M2,MUX2OUTPUT);
MUX2TO1 #(32) MUX3(PCNEXT,BranchAddress_Q2,MUX3INPUT,MUX3OUTPUT); //BranchAddres is used here for jumps too
MUX4TO1 #(32) MUX5(RDA1_Q3,MUX2OUTPUT,ALUOUT_Q4,PCNEXT_Q3,M5,MUX5OUTPUT);
MUX4TO1 #(32) MUX6(MUX1OUTPUT_Q3,MUX2OUTPUT,ALUOUT_Q4,PCNEXT_Q3,M6,MUX6OUTPUT);
MUX2TO1 #(1)  MUX7(enable,enable_f2,M7,MUX7OUTPUT);//enable_f2 is used to stall for one cycle after flushing while enable is for normal stalling

hazard_mux #(32) HZM1(MUX5OUTPUT,EXTOUT_Q4,MUX2OUTPUT,lui_select1,HZMUX1OUTPUT);
hazard_mux #(32) HZM2(MUX6OUTPUT,EXTOUT_Q4,MUX2OUTPUT,lui_select2,HZMUX2OUTPUT);

flop_enabled #(32) F1F(clk,reset,enable,MUX3OUTPUT,PC); 
flop_enabled #(96) F2D(clk,flush_f2,MUX7OUTPUT, {Instruction,PCNEXT,PC},{Instruction_Q2,PCNEXT_Q2,PC_Q2});
flop_enabled #(176) F3E(clk,reset,enable,{RDA2,RDA1,MUX1OUTPUT,PCNEXT_Q2,Instruction_Q2[11:7],EXTOUT,Instruction_Q2[19:15],Instruction_Q2[24:20],M1},{RDA2_Q3,RDA1_Q3,MUX1OUTPUT_Q3,PCNEXT_Q3,A3_Q3,EXTOUT_Q3,A1_Q3,A2_Q3,M1_Q3});
flop_enabled #(165) F4M(clk,reset,enable ,{RDA2_Q3,ALUOUT,ALUOUT,PCNEXT_Q3,A3_Q3,EXTOUT_Q3}, {RDA2_Q4, ALUOUT_Q4,ALUTOUT_Q4_2, PCNEXT_Q4,A3_Q4,EXTOUT_Q4} );
flop #(133) F5W(clk, reset, {ALUOUT_Q4, RDB1, PCNEXT_Q4,A3_Q4,EXTOUT_Q4}, {ALUOUT_Q5,RDB1_Q5,PCNEXT_Q5,A3_Q5,EXTOUT_Q5});
flop #(32)    FBRANCH(clk,reset,BranchAddress,BranchAddress_Q2); //special flop for branch address

endmodule
module hazard_mux #(parameter WIDTH)(input logic [WIDTH-1:0] MUXOUT,EXTOUT_Q4,MUX2OUTPUT, input logic [1:0]  select, output logic [WIDTH-1:0] out);
always_comb
begin
case(select) 
2'b00:out=MUXOUT;
2'b01:begin out[WIDTH-1:WIDTH-20]=MUX2OUTPUT[WIDTH-1:WIDTH-20]; out[WIDTH-21:0]=MUXOUT[WIDTH-21:0]; end
2'b10:begin out[WIDTH-1:WIDTH-20]=EXTOUT_Q4[WIDTH-1:WIDTH-20]; out[WIDTH-21:0]=MUXOUT[WIDTH-21:0]; end
default:out=out;
endcase
end

endmodule
module branch_flush(input clk,MUX3INPUT, output logic flush_f2,enable_f2,M7,flush_f3,M8);
logic flush_done;
always_comb
begin
if(MUX3INPUT & (flush_done)) begin flush_f2<=0; enable_f2<=0; M7<=1; end //when flush is done flush_done is low therefore needn't flush again. only for 1 cycle
else begin flush_f2<=1; enable_f2<=1; M7<=0; end //M7 is used for enabling and reseting F2 to seperate between flush and stall
end
always_ff@(posedge clk)
begin
if(MUX3INPUT)begin flush_done<=0; flush_f3<=0; M8<=1;  end
else begin  flush_done<=1; flush_f3<=1; M8<=0; end //m8 is used for F3 so that MUX3INPUT isn't 1 after jumping
end
endmodule

module hazard_unit_stall(input logic clk, input logic [4:0] A3_Q3,A1_Q2,A2_Q2,input logic M1 ,input logic [1:0] M2_Q3, output logic stall,enable);
always_ff@(posedge clk)
begin
if (stall)begin stall<=0; end else
if((((A3_Q3 == A2_Q2)!=0) & (M1 == 0) & (M2_Q3 == 2'b01)) |
               (((A3_Q3 == A1_Q2)!=0) & (M2_Q3 == 2'b01))) begin
    stall <= 1;   end

end

always_comb
begin
if(stall)
	begin
	enable<=0;
	end
else 
enable<=1;
end
endmodule


module hazard_unit_forwarding(input logic [4:0]A1_Q3,A2_Q3,A3_Q4,A3_Q5, input logic  M1_Q3,input logic [1:0] M2_Q4, output logic [1:0]M5,M6, output logic [1:0] lui_select1,lui_select2);

always_comb 
begin
  M5   = 2'b00;
  M6   = 2'b00;
  lui_select1=0;
  lui_select2=0;
 

  
 if (((A3_Q4 == A1_Q3)!=0) & (M2_Q4 == 2'b00)  ) //ALUOUT
    M5 = 2'b10; 
else if (((A3_Q4 == A1_Q3)!=0) & (M2_Q4 == 2'b11)  ) //ALUOUT
    lui_select1=2'b10; //lui from extout_q4
 else if ((A3_Q5 == A1_Q3)& (M2_Q4 == 2'b10))   //PCNEXT
    M5 = 2'b11; 
 else if ((A3_Q5 == A1_Q3)& (M2_Q4 == 2'b11))   //lui
   lui_select1=2'b01;//lui from mux2
 //first if
 else if ((A3_Q5 == A1_Q3))  M5 <= 2'b01; //MUX2OUTPUT 
 

if (((A3_Q4 == A2_Q3)!=0) & (M1_Q3 == 0) & (M2_Q4 == 2'b00)) //aluout 
    M6 = 2'b10;
  else 
if (((A3_Q4 == A2_Q3)!=0) & (M1_Q3 == 0) & (M2_Q4 == 2'b00)) 
    lui_select2=2'b01;
  else 
if (((A3_Q5 == A2_Q3)!=0) & (M1_Q3 == 0) &(M2_Q4 == 2'b10) ) M6 = 2'b11; //pcnext
  else 
if (((A3_Q5 == A2_Q3)!=0) & (M1_Q3 == 0) &(M2_Q4 == 2'b11) ) lui_select2=2'b01; //lui2 from mux2
else 
if (((A3_Q5 == A2_Q3)!=0) & (M1_Q3 == 0))  //mux2output 
    M6 = 2'b01; 
 //second if



 
end


endmodule

module branch_prediction();
endmodule

module ALU( input logic [31:0] operand1,operand2, input logic [3:0] ALUOP, output logic [31:0] result, output logic zero,LessThan);
always_comb //operand1 is RDA1 while operand2 is the output of MUX2, result is also connected to both data memory and MUX3
begin
case(ALUOP)
4'b0000: result=operand1+operand2;
4'b0001: result=operand1-operand2;
4'b0010: result=operand1&operand2;
4'b0011: result=operand1|operand2;
4'b0100: result=operand1^operand2;
4'b0101: result=operand1>>operand2;
4'b0110: result=operand1<<operand2;
4'b0111: result=operand1>>>operand2;
4'b1000: result=(operand1<operand2)?32'd1:32'd0;
default: result=result;
endcase
end
assign zero=(result==32'b0);
assign LessThan=result[31];
endmodule 


module Extendor(input logic [2:0] EXT_CODE, input logic [24:0]IMMIN, output logic [31:0]IMMOUT); //extendor will have [31:7]
always_comb
begin
case(EXT_CODE)
3'b000: IMMOUT = {{20{IMMIN[24]}}, IMMIN[24:13]}; //lw,immediate
3'b001: IMMOUT = {{20{IMMIN[24]}}, IMMIN[24:18], IMMIN[4:0]};//s-type
3'b010: IMMOUT = {{20{IMMIN[24]}}, IMMIN[24], IMMIN[0], IMMIN[23:18],IMMIN[4:1],1'b0};//branch
3'b011: IMMOUT = {{11{IMMIN[24]}}, IMMIN[24], IMMIN[12:5], IMMIN[13], IMMIN[23:14], 1'b0 }; //jump
3'b111: IMMOUT = {IMMIN[24:5],{12{1'b0}}};//Lui
default: IMMOUT=IMMOUT;
endcase

end

endmodule

module flop #(parameter WIDTH) (input logic clk, reset, input logic [WIDTH-1:0] D, output logic [WIDTH-1:0] Q);

always_ff @(posedge clk, negedge reset)
begin
if(!reset)
Q<=0;
else
Q<=D;
end
endmodule
module flop_enabled #(parameter WIDTH) (input logic clk, reset,enable, input logic [WIDTH-1:0] D, output logic [WIDTH-1:0] Q);

always_ff @(posedge clk, negedge reset)
begin
if(!reset)
Q<=0;
else if(enable)
Q<=D;
end
endmodule

module MUX2TO1 #(parameter WIDTH=8)(input logic [WIDTH-1:0] MUXIN1, MUXIN2, input logic select, output logic [WIDTH-1:0] MUXOUT);
assign MUXOUT = select?MUXIN2:MUXIN1;
endmodule

module MUX4TO1 #(parameter WIDTH=8)(input logic [WIDTH-1:0] MUXIN1, MUXIN2,MUXIN3,MUXIN4 , input logic [1:0] select, output logic [WIDTH-1:0] MUXOUT);
always_comb
begin
case(select)
2'b00: MUXOUT=MUXIN1;
2'b01: MUXOUT=MUXIN2;
2'b10: MUXOUT=MUXIN3;
2'b11: MUXOUT=MUXIN4;
default: MUXOUT=MUXOUT;
endcase
end
endmodule
module MUX3TO1 #(parameter WIDTH)(input logic [WIDTH-1:0] MUXIN1, MUXIN2,MUXIN3, input logic [1:0] select, output logic [WIDTH-1:0] MUXOUT);
always_comb
begin
case(select)
2'b00: MUXOUT=MUXIN1;
2'b01: MUXOUT=MUXIN2;
2'b10: MUXOUT=MUXIN3;
default: MUXOUT=MUXOUT;
endcase
end
endmodule

module SimpleAdder(input logic [31:0]INPUT1, INPUT2, output logic [31:0] OUTPUT);
always_comb
begin
OUTPUT= INPUT1+INPUT2;
end

endmodule

module main_dec(output logic LUI, input logic [6:0] OPCODE, output logic [1:0] M2, output logic WER,WEM,M1,J, output logic [2:0]EXT_CODE, output logic [2:0] ALUOPDEC);
always_comb
begin
case(OPCODE)
7'b0000011:begin  WER=1; WEM=0; M1=1;  LUI=0; M2=2'b01;   J=0; ALUOPDEC=3'b010; EXT_CODE=3'b000;  end //load
7'b0010011:begin  WER=1; WEM=0; M1=1;  LUI=0; M2=2'b00;  J=0; ALUOPDEC=3'b000; EXT_CODE=3'b000;  end //addi..
7'b1100111:begin  WER=1; WEM=0; M1=1;  LUI=0; M2=2'b10;  J=1; ALUOPDEC=3'b000; EXT_CODE=3'b000;  end //jalr
7'b0100011:begin  WER=0; WEM=1; M1=1;  LUI=0;   J=0; ALUOPDEC=3'b010; EXT_CODE=3'b001;  end //s-type
7'b0110011:begin  WER=1; WEM=0; M1=0;  LUI=0; M2=2'b00;  J=0; ALUOPDEC=3'b100;   end //R-type
7'b0110111:begin  WER=1; WEM=0; M2=2'b11; LUI=1;  J=0; EXT_CODE=3'b111;   end //U-type
7'b1100011:begin  WER=0; WEM=0; M1=0;  J=0;  ALUOPDEC=3'b110; EXT_CODE=3'b010;   end //B-type
7'b1101111:begin  WER=1; WEM=0; M2=2'b10;   LUI=0;  J=1;  EXT_CODE=3'b011;   end //J
default:  begin WER=0; WEM=0;J=0; LUI=0; end //make sure if a false OPCODE was inserted no writing to existing data occurs
endcase

end
endmodule
module ALUDEC(input logic [2:0] ALUOPDEC, input logic [2:0] FUNCT3,input logic [6:0] FUNCT7 , output logic [3:0] ALUOP, output logic [1:0] M3);
always_comb
begin
case(ALUOPDEC)
3'b000:begin  
	case(FUNCT3)
	3'b000:begin ALUOP=4'b0000; M3=2'b00; end  //addi
	3'b010:begin ALUOP=4'b1000; M3=2'b00; end //slti
	3'b100:begin ALUOP=4'b0100; M3=2'b00; end //xori
	3'b110:begin ALUOP=4'b0011; M3=2'b00; end //ori
	3'b111:begin ALUOP=4'b0010; M3=2'b00; end //andi
	3'b001:begin ALUOP=4'b0110; M3=2'b00; end //slli
	3'b101:begin if(FUNCT7==7'd0) begin ALUOP=4'b0101; M3=2'b00; end if(FUNCT7==7'b0100000) begin ALUOP=4'b0111; M3=2'b00; end end //srl, sra
	default: begin ALUOP=4'bxxxx; M3=2'b00; end //ALUOP could be anything it won't matter as long as write controls are LOW
	endcase   end

3'b100:begin   if(FUNCT7==7'd0) begin
	case(FUNCT3)
	3'b000:begin ALUOP=4'b0000; M3=2'b00; end  //add
	3'b010:begin ALUOP=4'b1000; M3=2'b00; end //slt
	3'b100:begin ALUOP=4'b0100; M3=2'b00; end //xor
	3'b110:begin ALUOP=4'b0011; M3=2'b00; end //or
	3'b111:begin ALUOP=4'b0010; M3=2'b00; end //and
	3'b001:begin ALUOP=4'b0110; M3=2'b00; end //sll
	3'b101:begin  ALUOP=4'b0101; M3=2'b00; end 
	default: begin ALUOP=4'bxxxx; M3=2'b00; end
	endcase   end
   
if(FUNCT7==7'b0100000) begin  case(FUNCT3) 
	 3'b000:begin ALUOP=4'b0001; M3=2'b00; end //sub
	 3'b101:begin ALUOP=4'b0111; M3=2'b00; end //sra
	 default: begin ALUOP=4'bxxxx; M3=2'b00; end
	 endcase end
end
3'b110:begin
case(FUNCT3)
3'b000: begin ALUOP=4'b0001; M3=2'b01; end //beq
3'b100: begin ALUOP=4'b0001; M3=2'b10; end //blt
default:begin ALUOP=4'bxxxx; M3=2'b00; end
endcase

end
3'b010:begin if(FUNCT3==3'b010)begin ALUOP=4'b0000; M3=2'b00; end end //lw,sw
default: begin ALUOP=4'bxxxx; M3=2'b00; end
endcase
end
endmodule

module data_memory(input logic [31:0] B1,WDM,input logic WEM,clk, output logic [31:0] RDB1);
logic [31:0] RAM [63:0];  initial RAM[7]=39;
always_comb
begin
RDB1=RAM[B1[31:2]];
end

//always_ff@(posedge clk)
//begin if(WEM) 
//RAM[B1[31:2]]<=WDM;
//end
endmodule

module instruction_memory(input logic [31:0] PC, output logic [31:0] instruction);

logic [31:0] instruction_ROM[63:0];
always_comb begin 
instruction=instruction_ROM[PC[31:2]]; 
end
endmodule
module REGFILE (input logic LUI,input logic [4:0] A1,A2,A3, input logic WER,clk,reset, input logic [31:0] WDR, output logic [31:0]RDA1, RDA2);
logic [31:0] REGS[31:0];


always_ff @(negedge clk, negedge reset)
begin
if(!reset)
begin for (int i=0; i<32; i++)  begin REGS[i]<=0;end end
else begin if (!clk) begin if(WER) begin if(LUI)begin REGS[A3][31:12]<=WDR[31:12];  end else if(A3!=0)begin  REGS[A3]<=WDR; end end end
end
end


always_comb
begin
RDA1=REGS[A1];
RDA2=REGS[A2];
end
endmodule

module test_bench_pipelined();
logic clk,reset;

RV32I_Pipelined RV32I_Pipelined(clk,reset);
initial begin $readmemh("instruction_memory.txt", RV32I_Pipelined.dp.IM1.instruction_ROM); end
initial begin reset<=1; #22 reset<=0; #5 reset<=1; end
initial
begin

clk<=0;
forever begin #5 clk<=~clk; end
end

initial begin
$monitor("m2 %b m2output %h a3q5 %d a1q3 %d luiselect1 %b h1out %h m5 %b luise2 %b h2out %h m6 %b Time is %d  reset is %b clk is %b current instruction is %h, \n REGs from 0 to 18 are: x00 is %h x01 is %h x02 is %h x03 is %h  \n x04 is %h x05 is %h x06 is %h x07 is %h x08 is %h \n x09 is %h x10 is %h x11 is %h x12 is %h \n x13 is %h x14 is %h x15 is %h \n x16 is %h x17 is %h x18 is %h"
,RV32I_Pipelined.control_unit.M2,RV32I_Pipelined.dp.MUX2OUTPUT,RV32I_Pipelined.dp.A3_Q5,RV32I_Pipelined.dp.A1_Q3,RV32I_Pipelined.lui_select1,RV32I_Pipelined.dp.HZMUX1OUTPUT,RV32I_Pipelined.dp.M5,
RV32I_Pipelined.lui_select2,RV32I_Pipelined.dp.HZMUX2OUTPUT,RV32I_Pipelined.dp.M6,$time,
reset,clk,
RV32I_Pipelined.Instruction,RV32I_Pipelined.dp.regfile.REGS[0],RV32I_Pipelined.dp.regfile.REGS[1],RV32I_Pipelined.dp.regfile.REGS[2],
RV32I_Pipelined.dp.regfile.REGS[3],RV32I_Pipelined.dp.regfile.REGS[4],RV32I_Pipelined.dp.regfile.REGS[5],RV32I_Pipelined.dp.regfile.REGS[6],
RV32I_Pipelined.dp.regfile.REGS[7],RV32I_Pipelined.dp.regfile.REGS[8],RV32I_Pipelined.dp.regfile.REGS[9],RV32I_Pipelined.dp.regfile.REGS[10],
RV32I_Pipelined.dp.regfile.REGS[11],RV32I_Pipelined.dp.regfile.REGS[12],RV32I_Pipelined.dp.regfile.REGS[13],RV32I_Pipelined.dp.regfile.REGS[14],
RV32I_Pipelined.dp.regfile.REGS[15],RV32I_Pipelined.dp.regfile.REGS[16],RV32I_Pipelined.dp.regfile.REGS[17],RV32I_Pipelined.dp.regfile.REGS[18],
);

end

endmodule
