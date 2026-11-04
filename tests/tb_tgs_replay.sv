`timescale 1ns/1ps
// File-driven reproducible replay, checked externally against Python integers.
module tb_tgs_replay;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,v0=0,v1=0;reg signed [15:0] x0=0,x1=0;reg [31:0] t=0;
 wire u0,u1,a0,a1;wire signed [31:0] f0,b0,d0,s0,f1,b1,d1,s1;
 wire [31:0] t0,c0,p0,e0,t1,c1,p1,e1;
 tgs_channel l0(clk,rstn,v0,x0,t,u0,f0,b0,d0,s0,t0,c0,p0,e0,a0);
 tgs_channel l1(clk,rstn,v1,x1,t,u1,f1,b1,d1,s1,t1,c1,p1,e1,a1);
 integer fi,fo,n,reset_value,vi0,vi1,xi0,xi1,stamp;
 reg [2047:0] input_path,output_path;
 initial begin
 if(!$value$plusargs("input=%s",input_path))begin $display("SKIP replay: invoke tools/verify_tgs_golden.py");$finish;end
 if(!$value$plusargs("output=%s",output_path))$fatal(1,"output missing");
 fi=$fopen(input_path,"r");fo=$fopen(output_path,"w");
 if(!fi || !fo)$fatal(1,"file open");
 while(!$feof(fi))begin
 n=$fscanf(fi,"%d %d %d %d %d %d\n",reset_value,vi0,xi0,vi1,xi1,stamp);
 if(n==6)begin
 @(negedge clk);rstn=!reset_value;v0=vi0;x0=xi0;v1=vi1;x1=xi1;t=stamp;
 @(posedge clk);#1;
 $fdisplay(fo,"%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
 u0,f0,b0,d0,s0,t0,c0,p0,e0,a0,u1,f1,b1,d1,s1,t1,c1,p1,e1,a1);
 end
 end
 $fclose(fi);$fclose(fo);$display("PASS replay file produced");$finish;
 end
endmodule
