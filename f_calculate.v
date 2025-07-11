module f_calculate(
	input clk,
	input rst_n,
   output [32:0]message,
   output[32:0]message1,
	output reg [15:0] peak_value,
	output reg [9:0]  peak_index,//未经过换算
	output reg [31:0]  peak_index2,//经过换算
	output reg [9:0]  index,
	output reg [31:0] peak_index22,
	output reg [9:0] peak_index1,
		output wire [7:0]value255
);
 reg [15:0]peak_value1;
reg [15:0] data_array [0:1023];
reg [30:0] fs=1000000;
reg [9:0] fs1=400;
reg [15:0] threshold1=500;//设置上下阈值
reg [15:0] threshold2=2000;
reg [31:0]max;
reg [31:0]max2;
wire [31:0] data_out;
wire source_valid;

(*preserve*)reg type0;
(*preserve*)reg type1;

fft_ram fft_ram_inst(
	.clk(clk),
	.rst_n(rst_n),
	
	.data_out(data_out),
	.source_valid(source_valid)
);
(*preserve*)reg [9:0]index3;
(*preserve*)reg [9:0]index4;
(*preserve*)reg [15:0]max3;
(*preserve*)reg [15:0]max4;
always@(posedge clk or negedge rst_n) 
begin
	if(!rst_n)
	begin
		index<=0;
		peak_value<=0;
		peak_value1<=0;
		peak_index<=0;
		max<=0;
		max2<=0;
		max3<=0;
		max4<=0;
		index3<=0;
		index4<=0;
		type0<=0;
		type1<=0;
	end
	else begin
		if(source_valid)
		begin
			data_array[index]<=data_out;
			index<=index+1;
			if(data_out>max )
			begin
			   max<=data_out;
				peak_value<=data_out;
				peak_index<=index<512 ? index : 1024-index;
				peak_index2<=(peak_index*fs1/1024)%2==1?(peak_index*fs1/1024+1)*2500:(peak_index*fs1/1024)*2500;
			end
			else if(data_out<max&&data_out>max2&&data_out>max/10&& peak_index!=index&&peak_index!=1024-index)
			begin
			peak_value1<=data_out;
			   max2<=data_out;
				peak_index1<=index<512 ? index : 1024-index;
				peak_index22<=(peak_index1*fs1/1024)%2==1?(peak_index1*fs1/1024+1)*2500:(peak_index1*fs1/1024)*2500;
			end
		 if(data_out<peak_value/4&&data_out>(peak_value/8)&&peak_value>20000)
		   begin
            max3<=data_out;
			index3<=index<512 ? index : 1024-index;

		    end
		else if(data_out<peak_value1/4&&data_out>(peak_value1/10)&&peak_value1>5000)
		   begin
			max4<=data_out;
			index4<=index<512 ? index : 1024-index;
			index4<=((3*peak_index1-3<index4)&&(index4<=3*peak_index1+3))?index4:0;
		 end
		if(index3!=0&&index4!=0)
		begin
            type0=1;
			type1=1;
		end
		else if((index3!=0&&index4==0)||(index4!=0&&index3==0))
		begin
            if((index3>peak_index*3?index3-peak_index*3:peak_index*3-index3)<(index4>peak_index*3?index4-peak_index*3:peak_index*3-index4))
			begin
            type0=1;
				type1=0;
			end
			else
			begin
			type0=0;
			type1=1;
			end
		end
	
		else
		begin
            type0=0;
			type1=0;
		end
		if(index==511&&peak_index1==0)
		begin
		
			peak_index1<=peak_index;
				peak_index22<=(peak_index1*fs1/1024)%2==1?(peak_index1*fs1/1024+1)*2500:(peak_index1*fs1/1024)*2500;
		end
	end
end
end
 assign message={type0,peak_index2};
 assign message1={type1,peak_index22};
endmodule