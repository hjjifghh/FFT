`timescale 1ns/1ps
module tb_fft_1024();
    reg clk;
    reg reset;
    reg start;
    reg [15:0] data_in;
    reg data_valid;
    wire [15:0] real_out;
    wire [15:0] imag_out;
    wire out_valid;
    wire ready;
    
    // 实例化FFT模块
    fft_1024 uut (
        .clk(clk),
        .reset(reset),
        .start(start),
        .data_in(data_in),
        .data_valid(data_valid),
        .real_out(real_out),
        .imag_out(imag_out),
        .out_valid(out_valid),
        .ready(ready)
    );
    
    // 生成时钟信号
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz时钟(10ns周期)
    end
    
    // 测试激励
    initial begin
        reset = 1;
        start = 0;
        data_in = 0;
        data_valid = 0;
        
        #20 reset = 0;
        
        // 等待模块就绪
        wait(ready == 1);
        
        // 开始加载数据
        start = 1;
        data_valid = 1;
        
        // 加载1024个样本
        for (integer i = 0; i < 1024; i = i + 1) begin
            data_in = i; // 简单递增测试数据
            #10;
        end
        
        data_valid = 0;
        start = 0;
        
        // 等待处理完成
        wait(out_valid == 1);
        
        // 读取输出
        for (integer j = 0; j < 1024; j = j + 1) begin
            #10;
            $display("Output[%d]: Real = %d, Imag = %d", j, real_out, imag_out);
        end
        
        #100 $finish;
    end
endmodule 
