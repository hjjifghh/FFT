module fft_1024 (
    input wire clk,              // 100MHz时钟
    input wire reset,           // 异步复位
    input wire start,           // 开始FFT计算
    input wire [15:0] data_in,  // 输入数据(16位)
    input wire data_valid,      // 输入数据有效
    output reg [15:0] real_out, // 实部输出(16位)
    output reg [15:0] imag_out, // 虚部输出(16位)
    output reg out_valid,       // 输出有效
    output reg ready            // 模块准备好接收新数据
);

// 参数定义
parameter N = 1024;             // FFT点数
parameter DATA_WIDTH = 16;      // 数据位宽
parameter STAGES = 10;          // log2(1024) = 10级

// 内部信号
reg [9:0] addr_in;              // 输入地址(0-1023)
reg [9:0] addr_out;             // 输出地址(0-1023)
reg [STAGES-1:0] stage_count;   // 级数计数
reg [9:0] butterfly_count;      // 蝶形运算计数
reg [9:0] group_count;          // 组计数
reg [9:0] twiddle_addr;         // 旋转因子地址
reg processing;                 // 处理状态标志

// 双端口RAM用于存储输入数据和中间结果
reg [DATA_WIDTH-1:0] ram [0:N-1];
reg [DATA_WIDTH-1:0] ram_out;

// 旋转因子ROM (预先计算好的)
reg [DATA_WIDTH-1:0] twiddle_real [0:N/2-1];
reg [DATA_WIDTH-1:0] twiddle_imag [0:N/2-1];

// 蝶形运算中间结果
reg [DATA_WIDTH-1:0] a_real, a_imag;
reg [DATA_WIDTH-1:0] b_real, b_imag;
reg [DATA_WIDTH-1:0] tw_real, tw_imag;
wire [DATA_WIDTH-1:0] b_tw_real, b_tw_imag;
wire [DATA_WIDTH-1:0] sum_real, sum_imag;
wire [DATA_WIDTH-1:0] diff_real, diff_imag;

// 复数乘法模块实例化
complex_mult cmult (
    .a_real(b_real),
    .a_imag(b_imag),
    .b_real(tw_real),
    .b_imag(tw_imag),
    .out_real(b_tw_real),
    .out_imag(b_tw_imag)
);

// 复数加减模块
assign sum_real = a_real + b_tw_real;
assign sum_imag = a_imag + b_tw_imag;
assign diff_real = a_real - b_tw_real;
assign diff_imag = a_imag - b_tw_imag;

// 状态机定义
parameter IDLE = 2'b00;
parameter LOAD = 2'b01;
parameter PROCESS = 2'b10;
parameter OUTPUT = 2'b11;

reg [1:0] state;

// 主状态机
always @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= IDLE;
        addr_in <= 0;
        addr_out <= 0;
        stage_count <= 0;
        butterfly_count <= 0;
        group_count <= 0;
        out_valid <= 0;
        ready <= 1;
        processing <= 0;
    end else begin
        case (state)
            IDLE: begin
                ready <= 1;
                if (start) begin
                    state <= LOAD;
                    addr_in <= 0;
                    ready <= 0;
                end
            end
            
            LOAD: begin
                if (data_valid) begin
                    ram[addr_in] <= data_in;
                    if (addr_in == N-1) begin
                        state <= PROCESS;
                        stage_count <= 0;
                        processing <= 1;
                    end else begin
                        addr_in <= addr_in + 1;
                    end
                end
            end
            
            PROCESS: begin
                // FFT处理逻辑
                if (stage_count < STAGES) begin
                    // 执行蝶形运算
                    // 这里简化了实际的蝶形运算地址生成逻辑
                    // 实际实现需要更复杂的地址生成
                    
                    // 读取数据
                    a_real <= ram[butterfly_count];
                    a_imag <= 0; // 初始输入只有实部
                    b_real <= ram[butterfly_count + (1 << stage_count)];
                    b_imag <= 0; // 初始输入只有实部
                    
                    // 获取旋转因子
                    twiddle_addr <= butterfly_count[STAGES-1-stage_count:0] << stage_count;
                    tw_real <= twiddle_real[twiddle_addr];
                    tw_imag <= twiddle_imag[twiddle_addr];
                    
                    // 执行蝶形运算
                    // 结果写回RAM
                    ram[butterfly_count] <= sum_real;
                    // 虚部存储需要额外的RAM或交错存储
                    
                    // 更新计数器
                    if (butterfly_count < N-1) begin
                        butterfly_count <= butterfly_count + 1;
                    end else begin
                        butterfly_count <= 0;
                        stage_count <= stage_count + 1;
                    end
                end else begin
                    // 处理完成
                    processing <= 0;
                    state <= OUTPUT;
                    addr_out <= 0;
                end
            end
            
            OUTPUT: begin
                // 输出结果
                real_out <= ram[addr_out];
                // 虚部需要从另一个RAM读取或交错读取
                out_valid <= 1;
                
                if (addr_out == N-1) begin
                    state <= IDLE;
                    out_valid <= 0;
                    ready <= 1;
                end else begin
                    addr_out <= addr_out + 1;
                end
            end
        endcase
    end
end

// 初始化旋转因子ROM (实际实现中应预先计算好)
// 正确声明位置（模块作用域）
integer i;
real angle, cos_val, sin_val;

initial begin
    for (i = 0; i < N/2; i = i + 1) begin
        // 分步计算避免复杂表达式
        angle = 2.0 * 3.141592653589793 * i / N;
        cos_val = $cos(angle) * 32767.0;
        sin_val = $sin(angle) * 32767.0;
        
        // 赋值前确保数值在16位有符号范围内
        if (cos_val > 32767.0) cos_val = 32767.0;
        if (cos_val < -32768.0) cos_val = -32768.0;
        if (sin_val > 32767.0) sin_val = 32767.0;
        if (sin_val < -32768.0) sin_val = -32768.0;
        
        twiddle_real[i] = $rtoi(cos_val);
        twiddle_imag[i] = $rtoi(-sin_val);
    end
end

endmodule 
