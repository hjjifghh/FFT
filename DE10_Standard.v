// `define ENABLE_HPS
`define ENABLE_HSMC

module DE10_Standard
       (
           ///////// CLOCK /////////
           input CLOCK2_50,
           input CLOCK3_50,
           input CLOCK4_50,
           input CLOCK_50,
           ///////// KEY /////////
           input [3: 0]KEY,

           ///////// SW /////////
           input [9: 0]SW,

           ///////// LED /////////
           output [9: 0]LEDR,

           ///////// Seg7 /////////
           output [6: 0]HEX0,
           output [6: 0]HEX1,
           output [6: 0]HEX2,
           output [6: 0]HEX3,
           output [6: 0]HEX4,
           output [6: 0]HEX5,

           ///////// SDRAM /////////
           output DRAM_CLK,
           output DRAM_CKE,
           output [12: 0]DRAM_ADDR,
           output [1: 0]DRAM_BA,
           inout [15: 0]DRAM_DQ,
           output DRAM_LDQM,
           output DRAM_UDQM,
           output DRAM_CS_N,
           output DRAM_WE_N,
           output DRAM_CAS_N,
           output DRAM_RAS_N,

           ///////// Video-In /////////
           input TD_CLK27,
           input TD_HS,
           input TD_VS,
           input [7: 0]TD_DATA,
           output TD_RESET_N,

           ///////// VGA /////////
           output VGA_CLK,
           output VGA_HS,
           output VGA_VS,
           output [7: 0]VGA_R,
           output [7: 0]VGA_G,
           output [7: 0]VGA_B,
           output VGA_BLANK_N,
           output VGA_SYNC_N,

           ///////// Audio /////////
           inout AUD_BCLK,
           output AUD_XCK,
           inout AUD_ADCLRCK,
           input AUD_ADCDAT,
           inout AUD_DACLRCK,
           output AUD_DACDAT,

           ///////// PS2 /////////
           inout PS2_CLK,
           inout PS2_CLK2,
           inout PS2_DAT,
           inout PS2_DAT2,

           ///////// ADC /////////
           output ADC_SCLK,
           input ADC_DOUT,
           output ADC_DIN,
           output ADC_CONVST,

           ///////// I2C for Audio and Video-In /////////
           output FPGA_I2C_SCLK,
           inout FPGA_I2C_SDAT,

           ///////// IR /////////
           output IRDA_TXD,
           input IRDA_RXD,

`ifdef ENABLE_HSMC
           ///////// HSMC /////////
           input HSMC_CLKIN_P1,
           input HSMC_CLKIN_N1,
           input HSMC_CLKIN_P2,
           input HSMC_CLKIN_N2,
           output HSMC_CLKOUT_P1,
           output HSMC_CLKOUT_N1,
           output HSMC_CLKOUT_P2,
           output HSMC_CLKOUT_N2,
           inout [16: 0]HSMC_TX_D_P,
           inout [16: 0]HSMC_TX_D_N,
           inout [16: 0]HSMC_RX_D_P,
           inout [16: 0]HSMC_RX_D_N,
           input HSMC_CLKIN0,
           output HSMC_CLKOUT0,
           inout [3: 0]HSMC_D,
           output HSMC_SCL,
           inout HSMC_SDA,
`endif /*ENABLE_HSMC*/

`ifdef ENABLE_HPS
           ///////// HPS /////////
           inout HPS_CONV_USB_N,
           output [14: 0]HPS_DDR3_ADDR,
           output [2: 0]HPS_DDR3_BA,
           output HPS_DDR3_CAS_N,
           output HPS_DDR3_CKE,
           output HPS_DDR3_CK_N,
           output HPS_DDR3_CK_P,
           output HPS_DDR3_CS_N,
           output [3: 0]HPS_DDR3_DM,
           inout [31: 0]HPS_DDR3_DQ,
           inout [3: 0]HPS_DDR3_DQS_N,
           inout [3: 0]HPS_DDR3_DQS_P,
           output HPS_DDR3_ODT,
           output HPS_DDR3_RAS_N,
           output HPS_DDR3_RESET_N,
           input HPS_DDR3_RZQ,
           output HPS_DDR3_WE_N,
           output HPS_ENET_GTX_CLK,
           inout HPS_ENET_INT_N,
           output HPS_ENET_MDC,
           inout HPS_ENET_MDIO,
           input HPS_ENET_RX_CLK,
           input [3: 0]HPS_ENET_RX_DATA,
           input HPS_ENET_RX_DV,
           output [3: 0]HPS_ENET_TX_DATA,
           output HPS_ENET_TX_EN,
           inout [3: 0]HPS_FLASH_DATA,
           output HPS_FLASH_DCLK,
           output HPS_FLASH_NCSO,
           inout HPS_GSENSOR_INT,
           inout HPS_I2C1_SCLK,
           inout HPS_I2C1_SDAT,
           inout HPS_I2C2_SCLK,
           inout HPS_I2C2_SDAT,
           inout HPS_I2C_CONTROL,
           inout HPS_KEY,
           inout HPS_LCM_BK,
           inout HPS_LCM_D_C,
           inout HPS_LCM_RST_N,
           output HPS_LCM_SPIM_CLK,
           output HPS_LCM_SPIM_MOSI,
           output HPS_LCM_SPIM_SS,
           input HPS_LCM_SPIM_MISO,
           inout HPS_LED,
           inout HPS_LTC_GPIO,
           output HPS_SD_CLK,
           inout HPS_SD_CMD,
           inout [3: 0]HPS_SD_DATA,
           output HPS_SPIM_CLK,
           input HPS_SPIM_MISO,
           output HPS_SPIM_MOSI,
           inout HPS_SPIM_SS,
           input HPS_UART_RX,
           output HPS_UART_TX,
           input HPS_USB_CLKOUT,
           inout [7: 0]HPS_USB_DATA,
           input HPS_USB_DIR,
           input HPS_USB_NXT,
           output HPS_USB_STP,
`endif /*ENABLE_HPS*/

           ///////// GPIO /////////
           inout [35: 0]GPIO
       );

`ifdef ENABLE_HSMC 
///////// HSMC /////////
`define TP HSMC_TX_D_P
`define TN HSMC_TX_D_N
`define RP HSMC_RX_D_P
`define RN HSMC_RX_D_N

`define D HSMC_D

`define CLKIN0 HSMC_CLKIN0
`define CLKOUT0 HSMC_CLKOUT0
`define CLKIN_N2 HSMC_CLKIN_N2
`define CLKOUT_N2 HSMC_CLKOUT_N2

`define PA15 `TP[0]
`define PA14 `TN[0]
`define PA13 `TP[1]
`define PA12 `TN[1]
`define PA11 `TP[2]
`define PA10 `TN[2]
`define PA9 `TP[3]
`define PA8 `TN[3]
`define PA7 `TP[4]
`define PA6 `TN[4]
`define PA5 `TP[5]
`define PA4 `TN[5]
`define PA3 `TP[6]
`define PA2 `TN[6]
`define PA1 `TP[7]
`define PA0 `TN[7]

`define PB15 `RP[0]
`define PB14 `RN[0]
`define PB13 `RP[1]
`define PB12 `RN[1]
`define PB11 `RP[2]
`define PB10 `RN[2]
`define PB9 `RP[3]
`define PB8 `RN[3]
`define PB7 `RP[4]
`define PB6 `RN[4]
`define PB5 `RP[5]
`define PB4 `RN[5]
`define PB3 `RP[6]
`define PB2 `RN[6]
`define PB1 `RP[7]
`define PB0 `RN[7]

`define PC15 `TP[8]
`define PC14 `TN[8]
`define PC13 `TP[9]
`define PC12 `TN[9]
`define PC11 `TP[10]
`define PC10 `TN[10]
`define PC9 `TP[11]
`define PC8 `TN[11]
`define PC7 `TP[12]
`define PC6 `TN[12]
`define PC5 `TP[13]
`define PC4 `TN[13]
`define PC3 `TP[14]
`define PC2 `TN[14]
`define PC1 `TP[15]
`define PC0 `TN[15]

`define PD15 `RP[8]
`define PD14 `RN[8]
`define PD13 `RP[9]
`define PD12 `RN [9]
`define PD11 `RP[10]
`define PD10 `RN[10]
`define PD9 `RP[11]
`define PD8 `RN[11]
`define PD7 `RP[12]
`define PD6 `RN[12]
`define PD5 `RP[13]
`define PD4 `RN[13]
`define PD3 `RP[14]
`define PD2 `RN[14]
`define PD1 `RP[15]
`define PD0 `RN[15]

`endif /*PIN_HSMC*/

wire RST_N;

reg [1: 0]rst_n_cnt;
reg soft_rst_n;

always@(posedge CLK_100M)
begin
	rst_n_cnt <= (rst_n_cnt < 2'd3) ? rst_n_cnt + 1'd1 : rst_n_cnt;
	if (rst_n_cnt == 2'd2)
	begin
		soft_rst_n <= 1'd0;
	end
	else if (rst_n_cnt == 2'd3)
	begin
		soft_rst_n <= 1'd1;
	end
end

assign RST_N = KEY[0] & soft_rst_n;

wire CLK_100M;
wire CLK_40M;
wire CLK_20M;

PLL PLL_inst
    (
        .refclk(CLOCK_50) , // input  refclk_sig
        .rst(!RST_N) , // input  rst_sig
        .outclk_0(CLK_100M) , // output  outclk_0_sig
        .outclk_1(CLK_40M) , // output  outclk_1_sig
        .locked() // output  locked_sig
    );

ClkDiv #(32'd2, 32'd1)ClkDiv_inst
       (
           .clk(CLK_40M) , // input  clk_sig
           .rst_n(RST_N) , // input  rst_n_sig
           .phase_rst(1'd0) , // input  phase_rst_sig
           .clk_div(CLK_20M) , // output  clk_div_sig
           .cnt() // output [31:0] cnt_sig
       );

wire HEX0P;
wire HEX1P;
wire HEX2P;
wire HEX3P;
wire HEX4P;
wire HEX5P;

// nios
nios u0
     (

         .clk_clk(CLOCK_50), //clk.clk
         .reset_reset_n(soft_rst_n), //reset.reset_n
         .pll_sdam_clk(DRAM_CLK), //pll_sdam.clk

         .key_external_connection_export(KEY), //key_external_connection.export
         .sw_external_connection_export(SW), //sw_external_connection.export

         .led_external_connection_export(LEDR), //led_external_connection.export
         .seg7_conduit_end_export({HEX5P, HEX5, HEX4P, HEX4,
                                   HEX3P, HEX3, HEX2P, HEX2,
                                   HEX1P, HEX1, HEX0P, HEX0}), //seg7_conduit_end.export

         .sdram_wire_addr(DRAM_ADDR), //sdram_wire.addr
         .sdram_wire_ba(DRAM_BA), //.ba
         .sdram_wire_cas_n(DRAM_CAS_N), //.cas_n
         .sdram_wire_cke(DRAM_CKE), //.cke
         .sdram_wire_cs_n(DRAM_CS_N), //.cs_n
         .sdram_wire_dq(DRAM_DQ), //.dq
         .sdram_wire_dqm({DRAM_UDQM, DRAM_LDQM}), //.dqm
         .sdram_wire_ras_n(DRAM_RAS_N), //.ras_n
         .sdram_wire_we_n(DRAM_WE_N), //.we_n

         // // UART
         // .uart_external_connection_txd(TX) ,// output  uart_external_connection_txd_sig
         // .uart_external_connection_rxd(RX) ,// input  uart_external_connection_rxd_sig

         // UART2
         .data_byte_out_external_connection_export(data_byte_out) , // output [7:0] data_byte_out_external_connection_sig
         .transmit_en_out_external_connection_export(transmit_en_out) , // output transmit_en_out_external_connection_sig
         .transmit_irq_external_connection_export(transmit_irq) , // output transmit_irq_external_connection_sig
         .data_frame_in_external_connection_export(data_frame_in) , // output [23:0] data_frame_in_external_connection_sig
         .receive_irq_external_connection_export(receive_irq) , // output receive_irq_external_connection_sig

         // // SPI

         // Other
     );

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UART
wire TX;
wire RX;

wire [7: 0]data_byte_out;
wire transmit_en_out;
wire transmit_irq;
wire [23: 0]data_frame_in;
wire receive_irq;

UartByteTX #(32'd921600)UartByteTX_inst
           (
               .clk_100M(CLK_100M) , // input  clk_100M_sig
               .rst_n(RST_N) , // input  rst_n_sig
               .data_byte(data_byte_out) , // input [7:0] data_byte_sig
               .transmit_en(transmit_en_out) , // input  transmit_en_sig
               .TX(TX) , // output  TX_sig
               .irq(transmit_irq) // output  irq_sig
           );
UartFrameRX #(32'd921600)UartFrameRX_inst
            (
                .clk_100M(CLK_100M) , // input  clk_100M_sig
                .rst_n(RST_N) , // input  rst_n_sig
                .RX(RX) , // input  RX_sig
                .data_frame(data_frame_in) , // output [23:0] data_frame_sig
                .irq(receive_irq) // output  irq_sig
            );

(*keep*)wire [8:0]index;
 (*keep*)wire [9:0]peak_value;
(*keep*)wire [9:0]freindex;
(*keep*)wire[30:0] freindex2;
(*keep*)wire[31:0]message;
(*keep*)wire[31:0]message1;
(*keep*)wire[7:0]value255;
f_calculate f(
.clk(CLOCK_50),
.rst_n(RST_N),
.message(message),
.message1(message1),
.peak_value(peak_value),
.peak_index(freindex),//未经过换算
.peak_index2(freindex2),//经过换算
.index(index),
.value255(value255))
 ;


//assign RX = `D[1];
//assign `D[3] = TX;

/////////////////////////////
// wire

// reg

endmodule
