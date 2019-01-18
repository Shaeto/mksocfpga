//
// The MIT License (MIT)
// Copyright (c) 2016 Intel Corporation
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

module DE10_Nano_SoC_DB25
       (
           //////////// ADC //////////
           output              ADC_CONVST,
           output              ADC_SCK,
           output              ADC_SDI,
           input               ADC_SDO,

           //////////// ARDUINO //////////
           inout       [15:0]  ARDUINO_IO,
           inout               ARDUINO_RESET_N,

           //////////// CLOCK //////////
           input               FPGA_CLK1_50,
           input               FPGA_CLK2_50,
           input               FPGA_CLK3_50,

           //////////// HDMI //////////
           inout               HDMI_I2C_SCL,
           inout               HDMI_I2C_SDA,
           inout               HDMI_I2S,
           inout               HDMI_LRCLK,
           inout               HDMI_MCLK,
           inout               HDMI_SCLK,
           output              HDMI_TX_CLK,
           output              HDMI_TX_DE,
           output      [23:0]  HDMI_TX_D,
           output              HDMI_TX_HS,
           input               HDMI_TX_INT,
           output              HDMI_TX_VS,

           //////////// HPS //////////
           inout               HPS_CONV_USB_N,
           output      [14:0]  HPS_DDR3_ADDR,
           output       [2:0]  HPS_DDR3_BA,
           output              HPS_DDR3_CAS_N,
           output              HPS_DDR3_CKE,
           output              HPS_DDR3_CK_N,
           output              HPS_DDR3_CK_P,
           output              HPS_DDR3_CS_N,
           output       [3:0]  HPS_DDR3_DM,
           inout       [31:0]  HPS_DDR3_DQ,
           inout        [3:0]  HPS_DDR3_DQS_N,
           inout        [3:0]  HPS_DDR3_DQS_P,
           output              HPS_DDR3_ODT,
           output              HPS_DDR3_RAS_N,
           output              HPS_DDR3_RESET_N,
           input               HPS_DDR3_RZQ,
           output              HPS_DDR3_WE_N,
           output              HPS_ENET_GTX_CLK,
           inout               HPS_ENET_INT_N,
           output              HPS_ENET_MDC,
           inout               HPS_ENET_MDIO,
           input               HPS_ENET_RX_CLK,
           input        [3:0]  HPS_ENET_RX_DATA,
           input               HPS_ENET_RX_DV,
           output       [3:0]  HPS_ENET_TX_DATA,
           output              HPS_ENET_TX_EN,
           inout               HPS_GSENSOR_INT,
           inout               HPS_I2C0_SCLK,
           inout               HPS_I2C0_SDAT,
           inout               HPS_I2C1_SCLK,
           inout               HPS_I2C1_SDAT,
           inout               HPS_KEY,
           inout               HPS_LED,
           inout               HPS_LTC_GPIO,
           output              HPS_SD_CLK,
           inout               HPS_SD_CMD,
           inout        [3:0]  HPS_SD_DATA,
           output              HPS_SPIM_CLK,
           input               HPS_SPIM_MISO,
           output              HPS_SPIM_MOSI,
           inout               HPS_SPIM_SS,
           input               HPS_UART_RX,
           output              HPS_UART_TX,
           input               HPS_USB_CLKOUT,
           inout        [7:0]  HPS_USB_DATA,
           input               HPS_USB_DIR,
           input               HPS_USB_NXT,
           output              HPS_USB_STP,

           //////////// KEY //////////
           input        [1:0]  KEY,

           //////////// LED //////////
           output       [7:0]  LED,

           //////////// SW //////////
           input        [3:0]  SW,

           //////////// GPIO_0, GPIO connect to GPIO Default //////////
           inout       [35:0]  GPIO_0,

           //////////// GPIO_1, GPIO connect to GPIO Default //////////
           inout       [35:0]  GPIO_1
       );

//REG/WIRE Declarations

// DE10-Nano Dev kit and I/O adaptors specific info
// import boardtype::*;

wire        hps_fpga_reset_n;
wire [1:0]  fpga_debounced_buttons;
wire [6:0]  fpga_led_internal;
wire [2:0]  hps_reset_req;
wire        hps_cold_reset;
wire        hps_warm_reset;
wire        hps_debug_reset;
wire [27:0] stm_hw_events;

wire        fpga_clk_50;

// hm2
localparam AddrWidth = 16;
localparam IOWidth   = 68;
localparam LIOWidth  = 0;

wire [AddrWidth-3:0]    hm_address;
wire [31:0]             hm_datao;
wire [31:0]             hm_datai;
wire                    hm_read;
wire                    hm_write;
wire                    hm_clk_med;
wire                    hm_clk_high;
wire                    clklow_sig;
wire                    clkmed_sig;
wire                    clkhigh_sig;
wire                    int_sig;

// connection of internal logics
assign LED[7:1] = fpga_led_internal;
assign fpga_clk_50=FPGA_CLK1_50;
assign stm_hw_events    = {{15{1'b0}}, SW, fpga_led_internal, fpga_debounced_buttons};

assign ARDUINO_RESET_N = hps_fpga_reset_n;

// i2c connection
wire hdmi_internal_scl_o_e;
wire hdmi_internal_scl_o;
wire hdmi_internal_sda_o_e;
wire hdmi_internal_sda_o;

ALT_IOBUF scl_iobuf (.i(1'b0), .oe(hdmi_internal_scl_o_e), .o(hdmi_internal_scl_o), .io(HDMI_I2C_SCL));
ALT_IOBUF sda_iobuf (.i(1'b0), .oe(hdmi_internal_sda_o_e), .o(hdmi_internal_sda_o), .io(HDMI_I2C_SDA));

// i2s audio
wire AUD_CTRL_CLK;
AUDIO_IF u_AVG(
        .clk(AUD_CTRL_CLK),
        .reset_n(hps_fpga_reset_n),
        .sclk(HDMI_SCLK),
        .lrclk(HDMI_LRCLK),
        .i2s(HDMI_I2S)
);

// arduino i2c connection
wire arduino_internal_scl_o_e;
wire arduino_internal_scl_o;
wire arduino_internal_sda_o_e;
wire arduino_internal_sda_o;

// arduino uart
//wire arduino_hps_0_uart1_rxd;
//wire arduino_hps_0_uart1_txd;

// arduino spi
wire arduino_hps_0_spim0_ss_0_n;
wire arduino_hps_0_spim0_ssi_oe_n;
wire arduino_hps_0_spim0_ss_in_n;
wire arduino_hps_0_spim0_txd;
wire arduino_hps_0_spim0_rxd;
wire arduino_hps_0_spim0_sclk_out_clk;

ALT_IOBUF arduino_scl_iobuf (.i(1'b0), .oe(arduino_internal_scl_o_e), .o(arduino_internal_scl_o), .io(ARDUINO_IO[15]));
ALT_IOBUF arduino_sda_iobuf (.i(1'b0), .oe(arduino_internal_sda_o_e), .o(arduino_internal_sda_o), .io(ARDUINO_IO[14]));

//uart
//ALT_IOBUF arduino_uart_rx_iobuf (.i(1'b0), .oe(1'b0), .o(arduino_hps_0_uart1_rxd), .io(ARDUINO_IO[0]));
//ALT_IOBUF arduino_uart_tx_iobuf (.i(arduino_hps_0_uart1_txd), .oe(1'b1), .o(), .io(ARDUINO_IO[1]));

//spim
ALT_IOBUF arduino_ss_iobuf (.i(arduino_hps_0_spim0_ss_0_n), .oe(1'b1), .o(), .io(ARDUINO_IO[10]));
ALT_IOBUF arduino_mosi_iobuf (.i(arduino_hps_0_spim0_txd), .oe(1'b1), .o(), .io(ARDUINO_IO[11]));
ALT_IOBUF arduino_miso_iobuf (.i(1'b0), .oe(1'b0), .o(arduino_hps_0_spim0_rxd), .io(ARDUINO_IO[12]));
ALT_IOBUF arduino_sck_iobuf (.i(arduino_hps_0_spim0_sclk_out_clk), .oe(1'b1), .o(), .io(ARDUINO_IO[13]));

// export hm2 int signal to arduino int0 pin (d2)
assign ARDUINO_IO[2] = int_sig;

// SoC sub-system module
soc_system u0 (
               //Clocks & Resets
               .clk_50_clk                            (FPGA_CLK1_50),
               .hps_0_h2f_reset_reset_n               (hps_fpga_reset_n),

               //DRAM
               .memory_mem_a                          (HPS_DDR3_ADDR),
               .memory_mem_ba                         (HPS_DDR3_BA),
               .memory_mem_ck                         (HPS_DDR3_CK_P),
               .memory_mem_ck_n                       (HPS_DDR3_CK_N),
               .memory_mem_cke                        (HPS_DDR3_CKE),
               .memory_mem_cs_n                       (HPS_DDR3_CS_N),
               .memory_mem_ras_n                      (HPS_DDR3_RAS_N),
               .memory_mem_cas_n                      (HPS_DDR3_CAS_N),
               .memory_mem_we_n                       (HPS_DDR3_WE_N),
               .memory_mem_reset_n                    (HPS_DDR3_RESET_N),
               .memory_mem_dq                         (HPS_DDR3_DQ),
               .memory_mem_dqs                        (HPS_DDR3_DQS_P),
               .memory_mem_dqs_n                      (HPS_DDR3_DQS_N),
               .memory_mem_odt                        (HPS_DDR3_ODT),
               .memory_mem_dm                         (HPS_DDR3_DM),
               .memory_oct_rzqin                      (HPS_DDR3_RZQ),

               //HPS Peripherals
               //enet
               .hps_0_hps_io_hps_io_emac1_inst_TX_CLK (HPS_ENET_GTX_CLK),
               .hps_0_hps_io_hps_io_emac1_inst_TXD0   (HPS_ENET_TX_DATA[0]),
               .hps_0_hps_io_hps_io_emac1_inst_TXD1   (HPS_ENET_TX_DATA[1]),
               .hps_0_hps_io_hps_io_emac1_inst_TXD2   (HPS_ENET_TX_DATA[2]),
               .hps_0_hps_io_hps_io_emac1_inst_TXD3   (HPS_ENET_TX_DATA[3]),
               .hps_0_hps_io_hps_io_emac1_inst_RXD0   (HPS_ENET_RX_DATA[0]),
               .hps_0_hps_io_hps_io_emac1_inst_RXD1   (HPS_ENET_RX_DATA[1]),
               .hps_0_hps_io_hps_io_emac1_inst_RXD2   (HPS_ENET_RX_DATA[2]),
               .hps_0_hps_io_hps_io_emac1_inst_RXD3   (HPS_ENET_RX_DATA[3]),
               .hps_0_hps_io_hps_io_emac1_inst_MDIO   (HPS_ENET_MDIO),
               .hps_0_hps_io_hps_io_emac1_inst_MDC    (HPS_ENET_MDC),
               .hps_0_hps_io_hps_io_emac1_inst_RX_CTL (HPS_ENET_RX_DV),
               .hps_0_hps_io_hps_io_emac1_inst_TX_CTL (HPS_ENET_TX_EN),
               .hps_0_hps_io_hps_io_emac1_inst_RX_CLK (HPS_ENET_RX_CLK),
               //SDMMC
               .hps_0_hps_io_hps_io_sdio_inst_CLK     (HPS_SD_CLK),
               .hps_0_hps_io_hps_io_sdio_inst_CMD     (HPS_SD_CMD),
               .hps_0_hps_io_hps_io_sdio_inst_D0      (HPS_SD_DATA[0]),
               .hps_0_hps_io_hps_io_sdio_inst_D1      (HPS_SD_DATA[1]),
               .hps_0_hps_io_hps_io_sdio_inst_D2      (HPS_SD_DATA[2]),
               .hps_0_hps_io_hps_io_sdio_inst_D3      (HPS_SD_DATA[3]),
               //USB1
               .hps_0_hps_io_hps_io_usb1_inst_D0      (HPS_USB_DATA[0]),
               .hps_0_hps_io_hps_io_usb1_inst_D1      (HPS_USB_DATA[1]),
               .hps_0_hps_io_hps_io_usb1_inst_D2      (HPS_USB_DATA[2]),
               .hps_0_hps_io_hps_io_usb1_inst_D3      (HPS_USB_DATA[3]),
               .hps_0_hps_io_hps_io_usb1_inst_D4      (HPS_USB_DATA[4]),
               .hps_0_hps_io_hps_io_usb1_inst_D5      (HPS_USB_DATA[5]),
               .hps_0_hps_io_hps_io_usb1_inst_D6      (HPS_USB_DATA[6]),
               .hps_0_hps_io_hps_io_usb1_inst_D7      (HPS_USB_DATA[7]),
               .hps_0_hps_io_hps_io_usb1_inst_CLK     (HPS_USB_CLKOUT),
               .hps_0_hps_io_hps_io_usb1_inst_STP     (HPS_USB_STP),
               .hps_0_hps_io_hps_io_usb1_inst_DIR     (HPS_USB_DIR),
               .hps_0_hps_io_hps_io_usb1_inst_NXT     (HPS_USB_NXT),
               //UART0
               .hps_0_hps_io_hps_io_uart0_inst_RX     (HPS_UART_RX),
               .hps_0_hps_io_hps_io_uart0_inst_TX     (HPS_UART_TX),
               //SPIM1
               .hps_0_hps_io_hps_io_spim1_inst_CLK    (HPS_SPIM_CLK),
               .hps_0_hps_io_hps_io_spim1_inst_MOSI   (HPS_SPIM_MOSI),
               .hps_0_hps_io_hps_io_spim1_inst_MISO   (HPS_SPIM_MISO),
               .hps_0_hps_io_hps_io_spim1_inst_SS0    (HPS_SPIM_SS),
               //I2C 0,1
               .hps_0_hps_io_hps_io_i2c0_inst_SDA     (HPS_I2C0_SDAT),
               .hps_0_hps_io_hps_io_i2c0_inst_SCL     (HPS_I2C0_SCLK),
               .hps_0_hps_io_hps_io_i2c1_inst_SDA     (HPS_I2C1_SDAT),
               .hps_0_hps_io_hps_io_i2c1_inst_SCL     (HPS_I2C1_SCLK),

               //STM
               .hps_0_f2h_cold_reset_req_reset_n      (~hps_cold_reset),
               .hps_0_f2h_debug_reset_req_reset_n     (~hps_debug_reset),
               .hps_0_f2h_warm_reset_req_reset_n      (~hps_warm_reset),
               .hps_0_f2h_stm_hw_events_stm_hwevents  (stm_hw_events),

               //HDMI
               .clk_hdmi_clk                                      (HDMI_TX_CLK),
               .alt_vip_cl_cvo_hdmi_clocked_video_vid_clk         (HDMI_TX_CLK),
               .alt_vip_cl_cvo_hdmi_clocked_video_vid_data        (HDMI_TX_D),
               .alt_vip_cl_cvo_hdmi_clocked_video_underflow       (),
               .alt_vip_cl_cvo_hdmi_clocked_video_vid_mode_change (),
               .alt_vip_cl_cvo_hdmi_clocked_video_vid_std         (),
               .alt_vip_cl_cvo_hdmi_clocked_video_vid_datavalid   (HDMI_TX_DE),
               .alt_vip_cl_cvo_hdmi_clocked_video_vid_v_sync      (HDMI_TX_VS),
               .alt_vip_cl_cvo_hdmi_clocked_video_vid_h_sync      (HDMI_TX_HS),
               .alt_vip_cl_cvo_hdmi_clocked_video_vid_f           (),
               .alt_vip_cl_cvo_hdmi_clocked_video_vid_h           (),
               .alt_vip_cl_cvo_hdmi_clocked_video_vid_v           (),

               //HDMI I2C
               .hps_0_i2c2_out_data   (hdmi_internal_sda_o_e),
               .hps_0_i2c2_sda        (hdmi_internal_sda_o),
               .hps_0_i2c2_clk_clk    (hdmi_internal_scl_o_e),
               .hps_0_i2c2_scl_in_clk (hdmi_internal_scl_o),

               // HDMI AUDIO
               .clk_audio_clk  (AUD_CTRL_CLK),

               //GPIO
               .hps_0_hps_io_hps_io_gpio_inst_GPIO09  ( HPS_CONV_USB_N ),  //                               .hps_io_gpio_inst_GPIO09
               .hps_0_hps_io_hps_io_gpio_inst_GPIO35  ( HPS_ENET_INT_N ),  //                               .hps_io_gpio_inst_GPIO35
               .hps_0_hps_io_hps_io_gpio_inst_GPIO40  ( HPS_LTC_GPIO   ),  //                               .hps_io_gpio_inst_GPIO40
               .hps_0_hps_io_hps_io_gpio_inst_GPIO53  ( HPS_LED   ),  //                               .hps_io_gpio_inst_GPIO53
               .hps_0_hps_io_hps_io_gpio_inst_GPIO54  ( HPS_KEY   ),  //                               .hps_io_gpio_inst_GPIO54
               .hps_0_hps_io_hps_io_gpio_inst_GPIO61  ( HPS_GSENSOR_INT ),  //                               .hps_io_gpio_inst_GPIO61

               //Arduino
               .hps_0_spim0_txd          (arduino_hps_0_spim0_txd),
               .hps_0_spim0_rxd          (arduino_hps_0_spim0_rxd),
               .hps_0_spim0_ss_in_n      (1'b1),
               .hps_0_spim0_ssi_oe_n     (arduino_hps_0_spim0_ssi_oe_n),
               .hps_0_spim0_ss_0_n       (arduino_hps_0_spim0_ss_0_n),
               .hps_0_spim0_ss_1_n       (),
               .hps_0_spim0_ss_2_n       (),
               .hps_0_spim0_ss_3_n       (),
               .hps_0_spim0_sclk_out_clk (arduino_hps_0_spim0_sclk_out_clk),
               /*
                              .hps_0_uart1_cts          (),
                              .hps_0_uart1_dsr          (),
                              .hps_0_uart1_dcd          (),
                              .hps_0_uart1_ri           (),
                              .hps_0_uart1_dtr          (),
                              .hps_0_uart1_rts          (),
                              .hps_0_uart1_out1_n       (),
                              .hps_0_uart1_out2_n       (),
                              .hps_0_uart1_rxd          (arduino_hps_0_uart1_rxd),
                              .hps_0_uart1_txd          (arduino_hps_0_uart1_txd),
               */					
               .hps_0_i2c3_scl_in_clk    (arduino_internal_scl_o),
               .hps_0_i2c3_clk_clk       (arduino_internal_scl_o_e),
               .hps_0_i2c3_out_data      (arduino_internal_sda_o_e),
               .hps_0_i2c3_sda           (arduino_internal_sda_o),
               //.arduino_gpio_export      (ARDUINO_IO[9:3]),

               // POWER SUPPLY UNIT
               // psu to hps signal "user requested shutdown"
               .psu_button_export        (ARDUINO_IO[8]),
               // hps to psu signal "i am ready please turn power off immediately"
               .psu_poweroff_req_export  (ARDUINO_IO[9]),

               //PIOs
               .button_pio_export        (fpga_debounced_buttons),
               .dipsw_pio_export         (SW),
               .led_pio_export           (fpga_led_internal),

               // hm2reg_io_0_conduit
               .mk_io_hm2_datain         (hm_datao),                 //           .hm2_datain
               .mk_io_hm2_dataout        (hm_datai),                    //           .hm2reg.hm2_dataout
               .mk_io_hm2_address        (hm_address),                  //           .hm2_address
               .mk_io_hm2_write          (hm_write),                    //           .hm2_write
               .mk_io_hm2_read           (hm_read),                     //           .hm2_read
               //  .mk_io_hm2_chipsel        (hm_chipsel),                  //           .hm2_chipsel
               .mk_io_hm2_int_in         (int_sig),                     //           .hm2_int_in

               // high & med clocks for hm2
               .clk_100mhz_out_clk       (hm_clk_med),                  //           .clk_100mhz_out.clk
               .clk_200mhz_out_clk       (hm_clk_high),                 //           .clk_100mhz_out.clk

               // ADC
               .adc_io_convst            (ADC_CONVST),
               .adc_io_sck               (ADC_SCK),
               .adc_io_sdi               (ADC_SDI),
               .adc_io_sdo               (ADC_SDO)
           );


// Debounce logic to clean out glitches within 1ms
debounce debounce_inst (
             .clk                                  (fpga_clk_50),
             .reset_n                              (hps_fpga_reset_n),
             .data_in                              (KEY),
             .data_out                             (fpga_debounced_buttons)
         );
defparam debounce_inst.WIDTH = 2;
defparam debounce_inst.POLARITY = "LOW";
defparam debounce_inst.TIMEOUT = 50000;               // at 50Mhz this is a debounce time of 1ms
defparam debounce_inst.TIMEOUT_WIDTH = 16;            // ceil(log2(TIMEOUT))

// Source/Probe megawizard instance
hps_reset hps_reset_inst (
              .source_clk (fpga_clk_50),
              .source     (hps_reset_req)
          );

altera_edge_detector pulse_cold_reset (
                         .clk       (fpga_clk_50),
                         .rst_n     (hps_fpga_reset_n),
                         .signal_in (hps_reset_req[0]),
                         .pulse_out (hps_cold_reset)
                     );
defparam pulse_cold_reset.PULSE_EXT = 6;
defparam pulse_cold_reset.EDGE_TYPE = 1;
defparam pulse_cold_reset.IGNORE_RST_WHILE_BUSY = 1;

altera_edge_detector pulse_warm_reset (
                         .clk       (fpga_clk_50),
                         .rst_n     (hps_fpga_reset_n),
                         .signal_in (hps_reset_req[1]),
                         .pulse_out (hps_warm_reset)
                     );
defparam pulse_warm_reset.PULSE_EXT = 2;
defparam pulse_warm_reset.EDGE_TYPE = 1;
defparam pulse_warm_reset.IGNORE_RST_WHILE_BUSY = 1;

altera_edge_detector pulse_debug_reset (
                         .clk       (fpga_clk_50),
                         .rst_n     (hps_fpga_reset_n),
                         .signal_in (hps_reset_req[2]),
                         .pulse_out (hps_debug_reset)
                     );
defparam pulse_debug_reset.PULSE_EXT = 32;
defparam pulse_debug_reset.EDGE_TYPE = 1;
defparam pulse_debug_reset.IGNORE_RST_WHILE_BUSY = 1;

led_blinker led_blinker_inst
            (
                .clk(fpga_clk_50) ,     // input  clk_sig
                .reset_n(hps_fpga_reset_n) ,     // input  reset_n_sig
                .LED(LED[0])       // output  LED_sig
            );
defparam led_blinker_inst.COUNT_MAX = 24999999;

// Mesa code ------------------------------------------------------//

assign clklow_sig = FPGA_CLK1_50;
assign clkhigh_sig = hm_clk_high;
assign clkmed_sig = hm_clk_med;

//
HostMot2_cfg HostMot2_inst
             (
                 .ibus(hm_datai) ,       // input [buswidth-1:0] ibus_sig
                 .obus(hm_datao) ,       // output [buswidth-1:0] obus_sig
                 .addr(hm_address) ,    // input [addrwidth-1:2] addr_sig -- addr => A(AddrWidth-1 downto 2),
                 .readstb(hm_read ) ,    // input  readstb_sig
                 .writestb(hm_write) ,    // input  writestb_sig

                 .clklow(clklow_sig) ,    // input  clklow_sig      -- PCI clock --> all
                 .clkmed(clkmed_sig) ,    // input  clkmed_sig      -- Processor clock --> sserialwa, twiddle
                 .clkhigh(clkhigh_sig) , // input  clkhigh_sig    -- High speed clock --> most
                 .irq(int_sig) ,          // output  int_sig       --int => LINT, ---> PCI ?

                 .iobits({
                             // GPIO_1           // DB25-P3
                             GPIO_1[18],  // PIN 13
                             GPIO_1[21],  // PIN 12
                             GPIO_1[20],  // PIN 11
                             GPIO_1[23],  // PIN 10
                             GPIO_1[22],  // PIN 9
                             GPIO_1[25],  // PIN 8
                             GPIO_1[24],  // PIN 7
                             GPIO_1[27],  // PIN 6
                             GPIO_1[26],  // PIN 5
                             GPIO_1[29],  // PIN 17
                             GPIO_1[28],  // PIN 4
                             GPIO_1[31],  // PIN 16
                             GPIO_1[30],  // PIN 3
                             GPIO_1[33],  // PIN 15
                             GPIO_1[32],  // PIN 2
                             GPIO_1[35],  // PIN 14
                             GPIO_1[34],  // PIN 1

                             // GPIO_1           // DB25-P2
                             GPIO_1[00],  // PIN 13
                             GPIO_1[03],  // PIN 12
                             GPIO_1[02],  // PIN 11
                             GPIO_1[05],  // PIN 10
                             GPIO_1[04],  // PIN 9
                             GPIO_1[07],  // PIN 8
                             GPIO_1[06],  // PIN 7
                             GPIO_1[09],  // PIN 6
                             GPIO_1[08],  // PIN 5
                             GPIO_1[11],  // PIN 17
                             GPIO_1[10],  // PIN 4
                             GPIO_1[13],  // PIN 16
                             GPIO_1[12],  // PIN 3
                             GPIO_1[15],  // PIN 15
                             GPIO_1[14],  // PIN 2
                             GPIO_1[17],  // PIN 14
                             GPIO_1[16],  // PIN 1

                             // GPIO_0           // DB25-P1
                             GPIO_0[18],  // PIN 13
                             GPIO_0[21],  // PIN 12
                             GPIO_0[20],  // PIN 11
                             GPIO_0[23],  // PIN 10
                             GPIO_0[22],  // PIN 9
                             GPIO_0[25],  // PIN 8
                             GPIO_0[24],  // PIN 7
                             GPIO_0[27],  // PIN 6
                             GPIO_0[26],  // PIN 5
                             GPIO_0[29],  // PIN 17
                             GPIO_0[28],  // PIN 4
                             GPIO_0[31],  // PIN 16
                             GPIO_0[30],  // PIN 3
                             GPIO_0[33],  // PIN 15
                             GPIO_0[32],  // PIN 2
                             GPIO_0[35],  // PIN 14
                             GPIO_0[34],  // PIN 1

                             // GPIO_0           // DB25-P0
                             GPIO_0[00],  // PIN 13
                             GPIO_0[03],  // PIN 12
                             GPIO_0[02],  // PIN 11
                             GPIO_0[05],  // PIN 10
                             GPIO_0[04],  // PIN 9
                             GPIO_0[07],  // PIN 8
                             GPIO_0[06],  // PIN 7
                             GPIO_0[09],  // PIN 6
                             GPIO_0[08],  // PIN 5
                             GPIO_0[11],  // PIN 17
                             GPIO_0[10],  // PIN 4
                             GPIO_0[13],  // PIN 16
                             GPIO_0[12],  // PIN 3
                             GPIO_0[15],  // PIN 15
                             GPIO_0[14],  // PIN 2
                             GPIO_0[17],  // PIN 14
                             GPIO_0[16]   // PIN 1
                         }),

                 .leds({
                           GPIO_1[19],
                           GPIO_1[1],
                           GPIO_0[19],
                           GPIO_0[1]
                       })
             );

endmodule
