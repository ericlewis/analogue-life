//
// User core top-level
//
// Instantiated by the real top-level: apf_top
//

`default_nettype none

module core_top (

//
// physical connections
//

///////////////////////////////////////////////////
// clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

input   wire            clk_74a, // mainclk1
input   wire            clk_74b, // mainclk1 

///////////////////////////////////////////////////
// cartridge interface
// switches between 3.3v and 5v mechanically
// output enable for multibit translators controlled by pic32

// GBA AD[15:8]
inout   wire    [7:0]   cart_tran_bank2,
output  wire            cart_tran_bank2_dir,

// GBA AD[7:0]
inout   wire    [7:0]   cart_tran_bank3,
output  wire            cart_tran_bank3_dir,

// GBA A[23:16]
inout   wire    [7:0]   cart_tran_bank1,
output  wire            cart_tran_bank1_dir,

// GBA [7] PHI#
// GBA [6] WR#
// GBA [5] RD#
// GBA [4] CS1#/CS#
//     [3:0] unwired
inout   wire    [7:4]   cart_tran_bank0,
output  wire            cart_tran_bank0_dir,

// GBA CS2#/RES#
inout   wire            cart_tran_pin30,
output  wire            cart_tran_pin30_dir,
// when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
// the goal is that when unconfigured, the FPGA weak pullups won't interfere.
// thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
// and general IO drive this pin.
output  wire            cart_pin30_pwroff_reset,

// GBA IRQ/DRQ
inout   wire            cart_tran_pin31,
output  wire            cart_tran_pin31_dir,

// infrared
input   wire            port_ir_rx,
output  wire            port_ir_tx,
output  wire            port_ir_rx_disable, 

// GBA link port
inout   wire            port_tran_si,
output  wire            port_tran_si_dir,
inout   wire            port_tran_so,
output  wire            port_tran_so_dir,
inout   wire            port_tran_sck,
output  wire            port_tran_sck_dir,
inout   wire            port_tran_sd,
output  wire            port_tran_sd_dir,
 
///////////////////////////////////////////////////
// cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

output  wire    [21:16] cram0_a,
inout   wire    [15:0]  cram0_dq,
input   wire            cram0_wait,
output  wire            cram0_clk,
output  wire            cram0_adv_n,
output  wire            cram0_cre,
output  wire            cram0_ce0_n,
output  wire            cram0_ce1_n,
output  wire            cram0_oe_n,
output  wire            cram0_we_n,
output  wire            cram0_ub_n,
output  wire            cram0_lb_n,

output  wire    [21:16] cram1_a,
inout   wire    [15:0]  cram1_dq,
input   wire            cram1_wait,
output  wire            cram1_clk,
output  wire            cram1_adv_n,
output  wire            cram1_cre,
output  wire            cram1_ce0_n,
output  wire            cram1_ce1_n,
output  wire            cram1_oe_n,
output  wire            cram1_we_n,
output  wire            cram1_ub_n,
output  wire            cram1_lb_n,

///////////////////////////////////////////////////
// sdram, 512mbit 16bit

output  wire    [12:0]  dram_a,
output  wire    [1:0]   dram_ba,
inout   wire    [15:0]  dram_dq,
output  wire    [1:0]   dram_dqm,
output  wire            dram_clk,
output  wire            dram_cke,
output  wire            dram_ras_n,
output  wire            dram_cas_n,
output  wire            dram_we_n,

///////////////////////////////////////////////////
// sram, 1mbit 16bit

output  wire    [16:0]  sram_a,
inout   wire    [15:0]  sram_dq,
output  wire            sram_oe_n,
output  wire            sram_we_n,
output  wire            sram_ub_n,
output  wire            sram_lb_n,

///////////////////////////////////////////////////
// vblank driven by dock for sync in a certain mode

input   wire            vblank,

///////////////////////////////////////////////////
// i/o to 6515D breakout usb uart

output  wire            dbg_tx,
input   wire            dbg_rx,

///////////////////////////////////////////////////
// i/o pads near jtag connector user can solder to

output  wire            user1,
input   wire            user2,

///////////////////////////////////////////////////
// RFU internal i2c bus 

inout   wire            aux_sda,
output  wire            aux_scl,

///////////////////////////////////////////////////
// RFU, do not use
output  wire            vpll_feed,


//
// logical connections
//

///////////////////////////////////////////////////
// video, audio output to scaler
output  wire    [23:0]  video_rgb,
output  wire            video_rgb_clock,
output  wire            video_rgb_clock_90,
output  wire            video_de,
output  wire            video_skip,
output  wire            video_vs,
output  wire            video_hs,
    
output  wire            audio_mclk,
input   wire            audio_adc,
output  wire            audio_dac,
output  wire            audio_lrck,

///////////////////////////////////////////////////
// bridge bus connection
// synchronous to clk_74a
output  wire            bridge_endian_little,
input   wire    [31:0]  bridge_addr,
input   wire            bridge_rd,
output  reg     [31:0]  bridge_rd_data,
input   wire            bridge_wr,
input   wire    [31:0]  bridge_wr_data,

///////////////////////////////////////////////////
// controller data
// 
// key bitmap:
//   [0]    dpad_up
//   [1]    dpad_down
//   [2]    dpad_left
//   [3]    dpad_right
//   [4]    face_a
//   [5]    face_b
//   [6]    face_x
//   [7]    face_y
//   [8]    trig_l1
//   [9]    trig_r1
//   [10]   trig_l2
//   [11]   trig_r2
//   [12]   trig_l3
//   [13]   trig_r3
//   [14]   face_select
//   [15]   face_start
// joy values - unsigned
//   [ 7: 0] lstick_x
//   [15: 8] lstick_y
//   [23:16] rstick_x
//   [31:24] rstick_y
// trigger values - unsigned
//   [ 7: 0] ltrig
//   [15: 8] rtrig
//
input   wire    [15:0]  cont1_key,
input   wire    [15:0]  cont2_key,
input   wire    [15:0]  cont3_key,
input   wire    [15:0]  cont4_key,
input   wire    [31:0]  cont1_joy,
input   wire    [31:0]  cont2_joy,
input   wire    [31:0]  cont3_joy,
input   wire    [31:0]  cont4_joy,
input   wire    [15:0]  cont1_trig,
input   wire    [15:0]  cont2_trig,
input   wire    [15:0]  cont3_trig,
input   wire    [15:0]  cont4_trig
    
);

// not using the IR port, so turn off both the LED, and
// disable the receive circuit to save power
assign port_ir_tx = 0;
assign port_ir_rx_disable = 1;

// bridge endianness
assign bridge_endian_little = 0;

// cart is unused, so set all level translators accordingly
// directions are 0:IN, 1:OUT
assign cart_tran_bank3 = 8'hzz;
assign cart_tran_bank3_dir = 1'b0;
assign cart_tran_bank2 = 8'hzz;
assign cart_tran_bank2_dir = 1'b0;
assign cart_tran_bank1 = 8'hzz;
assign cart_tran_bank1_dir = 1'b0;
assign cart_tran_bank0 = 4'hf;
assign cart_tran_bank0_dir = 1'b1;
assign cart_tran_pin30 = 1'b0;      // reset or cs2, we let the hw control it by itself
assign cart_tran_pin30_dir = 1'bz;
assign cart_pin30_pwroff_reset = 1'b0;  // hardware can control this
assign cart_tran_pin31 = 1'bz;      // input
assign cart_tran_pin31_dir = 1'b0;  // input

// link port is input only
assign port_tran_so = 1'bz;
assign port_tran_so_dir = 1'b0;     // SO is output only
assign port_tran_si = 1'bz;
assign port_tran_si_dir = 1'b0;     // SI is input only
assign port_tran_sck = 1'bz;
assign port_tran_sck_dir = 1'b0;    // clock direction can change
assign port_tran_sd = 1'bz;
assign port_tran_sd_dir = 1'b0;     // SD is input and not used

// tie off the rest of the pins we are not using
assign cram0_a = 'h0;
assign cram0_dq = {16{1'bZ}};
assign cram0_clk = 0;
assign cram0_adv_n = 1;
assign cram0_cre = 0;
assign cram0_ce0_n = 1;
assign cram0_ce1_n = 1;
assign cram0_oe_n = 1;
assign cram0_we_n = 1;
assign cram0_ub_n = 1;
assign cram0_lb_n = 1;

assign cram1_a = 'h0;
assign cram1_dq = {16{1'bZ}};
assign cram1_clk = 0;
assign cram1_adv_n = 1;
assign cram1_cre = 0;
assign cram1_ce0_n = 1;
assign cram1_ce1_n = 1;
assign cram1_oe_n = 1;
assign cram1_we_n = 1;
assign cram1_ub_n = 1;
assign cram1_lb_n = 1;

assign dram_a = 'h0;
assign dram_ba = 'h0;
assign dram_dq = {16{1'bZ}};
assign dram_dqm = 'h0;
assign dram_clk = 'h0;
assign dram_cke = 'h0;
assign dram_ras_n = 'h1;
assign dram_cas_n = 'h1;
assign dram_we_n = 'h1;

assign sram_a = 'h0;
assign sram_dq = {16{1'bZ}};
assign sram_oe_n  = 1;
assign sram_we_n  = 1;
assign sram_ub_n  = 1;
assign sram_lb_n  = 1;

assign dbg_tx = 1'bZ;
assign user1 = 1'bZ;
assign aux_scl = 1'bZ;
assign vpll_feed = 1'bZ;


// for bridge write data, we just broadcast it to all bus devices
// for bridge read data, we have to mux it
// add your own devices here
always @(*) begin
    casex(bridge_addr)
        32'h00100000: begin
		    bridge_rd_data <= gen_speed;
	    end
        32'hF8xxxxxx: begin
            bridge_rd_data <= cmd_bridge_rd_data;
        end
        default: begin
		    bridge_rd_data <= 0;
	    end
    endcase
end

always @(posedge clk_74a) begin
	if(bridge_wr) begin
        casex(bridge_addr)
            32'h00100000: begin
			    gen_speed <= bridge_wr_data;
		    end
        endcase
    end
end

//
// host/target command handler
//
    wire            reset_n;                // driven by host commands, can be used as core-wide reset
    wire    [31:0]  cmd_bridge_rd_data;
    
// bridge host commands
// synchronous to clk_74a
    wire            status_boot_done = pll_core_locked; 
    wire            status_setup_done = pll_core_locked; // rising edge triggers a target command
    wire            status_running = reset_n; // we are running as soon as reset_n goes high

    wire            dataslot_requestread;
    wire    [15:0]  dataslot_requestread_id;
    wire            dataslot_requestread_ack = 1;
    wire            dataslot_requestread_ok = 1;

    wire            dataslot_requestwrite;
    wire    [15:0]  dataslot_requestwrite_id;
    wire            dataslot_requestwrite_ack = 1;
    wire            dataslot_requestwrite_ok = 1;

    wire            dataslot_allcomplete;

    wire            savestate_supported;
    wire    [31:0]  savestate_addr;
    wire    [31:0]  savestate_size;
    wire    [31:0]  savestate_maxloadsize;

    wire            savestate_start;
    wire            savestate_start_ack;
    wire            savestate_start_busy;
    wire            savestate_start_ok;
    wire            savestate_start_err;

    wire            savestate_load;
    wire            savestate_load_ack;
    wire            savestate_load_busy;
    wire            savestate_load_ok;
    wire            savestate_load_err;
    
    wire            osnotify_inmenu;

// bridge target commands
// synchronous to clk_74a


// bridge data slot access

    wire    [9:0]   datatable_addr;
    wire            datatable_wren;
    wire    [31:0]  datatable_data;
    wire    [31:0]  datatable_q;

core_bridge_cmd icb (

    .clk                ( clk_74a ),
    .reset_n            ( reset_n ),

    .bridge_endian_little   ( bridge_endian_little ),
    .bridge_addr            ( bridge_addr ),
    .bridge_rd              ( bridge_rd ),
    .bridge_rd_data         ( cmd_bridge_rd_data ),
    .bridge_wr              ( bridge_wr ),
    .bridge_wr_data         ( bridge_wr_data ),
    
    .status_boot_done       ( status_boot_done ),
    .status_setup_done      ( status_setup_done ),
    .status_running         ( status_running ),

    .dataslot_requestread       ( dataslot_requestread ),
    .dataslot_requestread_id    ( dataslot_requestread_id ),
    .dataslot_requestread_ack   ( dataslot_requestread_ack ),
    .dataslot_requestread_ok    ( dataslot_requestread_ok ),

    .dataslot_requestwrite      ( dataslot_requestwrite ),
    .dataslot_requestwrite_id   ( dataslot_requestwrite_id ),
    .dataslot_requestwrite_ack  ( dataslot_requestwrite_ack ),
    .dataslot_requestwrite_ok   ( dataslot_requestwrite_ok ),

    .dataslot_allcomplete   ( dataslot_allcomplete ),

    .savestate_supported    ( savestate_supported ),
    .savestate_addr         ( savestate_addr ),
    .savestate_size         ( savestate_size ),
    .savestate_maxloadsize  ( savestate_maxloadsize ),

    .savestate_start        ( savestate_start ),
    .savestate_start_ack    ( savestate_start_ack ),
    .savestate_start_busy   ( savestate_start_busy ),
    .savestate_start_ok     ( savestate_start_ok ),
    .savestate_start_err    ( savestate_start_err ),

    .savestate_load         ( savestate_load ),
    .savestate_load_ack     ( savestate_load_ack ),
    .savestate_load_busy    ( savestate_load_busy ),
    .savestate_load_ok      ( savestate_load_ok ),
    .savestate_load_err     ( savestate_load_err ),

    .osnotify_inmenu        ( osnotify_inmenu ),
    
    .datatable_addr         ( datatable_addr ),
    .datatable_wren         ( datatable_wren ),
    .datatable_data         ( datatable_data ),
    .datatable_q            ( datatable_q ),

);



////////////////////////////////////////////////////////////////////////////////////////

reg  [31:0]	gen_speed = 1;
reg reset_button = 0;
reg prev_button_15 = 0;

always @(posedge clk_74a) begin
    reset_button <= 1;
    if (!prev_button_15 && cont1_key[15])
      reset_button <= 0;
    prev_button_15 = cont1_key[15];
end

life life_instance(
    .clock_74(clk_74a),
    .reset_n(coin_insert),
    .x(visible_x),
    .y(visible_y),
    .r(video_rgb[23:16]),
    .g(video_rgb[15:8]),
    .b(video_rgb[7:0]),
    .gen_speed(osnotify_inmenu ? 0 : gen_speed),
);

// video generation

assign video_rgb_clock = clk_core_23_75;
assign video_rgb_clock_90 = clk_core_23_75_90deg;
assign video_de = vidout_de;
assign video_skip = vidout_skip;
assign video_vs = vidout_vs;
assign video_hs = vidout_hs;

    localparam  VID_V_BPORCH = 'd20;
    localparam  VID_V_ACTIVE = 'd480;
    localparam  VID_V_TOTAL = 'd500;
    localparam  VID_H_BPORCH = 'd160;
    localparam  VID_H_ACTIVE = 'd640;
    localparam  VID_H_TOTAL = 'd800;

    reg [15:0]  frame_count;
    
    reg [9:0]   x_count;
    reg [9:0]   y_count;
    
    wire [9:0]  visible_x = x_count - VID_H_BPORCH;
    wire [9:0]  visible_y = y_count - VID_V_BPORCH;

    reg [23:0]  vidout_rgb;
    reg         vidout_de, vidout_de_1;
    reg         vidout_skip;
    reg         vidout_vs;
    reg         vidout_hs, vidout_hs_1;
    
always @(posedge clk_core_23_75 or negedge reset_n) begin

    if(~reset_n) begin
    
        x_count <= 0;
        y_count <= 0;
        
    end else begin
        vidout_de <= 0;
        vidout_skip <= 0;
        vidout_vs <= 0;
        vidout_hs <= 0;
        
        vidout_hs_1 <= vidout_hs;
        vidout_de_1 <= vidout_de;
        
        // x and y counters
        x_count <= x_count + 1'b1;
        if(x_count == VID_H_TOTAL-1) begin
            x_count <= 0;
            
            y_count <= y_count + 1'b1;
            if(y_count == VID_V_TOTAL-1) begin
                y_count <= 0;
            end
        end
        
        // generate sync 
        if(x_count == 0 && y_count == 0) begin
            // sync signal in back porch
            // new frame
            vidout_vs <= 1;
            frame_count <= frame_count + 1'b1;
        end
        
        // we want HS to occur a bit after VS, not on the same cycle
        if(x_count == 3) begin
            // sync signal in back porch
            // new line
            vidout_hs <= 1;
        end

        // inactive screen areas are black
        vidout_rgb <= 24'h0;
        // generate active video
        if(x_count >= VID_H_BPORCH && x_count < VID_H_ACTIVE+VID_H_BPORCH) begin

            if(y_count >= VID_V_BPORCH && y_count < VID_V_ACTIVE+VID_V_BPORCH) begin
                // data enable. this is the active region of the line
                vidout_de <= 1;
            end 
        end
    end
end

//
// audio i2s silence generator
// see other examples for actual audio generation
//

assign audio_mclk = audgen_mclk;
assign audio_dac = audgen_dac;
assign audio_lrck = audgen_lrck;

// generate MCLK = 12.288mhz with fractional accumulator
    reg         [21:0]  audgen_accum;
    reg                 audgen_mclk;
    parameter   [20:0]  CYCLE_48KHZ = 21'd122880 * 2;
always @(posedge clk_74a) begin
    audgen_accum <= audgen_accum + CYCLE_48KHZ;
    if(audgen_accum >= 21'd742500) begin
        audgen_mclk <= ~audgen_mclk;
        audgen_accum <= audgen_accum - 21'd742500 + CYCLE_48KHZ;
    end
end

// generate SCLK = 3.072mhz by dividing MCLK by 4
    reg [1:0]   aud_mclk_divider;
    wire        audgen_sclk = aud_mclk_divider[1] /* synthesis keep*/;
    reg         audgen_lrck_1;
always @(posedge audgen_mclk) begin
    aud_mclk_divider <= aud_mclk_divider + 1'b1;
end

// shift out audio data as I2S 
// 32 total bits per channel, but only 16 active bits at the start and then 16 dummy bits
//
    reg     [4:0]   audgen_lrck_cnt;    
    reg             audgen_lrck;
    reg             audgen_dac;
always @(negedge audgen_sclk) begin
    audgen_dac <= 1'b0;
    // 48khz * 64
    audgen_lrck_cnt <= audgen_lrck_cnt + 1'b1;
    if(audgen_lrck_cnt == 31) begin
        // switch channels
        audgen_lrck <= ~audgen_lrck;
        
    end 
end


///////////////////////////////////////////////


    wire    clk_core_23_75;
    wire    clk_core_23_75_90deg;
    
    wire    pll_core_locked;
    
mf_pllbase mp1 (
    .refclk         ( clk_74a ),
    .rst            ( 0 ),
    
    .outclk_0       ( clk_core_23_75 ),
    .outclk_1       ( clk_core_23_75_90deg ),
    
    .locked         ( pll_core_locked )
);

endmodule

module life(clock_74, reset_n, x, y, r, g, b, gen_speed);
    input clock_74;
    input reset_n;
    input  [9:0] x, y;
    output [9:0] r, g, b;
    input  [31:0] gen_speed;

    wire [0:64*48-1] cells;
    wire [0:64*48-1] cells_reset_state;
    reg [0:64*48-1] cells_preset;

    reg [3:0] preset_state;

    initial begin
        cells_preset[0    :64*0+63 ] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*1 :64*1+63 ] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*2 :64*2+63 ] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*3 :64*3+63 ] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*4 :64*4+63 ] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*5 :64*5+63 ] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*6 :64*6+63 ] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*7 :64*7+63 ] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*8 :64*8+63 ] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*9 :64*9+63 ] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*10:64*10+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*11:64*11+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*12:64*12+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*13:64*13+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*14:64*14+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*15:64*15+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*16:64*16+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*17:64*17+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*18:64*18+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*19:64*19+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*20:64*20+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*21:64*21+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*22:64*22+63] <= 64'b0000000000000000000000000000001100100000000000000000000000000000;
        cells_preset[64*23:64*23+63] <= 64'b0000000000000000000000000000001000100000000000000000000000000000;
        cells_preset[64*24:64*24+63] <= 64'b0000000000000000000000000000001001100000000000000000000000000000;
        cells_preset[64*25:64*25+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*26:64*26+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*27:64*27+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*28:64*28+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*29:64*29+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*30:64*30+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*31:64*31+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*32:64*32+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*33:64*33+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*34:64*34+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*35:64*35+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*36:64*36+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*37:64*37+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*38:64*38+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*39:64*39+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*40:64*40+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*41:64*41+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*42:64*42+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*43:64*43+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*44:64*44+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*45:64*45+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*46:64*46+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;
        cells_preset[64*47:64*47+63] <= 64'b0000000000000000000000000000000000000000000000000000000000000000;

        preset_state <= 2;
    end

    assign cells_reset_state = cells_preset;

    wire clk;
    clock(clk, clock_74, gen_speed);

    genvar i;
    generate
        for (i = 0; i < 64 * 48; i = i + 1) begin : CELLS
            wire [7:0] neighbours;
            if (i == 0) begin
                assign neighbours[0] = cells[64*48 - 1];
                assign neighbours[1] = cells[64*48 - 64];
                assign neighbours[2] = cells[64*48 - 64 + 1];
                assign neighbours[3] = cells[i     + 64 - 1];
                assign neighbours[4] = cells[i     + 1];
                assign neighbours[5] = cells[64    + 64 - 1];
                assign neighbours[6] = cells[64];
                assign neighbours[7] = cells[64    + 1];
            end else if (i == 63) begin
                assign neighbours[0] = cells[64*48 - 1 - 1];
                assign neighbours[1] = cells[64*48 - 1];
                assign neighbours[2] = cells[64*48 - 64];
                assign neighbours[3] = cells[i     - 1];
                assign neighbours[4] = cells[0];
                assign neighbours[5] = cells[i     + 64 - 1];
                assign neighbours[6] = cells[i     + 64];
                assign neighbours[7] = cells[i     + 1];
            end else if (i == 64*48 - 64) begin
                assign neighbours[0] = cells[i - 1];
                assign neighbours[1] = cells[i - 64];
                assign neighbours[2] = cells[i - 64 + 1];
                assign neighbours[3] = cells[i + 64 - 1];
                assign neighbours[4] = cells[i + 1];
                assign neighbours[5] = cells[0 + 64 - 1];
                assign neighbours[6] = cells[0];
                assign neighbours[7] = cells[0 + 1];
            end else if (i == 64*48 - 1) begin
                assign neighbours[0] = cells[i - 64 - 1];
                assign neighbours[1] = cells[i - 64];
                assign neighbours[2] = cells[i - 64 - 64 + 1];
                assign neighbours[3] = cells[i - 1];
                assign neighbours[4] = cells[i - 64 + 1];
                assign neighbours[5] = cells[0 + 64 - 1 - 1];
                assign neighbours[6] = cells[0 + 64 - 1];
                assign neighbours[7] = cells[0];
            end else if (i < 63) begin
                assign neighbours[0] = cells[64*48 - 64 + i - 1];
                assign neighbours[1] = cells[64*48 - 64 + i];
                assign neighbours[2] = cells[64*48 - 64 + i + 1];
                assign neighbours[3] = cells[i - 1];
                assign neighbours[4] = cells[i + 1];
                assign neighbours[5] = cells[i + 64 - 1];
                assign neighbours[6] = cells[i + 64];
                assign neighbours[7] = cells[i + 64 + 1];
            end else if (i > 64*48 - 64) begin
                assign neighbours[0] = cells[i - 64 - 1];
                assign neighbours[1] = cells[i - 64];
                assign neighbours[2] = cells[i - 64 + 1];
                assign neighbours[3] = cells[i - 1];
                assign neighbours[4] = cells[i + 1];
                assign neighbours[5] = cells[0 + i - 1];
                assign neighbours[6] = cells[0 + i];
                assign neighbours[7] = cells[0 + i + 1];
            end else if (i % 64 == 0) begin
                assign neighbours[0] = cells[i - 1];
                assign neighbours[1] = cells[i - 64];
                assign neighbours[2] = cells[i - 64 + 1];
                assign neighbours[3] = cells[i + 64 - 1];
                assign neighbours[4] = cells[i + 1];
                assign neighbours[5] = cells[i + 64 + 64 - 1];
                assign neighbours[6] = cells[i + 64];
                assign neighbours[7] = cells[i + 64 + 1];
            end else if ((i + 1) % 64 == 0) begin
                assign neighbours[0] = cells[i - 64 - 1];
                assign neighbours[1] = cells[i - 64];
                assign neighbours[2] = cells[i - 64 - 64 + 1];
                assign neighbours[3] = cells[i - 1];
                assign neighbours[4] = cells[i - 64 + 1];
                assign neighbours[5] = cells[i + 64 - 1];
                assign neighbours[6] = cells[i + 64];
                assign neighbours[7] = cells[i + 1];
            end else begin
                assign neighbours[0] = cells[i - 64 - 1];
                assign neighbours[1] = cells[i - 64];
                assign neighbours[2] = cells[i - 64 + 1];
                assign neighbours[3] = cells[i - 1];
                assign neighbours[4] = cells[i + 1];
                assign neighbours[5] = cells[i + 64 - 1];
                assign neighbours[6] = cells[i + 64];
                assign neighbours[7] = cells[i + 64 + 1];
            end

            life_cell(neighbours, clk, reset_n, cells_reset_state[i], cells[i]);

        end
    endgenerate

    draw(cells, x, y, r, g, b);

endmodule

module clock(clk, clock_74, gen_speed);
        input clock_74;
        output clk;
        input [31:0] gen_speed;
        reg   [31:0] counter;
        
        always @(posedge clock_74) begin
            if (counter >= 25000000)
                counter <= 0;
            else
                counter <= counter + gen_speed;
        end
        assign clk = (counter >= 25000000);
endmodule

module life_cell(neighbours, clk, reset_n, initial_state, state);
    input [7:0] neighbours;
    input clk;
    input reset_n;
    input initial_state;
    output reg state;
 
    wire [3:0] population;
    wire next_state;

    assign population = neighbours[7] + 
                        neighbours[6] +
                        neighbours[5] +
                        neighbours[4] +
                        neighbours[3] +
                        neighbours[2] +
                        neighbours[1] +
                        neighbours[0];
    
    assign next_state = (population == 2 & state) | population == 3;
 
    always @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            state = initial_state;
        end else begin
            state = next_state;
        end
    end
endmodule

module draw(cells, x, y, r, g, b);
    input [0:64*48-1] cells;
    input [9:0] x, y;
    output [9:0] r, g, b;

    reg [29:0] RGB;
    always @(x or y) begin
        if (cells[(y / 10) * 64 + (x / 10)]) begin
            RGB = 30'b111111111111111111111111111111;
        end else begin
            RGB = 30'b000000000000000000000000000000;
        end
    end

    assign r = RGB[29:20];
    assign g = RGB[19:10];
    assign b = RGB[9:0];

endmodule