`timescale 1ns / 1ps

module PPU
(
    input                   clk     ,
                            rst     ,
                            PPU_Enabled,
    output reg [15:0]       OAM_Addr,
                            VRAM_Addr,
                            MMIO_Addr,
    output reg [7:0]        OAM_Dout,
                            VRAM_Dout,
                            MMIO_Dout,
    input [7:0]             OAM_Din ,     // OAM
                            VRAM_Din,
                            MMIO_Din,
    output reg              OAM_Read,    // OAM
                            //OAM_Write,   // OAM
                            VRAM_Read,
                            MMIO_Read,
                            MMIO_Write,
                            Drawing, // When HIGH, Load TBPP output into FIFO
                            //VRAM_Write,
    output reg [ 1 : 0 ]    TBPP, //Two bit per pixel output
    output     [ 3 : 0 ]    State_Out
);
    //--------------------------- LOCALPARAM -----------------------------
    localparam  TILE_8_MIN  = 16'h8000, // min address 8000 method (unsigned)
                TILE_88_MIN = 16'h9000, // min address 8800 method (signed) 
                TILE_MAX    = 16'h97FF, // max address for tile data
                BG_MIN      = 16'h9800, // lowest address background
                BG_MAX      = 16'h9BFF, // max address background 
                WIN_MIN     = 16'h9C00, // lowest address window data
                WIN_MAX     = 16'h9FFF, // max address window data
                OAM_MIN     = 16'hFE00, // lowest address object attribute memory 
                OAM_MAX     = 16'hFE9F; // max address object attribute memory

    localparam  X_MAX = 160,
                Y_MAX = 144;
                
    localparam  BG_WIN_MAX = 32;
    
    localparam  PIXEL_MAX = 160;
    
    localparam  SCANLINE_MAX = 2 * PIXEL_MAX;
    
    localparam  VGA_SCANLINE_MAX = 640;

    localparam  NORMAL_H = 8, // normal sprite height
                TALL_H = 16; // tall sprite height

    localparam  OAM_ENTRY_MAX = 40, // maximum number of OAM entries
                OAM_TILE_MAX = 160; // maximum number of bytes stored in the oam buffer
    
    
    localparam  H_BLANK         = 4'b0000,
                V_BLANK         = 4'b0001,
                OAM_SCAN_Y      = 4'b0010,
                OAM_SCAN_X      = 4'b0011,
                OAM_SCAN_NUM    = 4'b0100,
                OAM_SCAN_FLAGS  = 4'b0101,
                OAM_SCAN_BUF_1  = 4'b0110,
                OAM_SCAN_BUF_2  = 4'b0111,
                DRAW_TILE       = 4'b1000, 
                DRAW_BG_1       = 4'b1001,
                DRAW_BG_2       = 4'b1010,
                DRAW_WIN_1      = 4'b1011,
                DRAW_WIN_2      = 4'b1100,
                DRAW_OUTPUT     = 4'b1101,
                INACTIVE        = 4'b1110;
    
    //-------------------------- Wires & Regs -----------------------------

    reg [ 7 : 0 ] X_Pixel_Async,
                  Y_Pixel_Async;

    reg [ 7 : 0 ] Register_LCDC,      // $FF40 LCD Control (R/W)
                  Register_Status,    // $FF41 LCDC Status (R/W)
                  Register_Scroll_Y,  // $FF42 Scroll Y (R/W)
                  Register_Scroll_X,  // $FF43 Scroll X (R/W)
                  Register_LY,        // $FF44 LCDC Y-Coordinate (R) Write reset the counter
                  Register_DMA,       // $FF46 DMA, actually handled outside of PPU for now
                  Register_Coord_Yc,  // $FF45 LY Compare (R/W)
                  Register_Bkgd_Plt,  // $FF47 BG Palette Data (R/W) Non-CGB mode only
                  Register_Obj_Plt_0, // $FF48 Object Palette 0 Data (R/W) Non-CGB mode only
                  Register_Obj_Plt_1, // $FF49 Object Palette 1 Data (R/W) Non-CGB mode only
                  Register_Win_Pos_Y, // $FF4A Window Y Position (R/W)
                  Register_Win_Pos_X; // $FF4B Window X Position (R/W)

    // FSM control flags
    reg         Frame_Done,
                Buffering,
                bg_win_fetch,
    //          Drawing,
    //            PPU_Enabled,
                OAM_Fetch_Done;
                //State_Done;

    reg [3:0]   State,
                Next_State;

    reg [3:0]   oam_buf_count; // number of sprites on the buffer

    reg [7:0]   oam_buf_i;  // oam buffer index
    
    reg [7:0]   temp;
    
    reg [5:0]   oam_scan_i; // oam scan index
                
    reg [15:0]  oam_buf_of,
                vram_buf_of; // oam buffer offset
    
    reg         oam_to_display  [9:0];

    reg [7:0]   oam_y_temp      [OAM_ENTRY_MAX - 1:0],
                oam_x_temp      [OAM_ENTRY_MAX - 1:0],
                oam_num_temp    [OAM_ENTRY_MAX - 1:0],
                oam_flags_temp  [OAM_ENTRY_MAX - 1:0];

    reg [7:0]   oam_buffer      [OAM_TILE_MAX - 1:0];
    reg [7:0]   bg_buffer       [BG_WIN_MAX - 1:0];
    reg [7:0]   win_buffer      [BG_WIN_MAX - 1:0];
    
    reg [7:0]   scanline_buffer [19:0];
    
    reg         scanline        [SCANLINE_MAX - 1:0];
    
    wire           Pixel_Tick;
    
    wire [ 7 : 0 ] OAM_Y,
                   OAM_X,
                   OAM_Num,
                   OAM_Flags;

    wire [5:0]   sprite_height;
    
    wire LCDC_Enable        = Register_LCDC[ 7 ];  // 0=Off, 1=On
    wire LCDC_Win_Disp_Sel  = Register_LCDC[ 6 ];  // 0=9800-9BFF, 1=9C00-9FFF
    wire LCDC_Win_Enable    = Register_LCDC[ 5 ];  // 0=Off, 1=On
    wire LCDC_Bkgd_Win_Sel  = Register_LCDC[ 4 ];  // 0=8800-97FF, 1=8000-8FFF
    wire LCDC_Bkgd_Disp_Sel = Register_LCDC[ 3 ];  // 0=9800-9BFF, 1=9C00-9FFF
    wire LCDC_Obj_Size      = Register_LCDC[ 2 ];  // 0=8x8, 1=8x16
    wire LCDC_Obj_Enable    = Register_LCDC[ 1 ];  // 0=Off, 1=On
    wire LCDC_Bkgd_Disp     = Register_LCDC[ 0 ];  // 0=Off, 1=On
    //reg [15:0]  oam[(10 >> 1) - 1:0];
    
    //reg [15:0]  oam_addr;

    integer     i, j;//temp;
    

    //----------------------------- ASSIGN -------------------------------- 
    
    assign State_Out = State;
    assign sprite_height = LCDC_Obj_Size ? TALL_H : NORMAL_H;
    //assign OAM_Dout = 8'b0;
    //assign VRAM_Dout = 8'b0;
    
    //assign OAM_Fetch_Done = (oam_scan_i == (OAM_ENTRY_MAX - 1));

    //------------------------------ MMIO ---------------------------------

    always @( * )
        begin
            // MMIO Bus
            MMIO_Dout = 8'hFF;
            case( MMIO_Addr )
                16'hFF40: MMIO_Dout = Register_LCDC;
                16'hFF41: MMIO_Dout = Register_Status;
                16'hFF42: MMIO_Dout = Register_Scroll_Y;
                16'hFF43: MMIO_Dout = Register_Scroll_X;
                16'hFF44: MMIO_Dout = Register_LY;
                16'hFF45: MMIO_Dout = Register_Coord_Yc;
                16'hFF46: MMIO_Dout = Register_DMA;
                16'hFF47: MMIO_Dout = Register_Bkgd_Plt;
                16'hFF48: MMIO_Dout = Register_Obj_Plt_0;
                16'hFF49: MMIO_Dout = Register_Obj_Plt_1;
                16'hFF4A: MMIO_Dout = Register_Win_Pos_Y;
                16'hFF4B: MMIO_Dout = Register_Win_Pos_X;
            endcase
        end
always @( posedge clk, posedge rst )
    begin
        if( rst ) 
        begin
            Register_LCDC            <= 8'h00;
            Register_Status[ 7 : 3 ] <= 5'h00;
            Register_Scroll_Y        <= 8'h00;
            Register_Scroll_X        <= 8'h00;
            Register_LY              <= 8'h00;
            Register_DMA             <= 8'h00;
            Register_Bkgd_Plt        <= 8'hFC;
            Register_Obj_Plt_0       <= 8'h00;
            Register_Obj_Plt_1       <= 8'h00;
            Register_Win_Pos_Y       <= 8'h00;
            Register_Win_Pos_X       <= 8'h00;
        end
        else if( MMIO_Write )
        begin
            case( MMIO_Addr )
                16'hFF40: Register_LCDC            <= MMIO_Din;
                16'hFF41: Register_Status[ 7 : 3 ] <= MMIO_Din[ 7 : 3 ];
                16'hFF42: Register_Scroll_Y        <= MMIO_Din;
                16'hFF43: Register_Scroll_X        <= MMIO_Din;
                16'hFF44: Register_LY              <= MMIO_Din;
                16'hFF45: Register_Coord_Yc        <= MMIO_Din;
                16'hFF46: Register_DMA             <= MMIO_Din;
                16'hFF47: Register_Bkgd_Plt        <= MMIO_Din;
                16'hFF48: Register_Obj_Plt_0       <= MMIO_Din;
                16'hFF49: Register_Obj_Plt_1       <= MMIO_Din;
                16'hFF4A: Register_Win_Pos_Y       <= MMIO_Din;
                16'hFF4B: Register_Win_Pos_X       <= MMIO_Din;
            endcase
            // VRAM and OAM access are not handled here
        end
    end

    //------------------------------- FSM ---------------------------------

    always@(*)
        if(State_Out >= 4'b0010 && State_Out <= 4'b0111)
        begin
            OAM_Read <= 1'b1;
            //OAM_Write <= 1'b0;
            VRAM_Read <= 1'b0;
            //VRAM_Write <= 1'b0;
        end
        
        else if(State_Out >= 4'b1000) // && 
        begin
            OAM_Read <= 1'b0;
            //OAM_Write <= 1'b0;
            VRAM_Read <= 1'b1;
            //VRAM_Write <= 1'b0;
        end

        else
        begin
            OAM_Read <= 1'b0;
            //OAM_Write <= 1'b0;
            VRAM_Read <= 1'b0;
            //VRAM_Write <= 1'b0;
        end

    // next state assignment, and global reset assignments

    always @( posedge clk, posedge rst )
        if(rst)
        begin
            State <= 0;
            Frame_Done <= 0;
            Buffering <= 0;
            OAM_Fetch_Done <= 0;
            OAM_Dout <= 0;
            VRAM_Dout <= 0;
            //State_Done <= 1;
            OAM_Addr <= OAM_MIN;
            VRAM_Addr <= TILE_8_MIN;
            TBPP <= 2'b0;
            oam_scan_i <= 0;
            oam_buf_i <= 0;
            oam_buf_of <= 0;
            oam_buf_count <= 0;
            vram_buf_of <= 0;
            bg_win_fetch <= 0;
            Drawing <= 0;
            temp <= 0;
            
            for(i = 0; i < 10; i = i + 1)
            begin
                oam_to_display[i] <= 0;
            end
            
            for(i = 0; i < SCANLINE_MAX - 1; i = i + 1)
            begin    
                scanline[i] <= 0;
            end
            
            for(i = 0; i < OAM_ENTRY_MAX; i = i + 1)
            begin
                oam_y_temp      [i] <= 0;
                oam_x_temp      [i] <= 0;
                oam_num_temp    [i] <= 0;
                oam_flags_temp  [i] <= 0;
            end
        end
        else
            begin
            State <= Next_State;
            end
    
    // outputing to VGA
    always@(posedge clk)
        if(Drawing && Buffering)
        begin
            if(i < VGA_SCANLINE_MAX)//PIXEL_MAX)
            begin
                /*
                TBPP[0] <= scanline[i * 2];
                TBPP[1] <= scanline[(i * 2) + 1];
                */
                if(i < PIXEL_MAX)
                begin
                    /*
                    TBPP[0] <= scanline[i * 2];
                    TBPP[1] <= scanline[(i * 2) + 1];
                    */
                    TBPP[0] <= i[0];
                    TBPP[1] <= i[1];
                end
                else
                begin
                    TBPP[0] <= 1'b0;
                    TBPP[1] <= 1'b1;
                end
                i = i + 1;
            end
            else
            begin
                OAM_Addr <= OAM_MIN;
                VRAM_Addr <= TILE_8_MIN;
                Drawing <= 1'b0;
                Buffering <= 1'b0;
                i = 0;
            end
        end
        
    always @( State, Buffering, PPU_Enabled)//Frame_Done, Buffering, OAM_Fetch_Done, PPU_Enabled, State_Done ) // this sensitivity list is probably wrong
            casez( { State, Frame_Done, Buffering, PPU_Enabled } )       // needs variables to handle case changing
                7'b0000_0_0_1 : Next_State = OAM_SCAN_Y;    // H_Blank -> OAM_SCAN (not end of frame)
                7'b0000_1_0_1 : Next_State = V_BLANK;       // H_Blank -> V_Blank (end of frame)
                7'b0001_?_?_1 : Next_State = OAM_SCAN_Y;    // V_Blank -> OAM_SCAN_Y
                7'b0010_?_?_1 : Next_State = OAM_SCAN_X;    // OAM_SCAN_Y -> OAM_SCAN_X
                7'b0011_?_?_1 : Next_State = OAM_SCAN_NUM;  // OAM_SCAN_X -> OAM_SCAN_NUM
                7'b0100_?_?_1 : Next_State = OAM_SCAN_FLAGS;// OAM_SCAN_NUM -> OAM_SCAN_FLAGS
                7'b0101_?_1_1 : Next_State = OAM_SCAN_Y;    // OAM_SCAN_FLAGS -> OAM_SCAN_Y
                7'b0101_?_0_1 : Next_State = OAM_SCAN_BUF_1;// OAM_SCAN_FLAGS -> OAM_SCAN_BUF_1
                7'b0110_?_?_1 : Next_State = OAM_SCAN_BUF_2;// OAM_SCAN_BUF_1 -> OAM_SCAN_BUF_2
                7'b0111_?_1_1 : Next_State = OAM_SCAN_BUF_1;// OAM_SCAN_BUF_2 -> OAM_SCAN_BUF_1
                7'b0111_?_0_1 : Next_State = DRAW_TILE;     // OAM_SCAN_BUF_2 -> DRAW_TILE
                7'b1000_?_?_1 : Next_State = DRAW_BG_1;     // DRAW_TILE -> DRAW_WIN_1
                7'b1001_?_?_1 : Next_State = DRAW_BG_2;     // DRAW_WIN_1 -> DRAW_WIN_2
                7'b1010_?_1_1 : Next_State = DRAW_BG_1;     // DRAW_WIN_2 -> DRAW_WIN_1
                7'b1010_?_0_1 : Next_State = DRAW_WIN_1;    // DRAW_WIN_2 -> DRAW_BG_1
                7'b1011_?_?_1 : Next_State = DRAW_WIN_2;    // DRAW_BG_1 -> DRAW_BG_2
                7'b1100_?_1_1 : Next_State = DRAW_WIN_1;    // DRAW_BG_2 -> DRAW_BG_1
                7'b1100_?_0_1 : Next_State = DRAW_OUTPUT;   // DRAW_BG_2 -> DRAW_OUTPUT
                7'b1101_?_1_1 : Next_State = DRAW_OUTPUT;   // DRAW_OUTPUT -> DRAW_OUTPUT
                7'b1101_?_0_1 : Next_State = H_BLANK;       // DRAW_OUTPUT -> H_Blank
                7'b1110_?_0_1 : Next_State = OAM_SCAN_Y;    // PPU active, INCATIVE -> OAM_SCAN_Y
                7'b????_?_?_0 : Next_State = INACTIVE;      // PPU inactive -> INACTIVE
                default       : Next_State = INACTIVE;
            endcase
                           
        
    always @( posedge clk )
        case( State )
            H_BLANK: 
            begin
                //State_Done <= 1'b0;
                //waits for end of scanline
                //State_Done <= 1'b1;
            end

            V_BLANK:
            begin
                //State_Done <= 1'b0;
                //waits for end of frame
                //State_Done <= 1'b1;
            end

            OAM_SCAN_Y:
            begin
                //State_Done <= 1'b0;
                
                //Buffering <= 1'b1;
                //Frame_Done <= 1'b0;
                //OAM_Fetch_Done <= 1'b0;
                oam_y_temp      [oam_scan_i] <= OAM_Din;
                OAM_Addr <= OAM_Addr + 16'h1;
                oam_scan_i <= oam_scan_i + 6'b1;
                
                //State_Done <= 1'b1;
            end

            OAM_SCAN_X:
            begin
                //State_Done <= 1'b0;

                oam_x_temp      [oam_scan_i] <= OAM_Din;
                OAM_Addr <= OAM_Addr + 16'h1;
                oam_scan_i <= oam_scan_i + 6'b1;
                
                //State_Done <= 1'b1;
            end

            OAM_SCAN_NUM:
            begin
                //State_Done <= 1'b0;

                oam_num_temp    [oam_scan_i] <= OAM_Din;
                OAM_Addr <= OAM_Addr + 16'h1;
                oam_scan_i <= oam_scan_i + 6'b1;
                
                //Buffering <= (oam_scan_i >= (OAM_ENTRY_MAX - 2));
                if(oam_scan_i >= (OAM_ENTRY_MAX - 2))
                begin
                   //OAM_Fetch_Done <= 1'b1;
                   oam_scan_i <= 0;
                   Buffering <= 1'b0;
                end
                else
                    Buffering <= 1'b1;
                
                //State_Done <= 1'b1;
            end

            OAM_SCAN_FLAGS:
            begin
                //State_Done <= 1'b0;
                
                oam_flags_temp  [oam_scan_i] <= OAM_Din;
                OAM_Addr <= OAM_Addr + 16'h1;
                oam_scan_i <= oam_scan_i + 6'b1;
                
                /*
                if( oam_scan_i >= OAM_ENTRY_MAX - 1)
                begin
                    OAM_Fetch_Done <= 1'b1;
                    oam_scan_i <= 0;
                    //Buffering <= 1'b0;
                end
                */
                
                // OAM_Fetch_Done <= (oam_scan_i >= (OAM_ENTRY_MAX - 1));
                
                //State_Done <= 1'b1;
            end

            OAM_SCAN_BUF_1:
            begin
                //State_Done <= 1'b0;
                
                //OAM_Fetch_Done <= 1'b0;
                Buffering <= 1'b1;

                if( (oam_x_temp[oam_scan_i] > 0) &&
                    (oam_y_temp[oam_scan_i] <= Register_LY + 16) &&
                    (oam_y_temp[oam_scan_i] + sprite_height >= Register_LY + 16) &&
                    (oam_buf_count <= 10))
                begin
                    //oam_buffer[oam_buf_i] <= VRAM_Din;
                    VRAM_Addr <= TILE_8_MIN + (oam_num_temp[oam_scan_i] * 16) + oam_buf_of;
                end

                else if( (oam_buf_i >= OAM_TILE_MAX - 2) ||
                    (oam_buf_count >= 9) ||
                    (oam_scan_i >= OAM_ENTRY_MAX - 2))
                begin
                    Buffering <= 1'b0;
                    oam_scan_i <= 1'b0;
                end
                
                //State_Done <= 1'b1;
            end

            OAM_SCAN_BUF_2:
            begin
                //State_Done <= 1'b0;
                
                if( (oam_x_temp[oam_scan_i] > 0) &&
                    (oam_y_temp[oam_scan_i] <= Register_LY + 16) &&
                    (oam_y_temp[oam_scan_i] + sprite_height >= Register_LY + 16) &&
                    (oam_buf_count <= 10))
                    begin
                        oam_buffer[oam_buf_i] <= VRAM_Din;
                        //VRAM_Addr <= TILE_8_MIN + (oam_num_temp[oam_buf_i] * 16);
                        oam_buf_of <= oam_buf_of + 16'h1;
                        oam_buf_i <= oam_buf_i + 8'b1;

                        if( oam_buf_of >= 16'd16)
                        begin
                            oam_to_display[oam_buf_count] <= oam_scan_i;
                            oam_buf_of <= 0;
                            oam_scan_i <= oam_scan_i + 6'b1;
                            oam_buf_count <= oam_buf_count + 4'b1;
                        end
                    end

                else
                begin
                    oam_buf_of <= 0;
                    oam_scan_i <= oam_scan_i + 6'b1;
                end
            end
            DRAW_TILE:
            begin
                //Buffering <= 1'b1;
                for(i = 0; i < 10; i = i + 1)
                begin
                    temp = Register_LY - oam_y_temp[oam_to_display[i]];
                    scanline_buffer[i * 2] <= oam_buffer[(i * 16) + (temp * 2)];
                    scanline_buffer[(i * 2) + 1] <= oam_buffer[(i * 16) + (temp * 2) + 1];
                end
                
                for(i = 0; i < 10; i = i + 1)
                begin
                    temp = oam_x_temp[oam_to_display[i]];
                    
                    for(j = 0; j < 8; j = j + 1)
                    begin
                        scanline[temp + (j * 2)] <= scanline_buffer[j][i * 2];
                        scanline[temp + (j * 2) + 1] <= scanline_buffer[j][(i * 2) + 1];
                    end
                end
            end
            
            DRAW_BG_1:
            begin
                Buffering <= 1'b1;
                
                if((vram_buf_of < 16'd32) && 
                    (bg_win_fetch == 1'b0))
                begin
                    VRAM_Addr <= BG_MIN + (Register_LY * 32) + vram_buf_of;
                end
                else if((vram_buf_of >= 16'd32) &&
                        (bg_win_fetch == 1'b0))
                begin
                    bg_win_fetch <= 1'b1;
                    vram_buf_of <= 0;
                end
                else if((vram_buf_of < 16'd32) &&
                        (bg_win_fetch == 1'b1))
                begin
                    VRAM_Addr <= TILE_8_MIN + (bg_buffer[vram_buf_of] * 16);
                end
                else if((vram_buf_of >= 16'd32) &&
                        (bg_win_fetch == 1'b1))
                begin
                    Buffering <= 1'b0;
                    vram_buf_of <= 1'b0;
                    bg_win_fetch <= 1'b0;
                end
            end
            
            DRAW_BG_2:
            begin
                bg_buffer[vram_buf_of] <= VRAM_Din;
                vram_buf_of <= vram_buf_of + 16'b1;
            end
            
            DRAW_WIN_1:
            begin
                Buffering <= 1'b1;
                
                if((vram_buf_of < 16'd32) && 
                    (bg_win_fetch == 1'b0))
                begin
                    VRAM_Addr <= WIN_MIN + (Register_LY * 32) + vram_buf_of;
                end
                else if((vram_buf_of >= 16'd32) &&
                        (bg_win_fetch == 1'b0))
                begin
                    bg_win_fetch <= 1'b1;
                    vram_buf_of <= 0;
                end
                else if((vram_buf_of < 16'd32) &&
                        (bg_win_fetch == 1'b1))
                begin
                    VRAM_Addr <= TILE_8_MIN + (bg_buffer[vram_buf_of] * 16);
                end
                else if((vram_buf_of >= 16'd32) &&
                        (bg_win_fetch == 1'b1))
                begin
                    Buffering <= 1'b0;
                    vram_buf_of <= 1'b0;
                    bg_win_fetch <= 1'b0;
                end
            end
            
            DRAW_WIN_2:
            begin
                win_buffer[vram_buf_of] <= VRAM_Din;
                vram_buf_of <= vram_buf_of + 16'b1;
            end
            
            DRAW_OUTPUT:
            begin
                if(Buffering == 1'b0)
                begin
                    Buffering <= 1'b1;
                    Drawing <= 1'b1;
                    i = 0;
                end
                //Drawing <= 1'b1;
            end
            
            INACTIVE:
            begin
                //State_Done <= 1'b0;
                // does nothing
                //State_Done <= 1'b1;
            end
        endcase

endmodule