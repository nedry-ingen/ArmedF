/*  SPDX-License-Identifier: GPL-3.0-or-later

    crt_adjust_glue.sv -- MiSTer glue for rmonic79's CRT Adjust
    (https://github.com/rmonic79/MiSTer-CRT-Adjust, crt_adjust.sv, core-side
    variant). Adapted from examples/core_side_snippet.v of that repository.

    It sits in the emu module between arcade_video and the VGA_* / CE_PIXEL
    outputs, so sys/ is never modified.

    Notes on how it differs from the reference snippet:

    * Read clock-enable: the accumulator adds RD_INC per clock and fires every
      RD_BASE+hsize, so the read period is (RD_BASE+hsize)/RD_INC clocks.
      RD_BASE/RD_INC must equal clk/pixel. Each H-Size step is 1/RD_BASE.

    * vb_in: crt_adjust latches vb_in on each HSync rise and delays it one
      line, which assumes VBlank already holds the value of the NEXT line when
      HSync rises. Cores that change VBlank after HSync (Inferno and ArmedF do) would
      lose their last active line, so vb_in is generated here as "VBlank of
      the next line", taken from the same line of the previous frame.

    * de_osd: the native-anchored DE rise is regenerated from the measured
      HSync-to-DE distance so that it exists on every line the module outputs.

    * H-Position is entered in HPOS_STEP-pixel steps: 0..24 = 0..+24*STEP,
      25..48 = -24*STEP..-STEP.

    * Frame position (st_frame): delays (or advances) the native HSync before
      anything else, so the whole frame moves on the CRT with CRT Adjust On or
      Off. 4-pixel steps: 0..FRAME_LMAX = 0,-4,..,-4*FRAME_LMAX px (left),
      then three more entries = +12,+8,+4 px (right).
*/

module crt_adjust_glue #(parameter
    HTOTAL    = 448,
    VTOTAL    = 272,
    HPOS_MODE = 1,      // 1 = CONTENTSHIFT, 0 = SYNCSHIFT (see crt_adjust.sv)
    RD_INC    = 7,      // read accumulator increment per clk
    RD_BASE   = 48,     // read period at hsize=0, in RD_INC units
    FRAME_LMAX= 12,     // number of 4-px left steps in the frame option
    HPOS_STEP = 2       // H-Position pixels per OSD step (1..4)
)(
    input            clk,
    // OSD
    input            enable,        // CRT Adjust On/Off
    input      [4:0] st_hsize,      // signed, -16..+15
    input      [5:0] st_hpos,       // 0..24 = right, 25..48 = left (see HPOS_STEP)
    input      [4:0] st_vshift,     // signed, -16..+15 lines
    input      [3:0] st_frame,      // whole-frame H position, see above
    input            scandoubler,   // the module only works at the native rate
    // native video (arcade_video output)
    input            ce_in,
    input      [7:0] r_in, g_in, b_in,
    input            hs_raw, vs_in, de_in,
    // adjusted video
    output           ce_out,
    output     [7:0] r_out, g_out, b_out,
    output           hs_out, vs_out,
    output           de_out         // to video_freak VGA_DE_IN
);

// ─── Decode the OSD values ──────────────────────────────────────────────────
reg              crt_on = 0;
reg signed [4:0] hsize_s;
reg signed [5:0] vshift_s;
reg        [5:0] hpos_d;
reg signed [8:0] hpos_off;

always @(posedge clk) if( ce_in ) begin
    crt_on   <= enable & ~scandoubler;
    hsize_s  <= $signed(st_hsize);
    vshift_s <= $signed(st_vshift);
    hpos_d   <= st_hpos;
    hpos_off <= hpos_d <= 6'd24 ? $signed( 9'(hpos_d) * 9'(HPOS_STEP) )
                                : $signed( 9'(hpos_d) * 9'(HPOS_STEP) ) - $signed( 9'(49*HPOS_STEP) );
end

// ─── Frame position: HSync delayed by frame_dly native pixels ────────────────
reg  [3:0]        frame_d;
reg  [9:0]        frame_dly = 0;
reg  [HTOTAL-1:0] hs_sr = 0;
wire              hs_in = frame_dly==0 ? hs_raw : hs_sr[frame_dly-10'd1];

always @(posedge clk) if( ce_in ) begin
    frame_d   <= st_frame;
    frame_dly <= frame_d <= 4'(FRAME_LMAX)
               ? { 4'd0, frame_d, 2'd0 }                                     // later: frame left
               : 10'(HTOTAL) - { 3'd0, 5'(FRAME_LMAX+4) - {1'b0,frame_d}, 2'd0 }; // earlier: frame right
    hs_sr     <= { hs_sr[HTOTAL-2:0], hs_raw };
end

// ─── Read clock-enable, reset on hs_ref_out ─────────────────────────────────
wire       hs_ref;
reg        hs_ref_d = 0;
wire       hs_ref_rise = hs_ref & ~hs_ref_d;
wire [7:0] rd_period = 8'(RD_BASE) + {{3{hsize_s[4]}}, hsize_s};
reg  [7:0] rd_acc = 0;
wire [8:0] rd_next = {1'b0, rd_acc} + 9'(RD_INC);
wire       rd_tick = rd_next >= {1'b0, rd_period};
wire       rd_ce   = crt_on ? rd_tick : ce_in;

always @(posedge clk) begin
    hs_ref_d <= hs_ref;
    if     ( hs_ref_rise ) rd_acc <= 0;
    else if( rd_tick     ) rd_acc <= rd_next[7:0] - rd_period;
    else                   rd_acc <= rd_next[7:0];
end

// ─── Line timing measured on the native stream ──────────────────────────────
reg        hs_l = 0, de_l = 0, had_de = 0, vb_cur = 1;
reg  [9:0] hcnt = 0, de_start = 0;
reg  [VTOTAL-1:0] vbsr = 0;
wire       hs_rise = ce_in & hs_in & ~hs_l;
wire       vb_next = vbsr[VTOTAL-2]; // VBlank of the next line (previous frame)

always @(posedge clk) begin
    if( ce_in ) begin
        hs_l <= hs_in;
        de_l <= de_in;
        hcnt <= hs_rise ? 10'd0 : hcnt + 10'd1;
        if( de_in & ~de_l ) de_start <= hcnt;
        if( de_in ) had_de <= 1;
    end
    if( hs_rise ) begin
        had_de <= 0;
        vb_cur <= ~had_de;                       // VBlank of the line just written
        vbsr   <= { vbsr[VTOTAL-2:0], ~had_de };
    end
end

// ─── CRT Adjust ─────────────────────────────────────────────────────────────
wire [7:0] str_r, str_g, str_b;
wire       str_hs, str_vs, str_hb, str_vb;

crt_adjust #(
    .VTOTAL   ( VTOTAL    ),
    .HTOTAL   ( HTOTAL    ),
    .HPOS_MODE( HPOS_MODE )
) u_crt_adjust (
    .clk        ( clk       ),
    .pxl_cen    ( ce_in     ),
    .pxl2_cen   ( rd_ce     ),
    .active     ( crt_on    ),
    .hsize      ( hsize_s   ),
    .hoffset    ( hpos_off  ),
    .voffset    ( vshift_s  ),
    .r_in       ( r_in      ),
    .g_in       ( g_in      ),
    .b_in       ( b_in      ),
    .hs_in      ( hs_in     ),
    .vs_in      ( vs_in     ),
    .hb_in      ( ~de_in    ),     // HBlank | VBlank
    .vb_in      ( vb_next   ),
    .r_out      ( str_r     ),
    .g_out      ( str_g     ),
    .b_out      ( str_b     ),
    .hs_out     ( str_hs    ),
    .vs_out     ( str_vs    ),
    .hb_out     ( str_hb    ),
    .vb_out     ( str_vb    ),
    .hs_ref_out ( hs_ref    )
);

// ─── OSD stays put: DE rise anchored to the native active start ─────────────
// The module outputs during this line the picture line whose VBlank is vb_cur
wire native_rise = ce_in && !hs_rise && hcnt == de_start && !vb_cur;
reg  str_active_d = 0, de_osd = 0;
wire str_fall = str_active_d & str_hb;

always @(posedge clk) begin
    if( rd_ce ) str_active_d <= ~str_hb;
    if     ( native_rise       ) de_osd <= 1;
    else if( rd_ce && str_fall ) de_osd <= 0;
end

// ─── Outputs ────────────────────────────────────────────────────────────────
assign ce_out = crt_on ? rd_ce  : ce_in;
assign r_out  = crt_on ? str_r  : r_in;
assign g_out  = crt_on ? str_g  : g_in;
assign b_out  = crt_on ? str_b  : b_in;
assign hs_out = crt_on ? str_hs : hs_in;
assign vs_out = crt_on ? str_vs : vs_in;
assign de_out = crt_on ? de_osd : de_in;

endmodule
