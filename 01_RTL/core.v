module core (                       //Don't modify interface
	input      		i_clk,
	input      		i_rst_n,
	input    	  	i_in_valid,
	input 	[31: 0] i_in_data,

	output			o_in_ready,

	output	[ 7: 0]	o_out_data1,
	output	[ 7: 0]	o_out_data2,
	output	[ 7: 0]	o_out_data3,
	output	[ 7: 0]	o_out_data4,

	output	[11: 0] o_out_addr1,
	output	[11: 0] o_out_addr2,
	output	[11: 0] o_out_addr3,
	output	[11: 0] o_out_addr4,

	output 			o_out_valid1,
	output 			o_out_valid2,
	output 			o_out_valid3,
	output 			o_out_valid4,

	output 			o_exe_finish
);

	// ============================================================================
	// Output ports (Sequential)
	// ============================================================================
	reg [7:0]  o_out_data1_r, o_out_data2_r, o_out_data3_r, o_out_data4_r;
	reg        o_in_ready_r;
	reg        o_out_valid1_r, o_out_valid2_r, o_out_valid3_r, o_out_valid4_r;
	reg [11:0] o_out_addr1_r;
	reg        exe_finish_r;

	assign o_out_data1  = o_out_data1_r;
	assign o_out_data2  = o_out_data2_r;
	assign o_out_data3  = o_out_data3_r;
	assign o_out_data4  = o_out_data4_r;
	assign o_out_valid1 = o_out_valid1_r;
	assign o_out_valid2 = o_out_valid2_r;
	assign o_out_valid3 = o_out_valid3_r;
	assign o_out_valid4 = o_out_valid4_r;
	assign o_in_ready   = o_in_ready_r;
	assign o_out_addr1  = o_out_addr1_r;
	assign o_exe_finish = exe_finish_r;

	assign o_out_addr2 = 12'd0;
	assign o_out_addr3 = 12'd0;
	assign o_out_addr4 = 12'd0;

	// ============================================================================
	// FSM States
	// ============================================================================
	localparam S_IDLE        = 5'b00000;
	localparam S_IMG         = 5'b00001;
	localparam S_BUFF        = 5'b00010;
	localparam S_WEI         = 5'b00011;
	localparam S_CONV11_INIT = 5'b00100;
	localparam S_CONV11      = 5'b00101;
	localparam S_CONV11_BUFF = 5'b00110;
	localparam S_DONE        = 5'b00111;
	localparam S_CONV12_INIT = 5'b01000;
	localparam S_CONV12      = 5'b01001;
	localparam S_CONV12_BUFF = 5'b01010;
	localparam S_CONV21_INIT = 5'b01011;
	localparam S_CONV21      = 5'b01100;
	localparam S_CONV21_BUFF = 5'b01101;
	localparam S_CONV22_INIT = 5'b01110;
	localparam S_CONV22      = 5'b01111;
	localparam S_CONV22_BUFF = 5'b10000;

	reg [4:0] state_r, next_state_w;

	// ============================================================================
	// Code 128-C Barcode Constants
	// ============================================================================
	localparam START_CODE = 11'b110_1001_1100;
	localparam ONE_CODE   = 11'b110_0110_1100;
	localparam TWO_CODE   = 11'b110_0110_0110;
	localparam THREE_CODE = 11'b100_1001_1000;
	localparam STOP_CODE  = 13'b1_1000_1110_1011;

	// ============================================================================
	// Barcode Decoding (Sequential Registers)
	// ============================================================================
	reg [12:0] shift_lsb_r;
	reg [6:0]  pass_cnt_r;
	reg [6:0]  seq_cnt_r;
	reg [3:0]  hei_cnt_r;
	reg [8:0]  index_r;
	reg [56:0] seq_r;
	reg [56:0] curr_seq_r;
	reg [1:0]  ker_r, str_r, dil_r;
	reg        start_r, row_start_r, barcode_done_r;

	// Barcode Decoding (Combinational Wires)
	wire [3:0]  lsb_w;
	wire [12:0] shift_lsb_next_w;
	wire [6:0]  pass_cnt_next_w;
	wire [2:0]  ker_decoded_w, str_decoded_w, dil_decoded_w;
	wire        barcode_valid_w;
	wire        is_53_w = (seq_cnt_r == 53);
	wire        is_54_w = (seq_cnt_r == 54);
	wire        is_55_w = (seq_cnt_r == 55);
	wire        is_56_w = (seq_cnt_r == 56);
	wire        not_stop53_w  = (is_53_w && ({curr_seq_r[8:0], lsb_w} != STOP_CODE));
	wire        not_stop54_w  = (is_54_w && ({curr_seq_r[9:0], lsb_w[3:1]} != STOP_CODE));
	wire        not_stop55_w  = (is_55_w && ({curr_seq_r[10:0], lsb_w[3:2]} != STOP_CODE));
	wire        not_stop56_w  = (is_56_w && ({curr_seq_r[11:0], lsb_w[3]} != STOP_CODE));
	wire        not_stop_w    = (not_stop53_w || not_stop54_w || not_stop55_w || not_stop56_w);
	wire        match_seq53_w = (is_53_w && (seq_r == {curr_seq_r[52:0], lsb_w}));
	wire        match_seq54_w = (is_54_w && (seq_r == {curr_seq_r[53:0], lsb_w[3:1]}));
	wire        match_seq55_w = (is_55_w && (seq_r == {curr_seq_r[54:0], lsb_w[3:2]}));
	wire        match_seq56_w = (is_56_w && (seq_r == {curr_seq_r[55:0], lsb_w[3]}));
	wire        is_match_seq_w = (match_seq53_w || match_seq54_w || match_seq55_w || match_seq56_w);

	assign lsb_w = {i_in_data[24], i_in_data[16], i_in_data[8], i_in_data[0]};
	assign shift_lsb_next_w = (pass_cnt_r == 0) ? {9'b0, lsb_w} : {shift_lsb_r[8:0], lsb_w};
	assign pass_cnt_next_w  = (pass_cnt_r == 60) ? 7'd0 : pass_cnt_r + 7'd4;

	// ============================================================================
	// Weight Storage (Sequential)
	// ============================================================================
	reg signed [7:0] weight_r [0:8];
	reg [1:0]        wei_cnt_r;
	
	wire [1:0] wei_cnt_next_w;
	assign wei_cnt_next_w = (wei_cnt_r == 2'd2) ? 2'd0 : wei_cnt_r + 2'd1;

	// ============================================================================
	// Image Storage (Sequential Registers)
	// ============================================================================
	reg        flip_r;
	reg        wen_r      [0:7];
	reg [8:0]  img_addr_r [0:7];
	reg [7:0]  img_in_r   [0:7];
	reg        img_done_r;
	reg [2:0]  img_addr_inc_r, img_addr_inc_w;
	reg [5:0]  img_addr_offset_r, img_addr_offset_w; 

	// Image Storage (Combinational Wires)
	wire [7:0] img_out_w  [0:7];
	wire [8:0] img_addr_col_w = img_addr_inc_r << 6;
	
	integer i;
	genvar j;

	// ============================================================================
	// Convolution (Sequential Registers)
	// ============================================================================
	reg [5:0]  row_cnt_r, col_cnt_r;
	reg [7:0]  mul_r [0:8];
	reg [2:0]  startup_delay_r, startup_delay_w;
	reg [1:0]  three_cnt_r, three_cnt_w;

	// Convolution (Combinational Wires)
	wire signed [16:0] prod [0:8];
	wire signed [20:0] sum_row0_w, sum_row1_w, sum_row2_w;
	wire signed [20:0] acc_sum, acc_rounded;
	wire signed [13:0] acc_shifted;
	wire [7:0]  acc_clamped_w;
	wire [63:0] sram_packed_w;

	// Parallel multipliers
	assign prod[0] = $signed({1'b0, mul_r[0]}) * weight_r[0];
	assign prod[1] = $signed({1'b0, mul_r[1]}) * weight_r[1];
	assign prod[2] = $signed({1'b0, mul_r[2]}) * weight_r[2];
	assign prod[3] = $signed({1'b0, mul_r[3]}) * weight_r[3];
	assign prod[4] = $signed({1'b0, mul_r[4]}) * weight_r[4];
	assign prod[5] = $signed({1'b0, mul_r[5]}) * weight_r[5];
	assign prod[6] = $signed({1'b0, mul_r[6]}) * weight_r[6];
	assign prod[7] = $signed({1'b0, mul_r[7]}) * weight_r[7];
	assign prod[8] = $signed({1'b0, mul_r[8]}) * weight_r[8];

	// Row sums
	assign sum_row0_w  = prod[0] + prod[1] + prod[2];
	assign sum_row1_w  = prod[3] + prod[4] + prod[5];
	assign sum_row2_w  = prod[6] + prod[7] + prod[8];

	// Accumulation, rounding, clamping
	assign acc_sum     = sum_row0_w + sum_row1_w + sum_row2_w;
	assign acc_rounded = acc_sum + 21'sd64;
	assign acc_shifted = acc_rounded >>> 7;
	assign acc_clamped_w = (acc_shifted < 14'sd0)   ? 8'd0 :
	                       (acc_shifted > 14'sd255) ? 8'd255 :
	                       acc_shifted[7:0];

	// Pack SRAM outputs
	assign sram_packed_w = {img_out_w[7], img_out_w[6], img_out_w[5], img_out_w[4],
	                        img_out_w[3], img_out_w[2], img_out_w[1], img_out_w[0]};

	// Boundary detection
	wire dil1_stop_w = (dil_r == 2'd1) && (row_cnt_r == 6'd61);
	wire dil1_inc_addr_w = (dil_r == 2'd1) && (row_cnt_r == 6'd62);
	wire str1_inc_col_w = (row_cnt_r == 6'd63);
	wire conv_done_w    = (str1_inc_col_w && col_cnt_r == 6'd63);

	// Padding detection
	wire signed [7:0] win_col_left  = $signed({1'b0, col_cnt_r}) - $signed({6'b0, dil_r});
	wire signed [7:0] win_col_right = $signed({1'b0, col_cnt_r}) + $signed({6'b0, dil_r});
	wire signed [7:0] win_next_col_right = $signed({1'b0, col_cnt_r}) + 1 + $signed({6'b0, dil_r});  // 1 = str but omitted
	wire signed [7:0] win_row_bot   = $signed({1'b0, row_cnt_r}) + $signed({6'b0, 1'b1}) + $signed({6'b0, dil_r}); // 1 = str but omitted
	wire pad_left   = (win_col_left < 8'sd0);
	wire pad_right  = (win_col_right > 8'sd63);
	wire pad_bottom = ((dil_r == 1) && (win_row_bot - 1 > 8'sd63)) || (dil_r == 2 && win_row_bot - 2 > 8'sd63);
	wire [7:0] bit_index_w = col_cnt_r[2:0] << 3;
	wire [7:0] left_index_w = (col_cnt_r[2:0] - dil_r) << 3;
	wire [7:0] right_index_w = (col_cnt_r[2:0] + dil_r) << 3;


	// Other look-ahead logic
	wire [8:0] addrf_reset_w = win_col_left[5:3] << 6;
	wire [8:0] addr0_reset_w = col_cnt_r[5:3] << 6;
	wire [8:0] addr1_reset_w = win_col_right[5:3] << 6;
	wire [8:0] addr2_reset_w = win_next_col_right[5:3] << 6;
	wire [5:0] row_dec_w = row_cnt_r - 6'd1;
	wire [5:0] col_dec_w = col_cnt_r - 6'd1;
	wire [4:0] half_row_w = row_cnt_r >> 1;
	wire [4:0] half_col_w = col_cnt_r >> 1;
	wire [4:0] half_row_dec_w = row_dec_w >> 1;
	wire [4:0] half_col_dec_w = col_dec_w >> 1;

	reg [7:0] buff1_r, buff1_w;
	reg [7:0] buff2_r, buff2_w;
	reg [4:0] prev_state_r;

	// ============================================================================
	// Function: Extract pixel with padding
	// ============================================================================
	function [7:0] extract_with_pad;
		input signed [7:0] col;
		reg [7:0] index_w;
		begin
			index_w = col[2:0] << 3;
			if (col < 8'sd0 || col > 8'sd63) begin
				extract_with_pad = 8'd0;  // Zero padding
			end
			else begin
				extract_with_pad = sram_packed_w[index_w +: 8];
			end
		end
	endfunction

	// ============================================================================
	// SRAM Instantiation
	// ============================================================================
	generate
		for (j = 0; j < 8; j = j + 1) begin : sram_gen
			sram_512x8 sram(
				.Q(img_out_w[j]),
				.CLK(i_clk),
				.CEN(1'b0),
				.WEN(wen_r[j]),
				.A(img_addr_r[j]),
				.D(img_in_r[j])
			);
		end
	endgenerate

	// ============================================================================
	// COMBINATIONAL: FSM Next State Logic
	// ============================================================================
	reg [5:0]  row_cnt_w, col_cnt_w;
	reg [8:0]  img_addr_w [0:7];
	reg [7:0]  mul_w [0:8];
	
	always @(*) begin
		// Default assignments
		next_state_w = state_r;
		row_cnt_w = row_cnt_r;
		col_cnt_w = col_cnt_r;
		buff1_w = buff1_r;
		buff2_w = buff2_r;
		img_addr_inc_w = img_addr_inc_r;
		img_addr_offset_w = img_addr_offset_r;
		startup_delay_w = startup_delay_r;
		three_cnt_w = three_cnt_r;
		
		for (i = 0; i < 8; i = i + 1) img_addr_w[i] = img_addr_r[i];
		for (i = 0; i < 9; i = i + 1) mul_w[i] = mul_r[i];
		
		case (state_r)
			S_IDLE: begin
				if (i_in_valid)
					next_state_w = S_IMG;
			end
			
			S_IMG: begin
				if (flip_r) img_addr_inc_w = img_addr_inc_r + 1;
				if (img_addr_inc_r == 7 && flip_r) img_addr_offset_w = img_addr_offset_r + 1;
				if (img_addr_r[0] == 9'd511 && flip_r == 1'b1)
					next_state_w = S_BUFF;
			end
			
			S_BUFF: begin
				if (ker_r != 3'd0)  // Valid barcode decoded
					next_state_w = S_WEI;
				else
					next_state_w = S_DONE;
			end
			
			S_WEI: begin
				case (wei_cnt_r)
					0: begin
						for (i = 0; i < 8; i = i + 1) img_addr_w[i] = 0;
					end
					1: begin
						if (dil_r == 2'd1) begin
							img_addr_w[0] = img_addr_r[0] + 1;
							img_addr_w[1] = img_addr_r[1] + 1;
						end
						else begin
							img_addr_w[0] = img_addr_r[0] + 2;
							img_addr_w[2] = img_addr_r[2] + 2;
						end
					end
					2: begin
						case ({str_r, dil_r})
							4'b0101: next_state_w = S_CONV11_INIT; 
							4'b0110: next_state_w = S_CONV12_INIT;
							4'b1001: next_state_w = S_CONV11_INIT;
							4'b1010: next_state_w = S_CONV12_INIT;
							default: next_state_w = state_r;
						endcase

						// Top row - all zeros (padding)
						mul_w[0] = 8'd0;
						mul_w[1] = 8'd0;
						mul_w[2] = 8'd0;
						
						// Middle row (row 0) - extract middle row elements
						mul_w[3] = 8'd0;
						mul_w[4] = sram_packed_w[0 +: 8];
						mul_w[5] = (dil_r == 1) ? sram_packed_w[8 +: 8] : sram_packed_w[16 +: 8];
						buff1_w = sram_packed_w[0 +: 8];
						buff2_w = (dil_r == 1) ? sram_packed_w[8 +: 8] : sram_packed_w[16 +: 8];
						
						// Bottom row - zeros (will load in first S_CONV11_INIT cycle)
						mul_w[6] = 8'd0;
						mul_w[7] = 8'd0;
						mul_w[8] = 8'd0;

						// Set SRAM addresses to read bottom row
						if (dil_r == 2'd1) begin
							img_addr_w[0] = img_addr_r[0] + 1;
							img_addr_w[1] = img_addr_r[1] + 1;
						end
						else begin
							img_addr_w[0] = 1;
							img_addr_w[2] = 1;
							three_cnt_w = 0;
							row_cnt_w = 1;
						end
					end
				endcase
			end
			
			// ====================================================================
			// S_CONV11_INIT: Initialize window for new column
			// ====================================================================
			S_CONV11_INIT: begin
				next_state_w = S_CONV11;
				row_cnt_w = 6'd1;
				startup_delay_w = 1;
				
				// Top row - all zeros (padding)
				mul_w[0] = 8'd0;
				mul_w[1] = 8'd0;
				mul_w[2] = 8'd0;
				
				// Middle row (row 0) - stay the same
				mul_w[3] = mul_r[3];
				mul_w[4] = mul_r[4];
				mul_w[5] = mul_r[5];
				
				// Bottom row - load data
				mul_w[6] = extract_with_pad(win_col_left);
				mul_w[7] = sram_packed_w[bit_index_w +: 8];
				mul_w[8] = extract_with_pad(win_col_right);
				
				case (col_cnt_r[2:0])
					3'd0: begin
						img_addr_w[7] = (col_cnt_r == 0) ? 0 : img_addr_r[7] + 1;
						img_addr_w[0] = img_addr_r[0] + 1;
						img_addr_w[1] = img_addr_r[1] + 1;
					end
					3'd1: begin
						img_addr_w[0] = img_addr_r[0] + 1;
						img_addr_w[1] = img_addr_r[1] + 1;
						img_addr_w[2] = img_addr_r[2] + 1;
					end
					3'd2: begin
						img_addr_w[1] = img_addr_r[1] + 1;
						img_addr_w[2] = img_addr_r[2] + 1;
						img_addr_w[3] = img_addr_r[3] + 1;
					end
					3'd3: begin
						img_addr_w[2] = img_addr_r[2] + 1;
						img_addr_w[3] = img_addr_r[3] + 1;
						img_addr_w[4] = img_addr_r[4] + 1;
					end
					3'd4: begin
						img_addr_w[3] = img_addr_r[3] + 1;
						img_addr_w[4] = img_addr_r[4] + 1;
						img_addr_w[5] = img_addr_r[5] + 1;
					end
					3'd5: begin
						img_addr_w[4] = img_addr_r[4] + 1;
						img_addr_w[5] = img_addr_r[5] + 1;
						img_addr_w[6] = img_addr_r[6] + 1;
					end
					3'd6: begin
						img_addr_w[5] = img_addr_r[5] + 1;
						img_addr_w[6] = img_addr_r[6] + 1;
						img_addr_w[7] = img_addr_r[7] + 1;
					end
					3'd7: begin
						img_addr_w[6] = img_addr_r[6] + 1;
						img_addr_w[7] = img_addr_r[7] + 1;
						img_addr_w[0] = img_addr_r[0] + 1;
					end
				endcase
			end
			
			// ====================================================================
			// S_CONV11: Main convolution11 loop with edge case handling
			// ====================================================================
			S_CONV11: begin
				if (dil1_stop_w) begin
					row_cnt_w = row_cnt_r + 1;
					for (i = 0; i < 8; i = i + 1) img_addr_w[i] = img_addr_r[i];
					startup_delay_w = 2;
					
					// Shift window upward
					mul_w[0] = mul_r[3];
					mul_w[1] = mul_r[4];
					mul_w[2] = mul_r[5];
					mul_w[3] = mul_r[6];
					mul_w[4] = mul_r[7];
					mul_w[5] = mul_r[8];
					
					// Load new bottom row from SRAM with edge detection
					if (pad_bottom) begin
						// Bottom edge - all zeros
						mul_w[6] = 8'd0;
						mul_w[7] = 8'd0;
						mul_w[8] = 8'd0;
					end
					else begin
						// Not at bottom edge - extract with left/right edge detection
						mul_w[6] = pad_left  ? 8'd0 : extract_with_pad(win_col_left);
						mul_w[7] = sram_packed_w[bit_index_w +: 8];
						mul_w[8] = pad_right ? 8'd0 : extract_with_pad(win_col_right);
					end
				end
				else if (dil1_inc_addr_w) begin
					row_cnt_w = row_cnt_r + 1;
					startup_delay_w = 2;
					
					// Shift window upward
					mul_w[0] = mul_r[3];
					mul_w[1] = mul_r[4];
					mul_w[2] = mul_r[5];
					mul_w[3] = mul_r[6];
					mul_w[4] = mul_r[7];
					mul_w[5] = mul_r[8];
					
					// Load new bottom row from SRAM with edge detection
					if (pad_bottom) begin
						// Bottom edge - all zeros
						mul_w[6] = 8'd0;
						mul_w[7] = 8'd0;
						mul_w[8] = 8'd0;
					end
					else begin
						// Not at bottom edge - extract with left/right edge detection
						mul_w[6] = pad_left  ? 8'd0 : extract_with_pad(win_col_left);
						mul_w[7] = sram_packed_w[bit_index_w +: 8];
						mul_w[8] = pad_right ? 8'd0 : extract_with_pad(win_col_right);
					end
					case (col_cnt_r[2:0])
						3'd0: img_addr_w[2] = addr2_reset_w;
						3'd1: img_addr_w[3] = addr2_reset_w;
						3'd2: img_addr_w[4] = addr2_reset_w;
						3'd3: img_addr_w[5] = addr2_reset_w;
						3'd4: img_addr_w[6] = addr2_reset_w;
						3'd5: img_addr_w[7] = addr2_reset_w;
						3'd6: img_addr_w[0] = addr2_reset_w;
						3'd7: img_addr_w[1] = addr2_reset_w;
					endcase
				end
				else if (str1_inc_col_w) begin
					startup_delay_w = 2;
					
					// Shift window upward
					mul_w[0] = mul_r[3];
					mul_w[1] = mul_r[4];
					mul_w[2] = mul_r[5];
					mul_w[3] = mul_r[6];
					mul_w[4] = mul_r[7];
					mul_w[5] = mul_r[8];
					
					// Load new bottom row from SRAM with edge detection
					if (pad_bottom) begin
						// Bottom edge - all zeros
						mul_w[6] = 8'd0;
						mul_w[7] = 8'd0;
						mul_w[8] = 8'd0;
					end
					else begin
						// Not at bottom edge - extract with left/right edge detection
						mul_w[6] = pad_left  ? 8'd0 : extract_with_pad(win_col_left);
						mul_w[7] = sram_packed_w[bit_index_w +: 8];
						mul_w[8] = pad_right ? 8'd0 : extract_with_pad(win_col_right);
					end
					row_cnt_w = row_cnt_r;
					next_state_w = S_CONV11_BUFF;
					case (col_cnt_r[2:0])
						3'd0: begin
							img_addr_w[0] = addr0_reset_w + 1;
							img_addr_w[1] = addr1_reset_w + 1;
							img_addr_w[2] = addr2_reset_w + 1;
						end
						3'd1: begin
							img_addr_w[1] = addr0_reset_w + 1;
							img_addr_w[2] = addr1_reset_w + 1;
							img_addr_w[3] = addr2_reset_w + 1;
						end
						3'd2: begin
							img_addr_w[2] = addr0_reset_w + 1;
							img_addr_w[3] = addr1_reset_w + 1;
							img_addr_w[4] = addr2_reset_w + 1;
						end
						3'd3: begin
							img_addr_w[3] = addr0_reset_w + 1;
							img_addr_w[4] = addr1_reset_w + 1;
							img_addr_w[5] = addr2_reset_w + 1;
						end
						3'd4: begin
							img_addr_w[4] = addr0_reset_w + 1;
							img_addr_w[5] = addr1_reset_w + 1;
							img_addr_w[6] = addr2_reset_w + 1;
						end
						3'd5: begin
							img_addr_w[5] = addr0_reset_w + 1;
							img_addr_w[6] = addr1_reset_w + 1;
							img_addr_w[7] = addr2_reset_w + 1;
						end
						3'd6: begin
							img_addr_w[6] = addr0_reset_w + 1;
							img_addr_w[7] = addr1_reset_w + 1;
							img_addr_w[0] = addr2_reset_w + 1;
						end
						3'd7: begin
							img_addr_w[7] = addr0_reset_w + 1;
							img_addr_w[0] = addr1_reset_w + 1;
							img_addr_w[1] = addr2_reset_w + 1;
						end
					endcase
				end
				else begin
					// Continue vertically in same column
					row_cnt_w = row_cnt_r + 1;
					startup_delay_w = 2;
					
					// Shift window upward
					mul_w[0] = mul_r[3];
					mul_w[1] = mul_r[4];
					mul_w[2] = mul_r[5];
					mul_w[3] = mul_r[6];
					mul_w[4] = mul_r[7];
					mul_w[5] = mul_r[8];
					
					// Load new bottom row from SRAM with edge detection
					if (pad_bottom) begin
						// Bottom edge - all zeros
						mul_w[6] = 8'd0;
						mul_w[7] = 8'd0;
						mul_w[8] = 8'd0;
					end
					else begin
						// Not at bottom edge - extract with left/right edge detection
						mul_w[6] = pad_left  ? 8'd0 : extract_with_pad(win_col_left);
						mul_w[7] = sram_packed_w[bit_index_w +: 8];
						mul_w[8] = pad_right ? 8'd0 : extract_with_pad(win_col_right);
					end
					
					// Increment SRAM addresses for next row
					case (col_cnt_r[2:0])
						3'd0: begin
							img_addr_w[7] = (col_cnt_r == 0) ? 0 : img_addr_r[7] + 1;
							img_addr_w[0] = img_addr_r[0] + 1;
							img_addr_w[1] = img_addr_r[1] + 1;
						end
						3'd1: begin
							img_addr_w[0] = img_addr_r[0] + 1;
							img_addr_w[1] = img_addr_r[1] + 1;
							img_addr_w[2] = img_addr_r[2] + 1;
						end
						3'd2: begin
							img_addr_w[1] = img_addr_r[1] + 1;
							img_addr_w[2] = img_addr_r[2] + 1;
							img_addr_w[3] = img_addr_r[3] + 1;
						end
						3'd3: begin
							img_addr_w[2] = img_addr_r[2] + 1;
							img_addr_w[3] = img_addr_r[3] + 1;
							img_addr_w[4] = img_addr_r[4] + 1;
						end
						3'd4: begin
							img_addr_w[3] = img_addr_r[3] + 1;
							img_addr_w[4] = img_addr_r[4] + 1;
							img_addr_w[5] = img_addr_r[5] + 1;
						end
						3'd5: begin
							img_addr_w[4] = img_addr_r[4] + 1;
							img_addr_w[5] = img_addr_r[5] + 1;
							img_addr_w[6] = img_addr_r[6] + 1;
						end
						3'd6: begin
							img_addr_w[5] = img_addr_r[5] + 1;
							img_addr_w[6] = img_addr_r[6] + 1;
							img_addr_w[7] = img_addr_r[7] + 1;
						end
						3'd7: begin
							img_addr_w[6] = img_addr_r[6] + 1;
							img_addr_w[7] = img_addr_r[7] + 1;
							img_addr_w[0] = img_addr_r[0] + 1;
						end
					endcase
				end
			end

			S_CONV11_BUFF: begin
				if (conv_done_w) begin
					next_state_w = S_DONE;
				end
				else begin
					next_state_w = S_CONV11_INIT;
					// Move to next column
					row_cnt_w = 6'd0;
					col_cnt_w = col_cnt_r + 1;
					next_state_w = S_CONV11_INIT;
					startup_delay_w = 0;

					mul_w[0] = 8'd0;
					mul_w[1] = 8'd0;
					mul_w[2] = 8'd0;
					
					// Middle row (row 0) - extract with edge detection
					mul_w[3] = buff1_r;
					mul_w[4] = buff2_r;
					mul_w[5] = extract_with_pad(win_next_col_right);
					buff1_w = buff2_r;
					buff2_w = extract_with_pad(win_next_col_right);
					
					// Bottom row - zeros (will load in S_CONV11_INIT cycle)
					mul_w[6] = 8'd0;
					mul_w[7] = 8'd0;
					mul_w[8] = 8'd0;
					case (col_cnt_r[2:0])
						3'd0: begin
							img_addr_w[0] = img_addr_r[0] + 1;
							img_addr_w[1] = img_addr_r[1] + 1;
							img_addr_w[2] = img_addr_r[2] + 1;
						end
						3'd1: begin
							img_addr_w[1] = img_addr_r[1] + 1;
							img_addr_w[2] = img_addr_r[2] + 1;
							img_addr_w[3] = img_addr_r[3] + 1;
						end
						3'd2: begin
							img_addr_w[2] = img_addr_r[2] + 1;
							img_addr_w[3] = img_addr_r[3] + 1;
							img_addr_w[4] = img_addr_r[4] + 1;
						end
						3'd3: begin
							img_addr_w[3] = img_addr_r[3] + 1;
							img_addr_w[4] = img_addr_r[4] + 1;
							img_addr_w[5] = img_addr_r[5] + 1;
						end
						3'd4: begin
							img_addr_w[4] = img_addr_r[4] + 1;
							img_addr_w[5] = img_addr_r[5] + 1;
							img_addr_w[6] = img_addr_r[6] + 1;
						end
						3'd5: begin
							img_addr_w[5] = img_addr_r[5] + 1;
							img_addr_w[6] = img_addr_r[6] + 1;
							img_addr_w[7] = img_addr_r[7] + 1;
						end
						3'd6: begin
							img_addr_w[6] = img_addr_r[6] + 1;
							img_addr_w[7] = img_addr_r[7] + 1;
							img_addr_w[0] = img_addr_r[0] + 1;
						end
						3'd7: begin
							img_addr_w[7] = img_addr_r[7] + 1;
							img_addr_w[0] = img_addr_r[0] + 1;
							img_addr_w[1] = img_addr_r[1] + 1;
						end
					endcase
				end
			end
			
			S_CONV12_INIT: begin
				next_state_w = S_CONV12;
				three_cnt_w = 1;

				mul_w[6] = extract_with_pad(win_col_left);
				mul_w[7] = sram_packed_w[bit_index_w +: 8];
				mul_w[8] = extract_with_pad(win_col_right);
				if (row_cnt_r <= 1) begin
					
				end
				else begin
					case (col_cnt_r[2:0])
						0: begin
							img_addr_w[6] = (col_cnt_r == 0) ? 0 : img_addr_r[6] + 2;
							img_addr_w[0] = img_addr_r[0] + 2;
							img_addr_w[2] = img_addr_r[2] + 2;
						end 
						1: begin
							img_addr_w[7] = (col_cnt_r == 0) ? 0 : img_addr_r[7] + 2;
							img_addr_w[1] = img_addr_r[1] + 2;
							img_addr_w[3] = img_addr_r[3] + 2;
						end 
						2: begin
							img_addr_w[0] = img_addr_r[0] + 2;
							img_addr_w[2] = img_addr_r[2] + 2;
							img_addr_w[4] = img_addr_r[4] + 2;
						end 
						3: begin
							img_addr_w[1] = img_addr_r[1] + 2;
							img_addr_w[3] = img_addr_r[3] + 2;
							img_addr_w[5] = img_addr_r[5] + 2;
						end 
						4: begin
							img_addr_w[2] = img_addr_r[2] + 2;
							img_addr_w[4] = img_addr_r[4] + 2;
							img_addr_w[6] = img_addr_r[6] + 2;
						end 
						5: begin
							img_addr_w[3] = img_addr_r[3] + 2;
							img_addr_w[5] = img_addr_r[5] + 2;
							img_addr_w[7] = img_addr_r[7] + 2;
						end 
						6: begin
							img_addr_w[4] = img_addr_r[4] + 2;
							img_addr_w[6] = img_addr_r[6] + 2;
							img_addr_w[0] = img_addr_r[0] + 2;
						end 
						7: begin
							img_addr_w[5] = img_addr_r[5] + 2;
							img_addr_w[7] = img_addr_r[7] + 2;
							img_addr_w[1] = img_addr_r[1] + 2;
						end
					endcase
				end
				
				
			end

			S_CONV12: begin
				three_cnt_w = (three_cnt_r == 2) ? 0 : three_cnt_r + 1;

				case (three_cnt_r)
					0: begin // output here
						next_state_w = S_CONV12;
						if (pad_bottom) begin
							// Bottom edge - all zeros
							mul_w[6] = 8'd0;
							mul_w[7] = 8'd0;
							mul_w[8] = 8'd0;
						end
						else begin
							// Not at bottom edge - extract with left/right edge detection
							mul_w[6] = extract_with_pad(win_col_left);
							mul_w[7] = sram_packed_w[bit_index_w +: 8];
							mul_w[8] = extract_with_pad(win_col_right);
						end
						if (row_cnt_r <= 1) begin
					
						end
						else begin
							case (col_cnt_r[2:0])
								0: begin
									img_addr_w[6] = (col_cnt_r == 0) ? 0 : img_addr_r[6] + 2;
									img_addr_w[0] = img_addr_r[0] + 2;
									img_addr_w[2] = img_addr_r[2] + 2;
								end 
								1: begin
									img_addr_w[7] = (col_cnt_r == 0) ? 0 : img_addr_r[7] + 2;
									img_addr_w[1] = img_addr_r[1] + 2;
									img_addr_w[3] = img_addr_r[3] + 2;
								end 
								2: begin
									img_addr_w[0] = img_addr_r[0] + 2;
									img_addr_w[2] = img_addr_r[2] + 2;
									img_addr_w[4] = img_addr_r[4] + 2;
								end 
								3: begin
									img_addr_w[1] = img_addr_r[1] + 2;
									img_addr_w[3] = img_addr_r[3] + 2;
									img_addr_w[5] = img_addr_r[5] + 2;
								end 
								4: begin
									img_addr_w[2] = img_addr_r[2] + 2;
									img_addr_w[4] = img_addr_r[4] + 2;
									img_addr_w[6] = img_addr_r[6] + 2;
								end 
								5: begin
									img_addr_w[3] = img_addr_r[3] + 2;
									img_addr_w[5] = img_addr_r[5] + 2;
									img_addr_w[7] = img_addr_r[7] + 2;
								end 
								6: begin
									img_addr_w[4] = img_addr_r[4] + 2;
									img_addr_w[6] = img_addr_r[6] + 2;
									img_addr_w[0] = img_addr_r[0] + 2;
								end 
								7: begin
									img_addr_w[5] = img_addr_r[5] + 2;
									img_addr_w[7] = img_addr_r[7] + 2;
									img_addr_w[1] = img_addr_r[1] + 2;
								end
							endcase
						end
					end
					1: begin
						next_state_w = S_CONV12;
						mul_w[0] = (row_cnt_r <= 1) ? 0 : extract_with_pad(win_col_left);
						mul_w[1] = (row_cnt_r <= 1) ? 0 : sram_packed_w[bit_index_w +: 8];
						mul_w[2] = (row_cnt_r <= 1) ? 0 : extract_with_pad(win_col_right);
						case (col_cnt_r[2:0])
							0: begin
								img_addr_w[6] = (col_cnt_r == 0) ? 0 : img_addr_r[6] + 2;
								img_addr_w[0] = img_addr_r[0] + 2;
								img_addr_w[2] = img_addr_r[2] + 2;
							end 
							1: begin
								img_addr_w[7] = (col_cnt_r == 0) ? 0 : img_addr_r[7] + 2;
								img_addr_w[1] = img_addr_r[1] + 2;
								img_addr_w[3] = img_addr_r[3] + 2;
							end 
							2: begin
								img_addr_w[0] = img_addr_r[0] + 2;
								img_addr_w[2] = img_addr_r[2] + 2;
								img_addr_w[4] = img_addr_r[4] + 2;
							end 
							3: begin
								img_addr_w[1] = img_addr_r[1] + 2;
								img_addr_w[3] = img_addr_r[3] + 2;
								img_addr_w[5] = img_addr_r[5] + 2;
							end 
							4: begin
								img_addr_w[2] = img_addr_r[2] + 2;
								img_addr_w[4] = img_addr_r[4] + 2;
								img_addr_w[6] = img_addr_r[6] + 2;
							end 
							5: begin
								img_addr_w[3] = img_addr_r[3] + 2;
								img_addr_w[5] = img_addr_r[5] + 2;
								img_addr_w[7] = img_addr_r[7] + 2;
							end 
							6: begin
								img_addr_w[4] = img_addr_r[4] + 2;
								img_addr_w[6] = img_addr_r[6] + 2;
								img_addr_w[0] = img_addr_r[0] + 2;
							end 
							7: begin
								img_addr_w[5] = img_addr_r[5] + 2;
								img_addr_w[7] = img_addr_r[7] + 2;
								img_addr_w[1] = img_addr_r[1] + 2;
							end
						endcase
					end
					2: begin
						mul_w[3] = extract_with_pad(win_col_left);
						mul_w[4] = sram_packed_w[bit_index_w +: 8];
						mul_w[5] = extract_with_pad(win_col_right);
						if (row_cnt_r != 63) begin
							next_state_w = S_CONV12;
							row_cnt_w = row_cnt_r + 1;
							case (col_cnt_r[2:0])
								0: begin
									img_addr_w[6] = (col_cnt_r == 0) ? 0 : (row_cnt_r == 0) ? addrf_reset_w + 1 : img_addr_r[6] - 3;
									img_addr_w[0] = (row_cnt_r == 0) ? addr0_reset_w + 1 : img_addr_r[0] - 3;
									img_addr_w[2] = (row_cnt_r == 0) ? addr1_reset_w + 1 : img_addr_r[2] - 3;
								end 
								1: begin
									img_addr_w[7] = (row_cnt_r == 0) ? addrf_reset_w + 1 : img_addr_r[7] - 3;
									img_addr_w[1] = (row_cnt_r == 0) ? addr0_reset_w + 1 : img_addr_r[1] - 3;
									img_addr_w[3] = (row_cnt_r == 0) ? addr1_reset_w + 1 : img_addr_r[3] - 3;
								end 
								2: begin
									img_addr_w[0] = (row_cnt_r == 0) ? addrf_reset_w + 1 : img_addr_r[0] - 3;
									img_addr_w[2] = (row_cnt_r == 0) ? addr0_reset_w + 1 : img_addr_r[2] - 3;
									img_addr_w[4] = (row_cnt_r == 0) ? addr1_reset_w + 1 : img_addr_r[4] - 3;
								end 
								3: begin
									img_addr_w[1] = (row_cnt_r == 0) ? addrf_reset_w + 1 : img_addr_r[1] - 3;
									img_addr_w[3] = (row_cnt_r == 0) ? addr0_reset_w + 1 : img_addr_r[3] - 3;
									img_addr_w[5] = (row_cnt_r == 0) ? addr1_reset_w + 1 : img_addr_r[5] - 3;
								end 
								4: begin
									img_addr_w[2] = (row_cnt_r == 0) ? addrf_reset_w + 1 : img_addr_r[2] - 3;
									img_addr_w[4] = (row_cnt_r == 0) ? addr0_reset_w + 1 : img_addr_r[4] - 3;
									img_addr_w[6] = (row_cnt_r == 0) ? addr1_reset_w + 1 : img_addr_r[6] - 3;
								end 
								5: begin
									img_addr_w[3] = (row_cnt_r == 0) ? addrf_reset_w + 1 : img_addr_r[3] - 3;
									img_addr_w[5] = (row_cnt_r == 0) ? addr0_reset_w + 1 : img_addr_r[5] - 3;
									img_addr_w[7] = (row_cnt_r == 0) ? addr1_reset_w + 1 : img_addr_r[7] - 3;
								end 
								6: begin
									img_addr_w[4] = (row_cnt_r == 0) ? addrf_reset_w + 1 : img_addr_r[4] - 3;
									img_addr_w[6] = (row_cnt_r == 0) ? addr0_reset_w + 1 : img_addr_r[6] - 3;
									img_addr_w[0] = (row_cnt_r == 0) ? addr1_reset_w + 1 : img_addr_r[0] - 3;
								end 
								7: begin
									img_addr_w[5] = (row_cnt_r == 0) ? addrf_reset_w + 1 : img_addr_r[5] - 3;
									img_addr_w[7] = (row_cnt_r == 0) ? addr0_reset_w + 1 : img_addr_r[7] - 3;
									img_addr_w[1] = (row_cnt_r == 0) ? addr1_reset_w + 1 : img_addr_r[1] - 3;
								end
							endcase
						end
						else begin
							row_cnt_w = 0;
							col_cnt_w = col_cnt_r + 1;
							if (startup_delay_r) next_state_w = S_DONE;
							else if (conv_done_w) begin
								startup_delay_w = 1;
								next_state_w = S_CONV12_BUFF;
							end
							else next_state_w = S_CONV12_BUFF;
							case (col_cnt_r[2:0])
								0: begin
									img_addr_w[6] = (col_cnt_r == 0) ? 0 : (col_cnt_r[5:3]) << 6;
									img_addr_w[0] = addr0_reset_w;
									img_addr_w[2] = addr1_reset_w;
								end 
								1: begin
									img_addr_w[7] = (col_cnt_r == 1) ? 0 : (col_cnt_r[5:3]) << 6;
									img_addr_w[1] = addr0_reset_w;
									img_addr_w[3] = addr1_reset_w; 
								end
								2: begin
									img_addr_w[0] = (col_cnt_r[5:3] + 1) << 6;
									img_addr_w[2] = addr0_reset_w;
									img_addr_w[4] = addr1_reset_w; 
								end
								3: begin
									img_addr_w[1] = (col_cnt_r[5:3] + 1) << 6;
									img_addr_w[3] = addr0_reset_w;
									img_addr_w[5] = addr1_reset_w;
								end 
								4: begin
									img_addr_w[2] = (col_cnt_r[5:3] + 1) << 6;
									img_addr_w[4] = addr0_reset_w;
									img_addr_w[6] = addr1_reset_w; 
								end
								5: begin
									img_addr_w[3] = (col_cnt_r[5:3] + 1) << 6;
									img_addr_w[5] = addr0_reset_w;
									img_addr_w[7] = addr1_reset_w; 
								end
								6: begin
									img_addr_w[4] = (col_cnt_r[5:3] + 1) << 6;
									img_addr_w[6] = addr0_reset_w;
									img_addr_w[0] = addr1_reset_w; 
								end
								7: begin
									img_addr_w[5] = (col_cnt_r[5:3] + 1) << 6;
									img_addr_w[7] = addr0_reset_w;
									img_addr_w[1] = addr1_reset_w; 
								end
							endcase
						end
					end
				endcase
		
			end

			S_CONV12_BUFF: begin
				three_cnt_w = 1;
				next_state_w = S_CONV12;
				mul_w[0] = 0;
				mul_w[1] = 0;
				mul_w[2] = 0;
				mul_w[3] = extract_with_pad(win_col_right);
				mul_w[4] = sram_packed_w[bit_index_w +: 8];
				mul_w[5] = extract_with_pad(win_col_right);
			end

			S_DONE: begin
				next_state_w = S_DONE;
			end
			
			default: begin
				next_state_w = S_IDLE;
			end
		endcase
	end

	// ============================================================================
	// SEQUENTIAL: FSM State Register
	// ============================================================================
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			state_r <= S_IDLE;
			prev_state_r <= S_IDLE;
		end
		else begin
			prev_state_r <= state_r;
			state_r <= next_state_w;
		end
	end

	// ============================================================================
	// SEQUENTIAL: Convolution Counter and Pixel Window Updates
	// ============================================================================
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			buff1_r <= 0;
			buff2_r <= 0;
			row_cnt_r <= 6'd0;
			col_cnt_r <= 6'd0;
			for (i = 0; i < 9; i = i + 1)
				mul_r[i] <= 8'd0;
		end
		else begin
			// Update from combinational logic
			buff1_r <= buff1_w;
			buff2_r <= buff2_w;
			row_cnt_r <= row_cnt_w;
			col_cnt_r <= col_cnt_w;
			for (i = 0; i < 9; i = i + 1)
				mul_r[i] <= mul_w[i];
		end
	end

	// ============================================================================
	// SEQUENTIAL: Barcode Decoding State Machine
	// ============================================================================
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			shift_lsb_r <= 13'd0;
			pass_cnt_r <= 7'd0;
			seq_cnt_r <= 7'd0;
			hei_cnt_r <= 4'd0;
			index_r <= 9'd0;
			seq_r <= 57'd0;
			curr_seq_r <= 57'd0;
			ker_r <= 2'd0;
			str_r <= 2'd0;
			dil_r <= 2'd0;
			start_r <= 1'b0;
			row_start_r <= 1'b0;
			barcode_done_r <= 1'b0;
		end
		else if (state_r == S_IDLE && !i_in_valid) begin
			// Reset barcode decoding on new image
			shift_lsb_r <= 13'd0;
			pass_cnt_r <= 7'd0;
			seq_cnt_r <= 7'd0;
			hei_cnt_r <= 4'd0;
			index_r <= 9'd0;
			seq_r <= 57'd0;
			curr_seq_r <= 57'd0;
			start_r <= 1'b0;
			row_start_r <= 1'b0;
			barcode_done_r <= 1'b0;
		end
		else if ((state_r == S_IMG || state_r == S_IDLE) && i_in_valid && !barcode_done_r) begin
			// Update shift register and pass counter
			shift_lsb_r <= shift_lsb_next_w;
			pass_cnt_r <= pass_cnt_next_w;
			
			if (start_r) begin
				// Already found potential barcode start, now validating height
				
				// At the beginning of each row
				if (!row_start_r && hei_cnt_r > 0) begin
					if (shift_lsb_r[10:0] == START_CODE && pass_cnt_r - 9'd10 == index_r) begin
						seq_cnt_r <= 7'd15;
						row_start_r <= 1'b1;
						curr_seq_r <= {42'b0, shift_lsb_r[10:0], lsb_w};
					end
					else if (shift_lsb_r[11:1] == START_CODE && pass_cnt_r - 9'd11 == index_r) begin
						seq_cnt_r <= 7'd16;
						row_start_r <= 1'b1;
						curr_seq_r <= {41'b0, shift_lsb_r[11:0], lsb_w};
					end
					else if (shift_lsb_r[12:2] == START_CODE && pass_cnt_r - 9'd12 == index_r) begin
						seq_cnt_r <= 7'd17;
						row_start_r <= 1'b1;
						curr_seq_r <= {40'b0, shift_lsb_r[12:0], lsb_w};
					end
				end
				
				// Collect sequence bits in current row
				else if (row_start_r && seq_cnt_r < 53) begin
					if (hei_cnt_r == 0) begin
						seq_r <= {curr_seq_r[52:0], lsb_w};
					end
					curr_seq_r <= {curr_seq_r[52:0], lsb_w};
					seq_cnt_r <= seq_cnt_r + 7'd4;
				end
				
				// At end of row
				else if (seq_cnt_r > 52 && row_start_r) begin
					if (hei_cnt_r == 4'd0) begin
						// First row - check if there's stop code
						if (not_stop_w) begin
							// Invalid - reset
							start_r <= 1'b0;
							hei_cnt_r <= 4'd0;
							row_start_r <= 1'b0;
							index_r <= 9'd0;
							seq_cnt_r <= 7'd0;
							seq_r <= 57'd0;
							curr_seq_r <= 57'd0;
						end
						else begin
							case (seq_cnt_r)
								53: seq_r <= {curr_seq_r[52:0], lsb_w};
								54: seq_r <= {curr_seq_r[53:0], lsb_w[3:1]};
								55: seq_r <= {curr_seq_r[54:0], lsb_w[3:2]};
								56: seq_r <= {curr_seq_r[55:0], lsb_w[3]};
							endcase
							hei_cnt_r <= 4'd1;
							seq_cnt_r <= 7'd0;
							row_start_r <= 1'b0;
						end
					end
					else if (hei_cnt_r < 4'd9) begin
						// Rows 2-9: verify they match first row
						if (is_match_seq_w) begin
							hei_cnt_r <= hei_cnt_r + 4'd1;
							seq_cnt_r <= 7'd0;
							row_start_r <= 1'b0;
						end
						else begin
							// Mismatch - reset
							start_r <= 1'b0;
							hei_cnt_r <= 4'd0;
							row_start_r <= 1'b0;
							index_r <= 9'd0;
							seq_cnt_r <= 7'd0;
							seq_r <= 57'd0;
						end
					end
					else if (hei_cnt_r == 4'd9) begin
						// 10th row: final validation and decode
						if (is_match_seq_w) begin
							// Valid 10-row barcode found!
							if (seq_r[56:46] == START_CODE && seq_r[12:0] == STOP_CODE) begin
								// Decode parameters
								ker_r <= decoded_input(seq_r[45:35]);
								str_r <= decoded_input(seq_r[34:24]);
								dil_r <= decoded_input(seq_r[23:13]);
								
								// Validate: K must be 3, S and D must be non-zero
								if (decoded_input(seq_r[45:35]) != 2'd3 || 
								    decoded_input(seq_r[34:24]) == 2'd0 || 
								    decoded_input(seq_r[23:13]) == 2'd0) begin
									ker_r <= 2'd0;
									str_r <= 2'd0;
									dil_r <= 2'd0;
								end
							end
							else begin
								// Invalid START/STOP
								ker_r <= 2'd0;
								str_r <= 2'd0;
								dil_r <= 2'd0;
							end
							barcode_done_r <= 1'b1;
						end
						else begin
							// Mismatch on 10th row - reset
							start_r <= 1'b0;
							hei_cnt_r <= 4'd0;
							row_start_r <= 1'b0;
							index_r <= 9'd0;
							seq_cnt_r <= 7'd0;
							seq_r <= 57'd0;
						end
					end
				end
			end
			else begin
				// Search for START_CODE in shift register
				if (shift_lsb_r[10:0] == START_CODE) begin
					start_r <= 1'b1;
					seq_cnt_r <= 7'd15;
					seq_r <= {42'b0, shift_lsb_r[10:0], lsb_w};
					index_r <= pass_cnt_r - 9'd10;
					row_start_r <= 1'b1;
					hei_cnt_r <= 4'd0;
					curr_seq_r <= {42'b0, shift_lsb_r[10:0], lsb_w};
				end
				else if (shift_lsb_r[11:1] == START_CODE) begin
					start_r <= 1'b1;
					seq_cnt_r <= 7'd16;
					seq_r <= {41'b0, shift_lsb_r[11:0], lsb_w};
					index_r <= pass_cnt_r - 9'd11;
					row_start_r <= 1'b1;
					hei_cnt_r <= 4'd0;
					curr_seq_r <= {41'b0, shift_lsb_r[11:0], lsb_w};
				end
				else if (shift_lsb_r[12:2] == START_CODE) begin
					start_r <= 1'b1;
					seq_cnt_r <= 7'd17;
					seq_r <= {40'b0, shift_lsb_r[12:0], lsb_w};
					index_r <= pass_cnt_r - 9'd12;
					row_start_r <= 1'b1;
					hei_cnt_r <= 4'd0;
					curr_seq_r <= {40'b0, shift_lsb_r[12:0], lsb_w};
				end
			end
		end
	end

	// ============================================================================
	// SEQUENTIAL: Image Loading
	// ============================================================================
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			flip_r <= 1'b0;
			img_done_r <= 1'b0;
			for (i = 0; i < 8; i = i + 1) begin
				wen_r[i] <= 1'b1;
				img_addr_r[i] <= 9'd0;
				img_in_r[i] <= 8'd0;
			end
			img_addr_inc_r <= 0;
			img_addr_offset_r <= 0;
		end
		else if (state_r == S_IDLE) begin
			img_done_r <= 1'b0;
			for (i = 0; i < 8; i = i + 1) begin
				wen_r[i] <= 1'b1;
				img_addr_r[i] <= 9'd0;
			end
			// Load first 4 pixels on transition to IMG state
			if (i_in_valid) begin
				img_in_r[0] <= i_in_data[31:24];
				img_in_r[1] <= i_in_data[23:16];
				img_in_r[2] <= i_in_data[15:8];
				img_in_r[3] <= i_in_data[7:0];
				for (i = 0; i < 4; i = i + 1)
					wen_r[i] <= 1'b0;
				flip_r <= 1'b1;
				img_addr_inc_r <= img_addr_inc_w;
				img_addr_offset_r <= img_addr_offset_w;
			end
		end
		else if (state_r == S_IMG && i_in_valid) begin
			img_addr_inc_r <= img_addr_inc_w;
			img_addr_offset_r <= img_addr_offset_w;
			if (!flip_r) begin
				// Write to SRAM 0-3, load data for SRAM 4-7
				for (i = 0; i < 4; i = i + 1)
					wen_r[i] <= 1'b0;
				for (i = 4; i < 8; i = i + 1)
					wen_r[i] <= 1'b1;
				for (i = 0; i < 8; i = i + 1)
					img_addr_r[i] <= img_addr_offset_r + img_addr_col_w;
				img_in_r[0] <= i_in_data[31:24];
				img_in_r[1] <= i_in_data[23:16];
				img_in_r[2] <= i_in_data[15:8];
				img_in_r[3] <= i_in_data[7:0];
				flip_r <= 1'b1;
			end
			else begin
				// Write to SRAM 4-7, load data for SRAM 0-3
				img_in_r[4] <= i_in_data[31:24];
				img_in_r[5] <= i_in_data[23:16];
				img_in_r[6] <= i_in_data[15:8];
				img_in_r[7] <= i_in_data[7:0];
				for (i = 0; i < 4; i = i + 1)
					wen_r[i] <= 1'b1;
				for (i = 4; i < 8; i = i + 1)
					wen_r[i] <= 1'b0;
				flip_r <= 1'b0;
			end
			
			// Mark done when last address reached
			if (img_addr_r[0] == 9'd511 && flip_r == 1'b1)
				img_done_r <= 1'b1;
		end
		else if (state_r == S_BUFF) begin
			// Disable writing after image loading complete
			for (i = 4; i < 8; i = i + 1)
				wen_r[i] <= 1'b1;
		end
		else begin
			for (i = 0; i < 8; i = i + 1) img_addr_r[i] <= img_addr_w[i];
		end
	end

	// ============================================================================
	// SEQUENTIAL: Weight Loading
	// ============================================================================
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			wei_cnt_r <= 2'd0;
			for (i = 0; i < 9; i = i + 1)
				weight_r[i] <= 8'sd0;
		end
		else if (state_r == S_WEI && i_in_valid) begin
			wei_cnt_r <= wei_cnt_next_w;
			if (wei_cnt_r == 2'd0) begin
				weight_r[0] <= i_in_data[31:24];
				weight_r[1] <= i_in_data[23:16];
				weight_r[2] <= i_in_data[15:8];
				weight_r[3] <= i_in_data[7:0];
			end
			else if (wei_cnt_r == 2'd1) begin
				weight_r[4] <= i_in_data[31:24];
				weight_r[5] <= i_in_data[23:16];
				weight_r[6] <= i_in_data[15:8];
				weight_r[7] <= i_in_data[7:0];
			end
			else begin  // wei_cnt_r == 2
				weight_r[8] <= i_in_data[31:24];
			end
		end
	end

	// ============================================================================
	// SEQUENTIAL: Output Port Updates
	// ============================================================================
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			o_in_ready_r <= 1'b0;
			o_out_valid1_r <= 1'b0;
			o_out_valid2_r <= 1'b0;
			o_out_valid3_r <= 1'b0;
			o_out_valid4_r <= 1'b0;
			o_out_data1_r <= 8'd0;
			o_out_data2_r <= 8'd0;
			o_out_data3_r <= 8'd0;
			o_out_data4_r <= 8'd0;
			o_out_addr1_r <= 12'd0;
			exe_finish_r <= 1'b0;
			startup_delay_r <= 3'd0;
			three_cnt_r <= 2'd0;
		end
		else begin
			three_cnt_r <= three_cnt_w;
			startup_delay_r <= startup_delay_w;
			case (state_r)
				S_IDLE: begin
					o_in_ready_r <= 1'b1;
					o_out_valid1_r <= 1'b0;
					o_out_valid2_r <= 1'b0;
					o_out_valid3_r <= 1'b0;
					o_out_valid4_r <= 1'b0;
					exe_finish_r <= 1'b0;
				end
				S_IMG: begin
					o_in_ready_r <= 1'b1;
					// Output barcode decode results when image loading complete
					if (img_addr_r[0] == 9'd511 && flip_r == 1'b1) begin
						o_out_valid1_r <= 1'b1;
						o_out_valid2_r <= 1'b1;
						o_out_valid3_r <= 1'b1;
						o_out_data1_r <= {5'd0, ker_r};
						o_out_data2_r <= {5'd0, str_r};
						o_out_data3_r <= {5'd0, dil_r};
					end
				end
				S_BUFF: begin
					o_in_ready_r <= 1'b1;
					o_out_valid1_r <= 1'b0;
					o_out_valid2_r <= 1'b0;
					o_out_valid3_r <= 1'b0;
				end
				S_WEI: begin
					o_in_ready_r <= 1'b1;
				end
				S_CONV11_INIT: begin
					o_in_ready_r <= 1'b0;
					// startup_delay_r <= 3'd1;
				end
				S_CONV11: begin
					if (startup_delay_r >= 1) begin
						if (str_r == 1) begin
							o_out_valid1_r <= 1'b1;
							o_out_data1_r <= acc_clamped_w;
							o_out_addr1_r <= {row_dec_w, col_cnt_r};
						end
						else begin
							if (row_dec_w[0] || col_cnt_r[0]) begin
								o_out_valid1_r <= 0;
							end
							else begin
								o_out_valid1_r <= 1'b1;
								o_out_data1_r <= acc_clamped_w;
								o_out_addr1_r <= {2'b0, half_row_dec_w, half_col_w};
							end
						end
					end
					else begin
						
					end
				end
				S_CONV11_BUFF: begin
					if (str_r == 1) begin
						o_out_valid1_r <= 1'b1;
						o_out_data1_r <= acc_clamped_w;
						o_out_addr1_r <= {row_cnt_r, col_cnt_r};
					end
					else begin
						if (row_cnt_r[0] || col_cnt_r[0]) begin
							o_out_valid1_r <= 0;
						end
						else begin
							o_out_valid1_r <= 1'b1;
							o_out_data1_r <= acc_clamped_w;
							o_out_addr1_r <= {2'b0, half_row_w, half_col_w};
						end
					end
				end

				S_CONV12_INIT: begin

				end

				S_CONV12: begin
					if (three_cnt_r == 1 && prev_state_r != S_CONV12_BUFF) begin
						if (str_r == 1) begin
							o_out_valid1_r <= 1;
							o_out_data1_r <= acc_clamped_w;
							o_out_addr1_r <= {row_dec_w, col_cnt_r};
						end
						else begin
							if (row_dec_w[0] || col_cnt_r[0]) begin
								o_out_valid1_r <= 0;
							end
							else begin
								o_out_valid1_r <= 1'b1;
								o_out_data1_r <= acc_clamped_w;
								o_out_addr1_r <= {2'b0, half_row_dec_w, half_col_w};
							end
						end
					end 
					else begin
						o_out_valid1_r <= 0;
					end
				end

				S_CONV12_BUFF: begin
					if (str_r == 1) begin
						o_out_valid1_r <= 1;
						o_out_data1_r <= acc_clamped_w;
						o_out_addr1_r <= {row_cnt_r - 6'd1, col_cnt_r - 6'd1};
					end
					else begin
						if (row_dec_w[0] || col_cnt_r[0]) begin
							o_out_valid1_r <= 0;
						end
						else begin
							o_out_valid1_r <= 1'b1;
							o_out_data1_r <= acc_clamped_w;
							o_out_addr1_r <= {2'b0, half_row_dec_w, half_col_dec_w};
						end
					end
				end

				S_DONE: begin
					o_in_ready_r <= 1'b0;
					o_out_valid1_r <= 1'b0;
					exe_finish_r <= 1'b1;
				end
			endcase
		end
	end

	// ============================================================================
	// Function: Decode Code 128-C to number
	// ============================================================================
	function automatic [1:0] decoded_input;
		input [10:0] i_data;
		begin
			case (i_data)
				ONE_CODE:   decoded_input = 2'd1;
				TWO_CODE:   decoded_input = 2'd2;
				THREE_CODE: decoded_input = 2'd3;
				default:    decoded_input = 2'd0;
			endcase
		end
	endfunction

endmodule