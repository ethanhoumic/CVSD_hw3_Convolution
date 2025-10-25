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

	// Note: Other output addresses (addr2-4) missing in original code - add if needed
	assign o_out_addr2 = 12'd0;  // TODO: Add proper logic
	assign o_out_addr3 = 12'd0;
	assign o_out_addr4 = 12'd0;

	// ============================================================================
	// FSM States
	// ============================================================================
	localparam S_IDLE      = 3'b000;
	localparam S_IMG       = 3'b001;
	localparam S_WEI       = 3'b010;
	localparam S_CONV      = 3'b011;
	localparam S_BUFF      = 3'b100;
	localparam S_CONV_INIT = 3'b101;
	localparam S_DONE      = 3'b110;

	reg [2:0] state_r, next_state_w;

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
	reg [2:0]  ker_r, str_r, dil_r;
	reg        start_r, row_start_r, barcode_done_r;

	// Barcode Decoding (Combinational Wires)
	wire [3:0]  lsb_w;
	wire [12:0] shift_lsb_next_w;
	wire [6:0]  pass_cnt_next_w;
	wire [2:0]  ker_decoded_w, str_decoded_w, dil_decoded_w;
	wire        barcode_valid_w;

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

	// Image Storage (Combinational Wires)
	wire [7:0] img_out_w  [0:7];
	
	integer i;
	genvar j;

	// ============================================================================
	// Convolution (Sequential Registers)
	// ============================================================================
	reg [5:0]  row_cnt_r, col_cnt_r;
	reg [1:0]  mac_stage_r;
	reg signed [19:0] acc0_r, acc1_r, acc2_r;
	reg [7:0]  mul_r [0:8];
	reg [7:0]  output_data_r;
	reg [11:0] output_addr_r;
	reg        output_valid_r;
	reg [2:0]  startup_delay_r;

	// Convolution (Combinational Wires)
	wire signed [15:0] prod [0:8];
	wire signed [17:0] sum_row0, sum_row1, sum_row2;
	wire signed [19:0] acc_sum, acc_rounded;
	wire signed [12:0] acc_shifted;
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
	assign sum_row0 = prod[0] + prod[1] + prod[2];
	assign sum_row1 = prod[3] + prod[4] + prod[5];
	assign sum_row2 = prod[6] + prod[7] + prod[8];

	// Accumulation, rounding, clamping
	assign acc_sum     = acc0_r + acc1_r + acc2_r;
	assign acc_rounded = acc_sum + 20'sd64;
	assign acc_shifted = acc_rounded >>> 7;
	assign acc_clamped_w = (acc_shifted < 13'sd0)   ? 8'd0 :
	                       (acc_shifted > 13'sd255) ? 8'd255 :
	                       acc_shifted[7:0];

	// Pack SRAM outputs
	assign sram_packed_w = {img_out_w[7], img_out_w[6], img_out_w[5], img_out_w[4],
	                        img_out_w[3], img_out_w[2], img_out_w[1], img_out_w[0]};

	// Boundary detection
	wire dil1_inc_col_w = (dil_r == 2'd1) && (row_cnt_r == 6'd63);
	wire dil2_inc_col_w = (dil_r == 2'd2) && (row_cnt_r == 6'd31);
	wire conv_done_w    = (dil1_inc_col_w && col_cnt_r == 6'd63) || 
	                      (dil2_inc_col_w && col_cnt_r == 6'd31);

	// Padding detection
	wire signed [7:0] win_col_left  = $signed({1'b0, col_cnt_r}) - $signed({6'b0, dil_r});
	wire signed [7:0] win_col_right = $signed({1'b0, col_cnt_r}) + $signed({6'b0, dil_r});
	wire signed [7:0] win_row_bot   = $signed({1'b0, row_cnt_r}) + $signed({6'b0, str_r}) + $signed({6'b0, dil_r});
	wire pad_left   = (win_col_left < 8'sd0);
	wire pad_right  = (win_col_right > 8'sd63);
	wire pad_bottom = (win_row_bot > 8'sd63);

	// ============================================================================
	// Function: Extract pixel with padding
	// ============================================================================
	function [7:0] extract_with_pad;
		input signed [7:0] col;
		begin
			if (col < 8'sd0 || col > 8'sd63) begin
				extract_with_pad = 8'd0;  // Zero padding
			end
			else begin
				extract_with_pad = sram_packed_w[(col[2:0] << 3) +: 8];
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
		
		for (i = 0; i < 8; i = i + 1) begin
			img_addr_w[i] = img_addr_r[i];
		end
		for (i = 0; i < 9; i = i + 1) begin
			mul_w[i] = mul_r[i];
		end
		
		case (state_r)
			S_IDLE: begin
				if (i_in_valid)
					next_state_w = S_IMG;
			end
			
			S_IMG: begin
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
				if (wei_cnt_r == 2'd2 && i_in_valid)
					next_state_w = S_CONV_INIT;
			end
			
			// ====================================================================
			// S_CONV_INIT: Initialize window for new column
			// ====================================================================
			S_CONV_INIT: begin
				next_state_w = S_CONV;
				row_cnt_w = 6'd0;
				
				// Top row - all zeros (padding)
				mul_w[0] = 8'd0;
				mul_w[1] = 8'd0;
				mul_w[2] = 8'd0;
				
				// Middle row (row 0) - extract with edge detection
				mul_w[3] = extract_with_pad(win_col_left);
				mul_w[4] = sram_packed_w[(col_cnt_r[2:0] << 3) +: 8];
				mul_w[5] = extract_with_pad(win_col_right);
				
				// Bottom row - zeros (will load in first S_CONV cycle)
				mul_w[6] = 8'd0;
				mul_w[7] = 8'd0;
				mul_w[8] = 8'd0;
				
				// Set SRAM addresses to read bottom row
				if (dil_r == 2'd1) begin
					// Dilation = 1, bottom row is row 1
					case (col_cnt_r[2:0])
						3'd0, 3'd1: begin
							img_addr_w[0] = 9'd1;
							img_addr_w[1] = 9'd1;
							img_addr_w[2] = 9'd1;
						end
						3'd2: begin
							img_addr_w[1] = 9'd1;
							img_addr_w[2] = 9'd1;
							img_addr_w[3] = 9'd1;
						end
						3'd3: begin
							img_addr_w[2] = 9'd1;
							img_addr_w[3] = 9'd1;
							img_addr_w[4] = 9'd1;
						end
						3'd4: begin
							img_addr_w[3] = 9'd1;
							img_addr_w[4] = 9'd1;
							img_addr_w[5] = 9'd1;
						end
						3'd5: begin
							img_addr_w[4] = 9'd1;
							img_addr_w[5] = 9'd1;
							img_addr_w[6] = 9'd1;
						end
						3'd6: begin
							img_addr_w[5] = 9'd1;
							img_addr_w[6] = 9'd1;
							img_addr_w[7] = 9'd1;
						end
						3'd7: begin
							img_addr_w[6] = 9'd1;
							img_addr_w[7] = 9'd1;
							img_addr_w[0] = 9'd1;
						end
					endcase
				end
				else begin
					// Dilation = 2, bottom row is row 2
					case (col_cnt_r[2:0])
						3'd0, 3'd2: begin
							img_addr_w[0] = 9'd2;
							img_addr_w[2] = 9'd2;
							img_addr_w[4] = 9'd2;
						end
						3'd1, 3'd3: begin
							img_addr_w[1] = 9'd2;
							img_addr_w[3] = 9'd2;
							img_addr_w[5] = 9'd2;
						end
						3'd4: begin
							img_addr_w[2] = 9'd2;
							img_addr_w[4] = 9'd2;
							img_addr_w[6] = 9'd2;
						end
						3'd5: begin
							img_addr_w[3] = 9'd2;
							img_addr_w[5] = 9'd2;
							img_addr_w[7] = 9'd2;
						end
						3'd6: begin
							img_addr_w[4] = 9'd2;
							img_addr_w[6] = 9'd2;
							img_addr_w[0] = 9'd2;
						end
						3'd7: begin
							img_addr_w[5] = 9'd2;
							img_addr_w[7] = 9'd2;
							img_addr_w[1] = 9'd2;
						end
					endcase
				end
			end
			
			// ====================================================================
			// S_CONV: Main convolution loop with edge case handling
			// ====================================================================
			S_CONV: begin
				if (conv_done_w) begin
					next_state_w = S_DONE;
				end
				else if (dil1_inc_col_w || dil2_inc_col_w) begin
					// Move to next column
					row_cnt_w = 6'd0;
					col_cnt_w = col_cnt_r + str_r;
					next_state_w = S_CONV_INIT;
				end
				else begin
					// Continue vertically in same column
					row_cnt_w = row_cnt_r + str_r;
					
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
						mul_w[7] = sram_packed_w[(col_cnt_r[2:0] << 3) +: 8];
						mul_w[8] = pad_right ? 8'd0 : extract_with_pad(win_col_right);
					end
					
					// Increment SRAM addresses for next row
					if (dil_r == 2'd1) begin
						// Dilation = 1
						case (col_cnt_r[2:0])
							3'd0, 3'd1: begin
								img_addr_w[0] = img_addr_r[0] + str_r;
								img_addr_w[1] = img_addr_r[1] + str_r;
								img_addr_w[2] = img_addr_r[2] + str_r;
							end
							3'd2: begin
								img_addr_w[1] = img_addr_r[1] + str_r;
								img_addr_w[2] = img_addr_r[2] + str_r;
								img_addr_w[3] = img_addr_r[3] + str_r;
							end
							3'd3: begin
								img_addr_w[2] = img_addr_r[2] + str_r;
								img_addr_w[3] = img_addr_r[3] + str_r;
								img_addr_w[4] = img_addr_r[4] + str_r;
							end
							3'd4: begin
								img_addr_w[3] = img_addr_r[3] + str_r;
								img_addr_w[4] = img_addr_r[4] + str_r;
								img_addr_w[5] = img_addr_r[5] + str_r;
							end
							3'd5: begin
								img_addr_w[4] = img_addr_r[4] + str_r;
								img_addr_w[5] = img_addr_r[5] + str_r;
								img_addr_w[6] = img_addr_r[6] + str_r;
							end
							3'd6: begin
								img_addr_w[5] = img_addr_r[5] + str_r;
								img_addr_w[6] = img_addr_r[6] + str_r;
								img_addr_w[7] = img_addr_r[7] + str_r;
							end
							3'd7: begin
								img_addr_w[6] = img_addr_r[6] + str_r;
								img_addr_w[7] = img_addr_r[7] + str_r;
								img_addr_w[0] = img_addr_r[0] + str_r;
							end
						endcase
					end
					else begin
						// Dilation = 2
						case (col_cnt_r[2:0])
							3'd0, 3'd2: begin
								img_addr_w[0] = img_addr_r[0] + str_r;
								img_addr_w[2] = img_addr_r[2] + str_r;
								img_addr_w[4] = img_addr_r[4] + str_r;
							end
							3'd1, 3'd3: begin
								img_addr_w[1] = img_addr_r[1] + str_r;
								img_addr_w[3] = img_addr_r[3] + str_r;
								img_addr_w[5] = img_addr_r[5] + str_r;
							end
							3'd4: begin
								img_addr_w[2] = img_addr_r[2] + str_r;
								img_addr_w[4] = img_addr_r[4] + str_r;
								img_addr_w[6] = img_addr_r[6] + str_r;
							end
							3'd5: begin
								img_addr_w[3] = img_addr_r[3] + str_r;
								img_addr_w[5] = img_addr_r[5] + str_r;
								img_addr_w[7] = img_addr_r[7] + str_r;
							end
							3'd6: begin
								img_addr_w[4] = img_addr_r[4] + str_r;
								img_addr_w[6] = img_addr_r[6] + str_r;
								img_addr_w[0] = img_addr_r[0] + str_r;
							end
							3'd7: begin
								img_addr_w[5] = img_addr_r[5] + str_r;
								img_addr_w[7] = img_addr_r[7] + str_r;
								img_addr_w[1] = img_addr_r[1] + str_r;
							end
						endcase
					end
				end
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
		if (!i_rst_n)
			state_r <= S_IDLE;
		else
			state_r <= next_state_w;
	end

	// ============================================================================
	// SEQUENTIAL: MAC Pipeline
	// ============================================================================
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			acc0_r <= 20'sd0;
			acc1_r <= 20'sd0;
			acc2_r <= 20'sd0;
			mac_stage_r <= 2'd0;
		end
		else if (state_r == S_CONV) begin
			case (mac_stage_r)
				2'd0: begin
					acc0_r <= sum_row0;
					mac_stage_r <= 2'd1;
				end
				2'd1: begin
					acc1_r <= sum_row1;
					mac_stage_r <= 2'd2;
				end
				2'd2: begin
					acc2_r <= sum_row2;
					mac_stage_r <= 2'd0;
				end
			endcase
		end
		else if (state_r == S_CONV_INIT) begin
			mac_stage_r <= 2'd0;
			acc0_r <= 20'sd0;
			acc1_r <= 20'sd0;
			acc2_r <= 20'sd0;
		end
	end

	// ============================================================================
	// SEQUENTIAL: Convolution Counter and Pixel Window Updates
	// ============================================================================
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			row_cnt_r <= 6'd0;
			col_cnt_r <= 6'd0;
			for (i = 0; i < 9; i = i + 1)
				mul_r[i] <= 8'd0;
		end
		else if (state_r == S_CONV_INIT || state_r == S_CONV) begin
			// Update from combinational logic
			row_cnt_r <= row_cnt_w;
			col_cnt_r <= col_cnt_w;
			for (i = 0; i < 9; i = i + 1)
				mul_r[i] <= mul_w[i];
		end
	end

	// ============================================================================
	// SEQUENTIAL: Output Logic
	// ============================================================================
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			output_data_r <= 8'd0;
			output_addr_r <= 12'd0;
			output_valid_r <= 1'b0;
			startup_delay_r <= 3'd0;
		end
		else if (state_r == S_CONV_INIT) begin
			startup_delay_r <= 3'd0;
			output_valid_r <= 1'b0;
		end
		else if (state_r == S_CONV) begin
			// Track startup delay
			if (startup_delay_r < 3'd3)
				startup_delay_r <= startup_delay_r + 3'd1;
			
			// Output when MAC stage 2 completes and after startup delay
			if (mac_stage_r == 2'd2 && startup_delay_r >= 3'd3) begin
				output_data_r <= acc_clamped_w;
				output_addr_r <= {row_cnt_r, col_cnt_r};
				output_valid_r <= 1'b1;
			end
			else begin
				output_valid_r <= 1'b0;
			end
		end
		else begin
			output_valid_r <= 1'b0;
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
			ker_r <= 3'd0;
			str_r <= 3'd0;
			dil_r <= 3'd0;
			start_r <= 1'b0;
			row_start_r <= 1'b0;
			barcode_done_r <= 1'b0;
		end
		else if (state_r == S_IDLE) begin
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
		else if (state_r == S_IMG && i_in_valid && !barcode_done_r) begin
			// Update shift register and pass counter
			shift_lsb_r <= shift_lsb_next_w;
			pass_cnt_r <= pass_cnt_next_w;
			
			if (start_r) begin
				// Already found potential barcode start, now validating height
				
				// At the beginning of each row
				if (!row_start_r && pass_cnt_r >= index_r && pass_cnt_r < index_r + 4) begin
					row_start_r <= 1'b1;
					curr_seq_r <= 57'b0;
				end
				
				// Collect sequence bits in current row
				if (row_start_r && seq_cnt_r < 57) begin
					if (hei_cnt_r == 0) begin
						seq_r <= {curr_seq_r[52:0], lsb_w};
					end
					curr_seq_r <= {curr_seq_r[52:0], lsb_w};
					seq_cnt_r <= seq_cnt_r + 7'd4;
				end
				
				// At end of row
				if (seq_cnt_r >= 56 && row_start_r) begin
					if (hei_cnt_r == 4'd0) begin
						// First row - check if there's stop code
						if ({curr_seq_r[8:0], lsb_w} != STOP_CODE) begin
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
							seq_r <= {curr_seq_r[52:0], lsb_w};
							hei_cnt_r <= 4'd1;
							seq_cnt_r <= 7'd0;
							row_start_r <= 1'b0;
						end
					end
					else if (hei_cnt_r < 4'd9) begin
						// Rows 2-9: verify they match first row
						if (seq_r == {curr_seq_r[52:0], lsb_w}) begin
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
						if (seq_r == {curr_seq_r[52:0], lsb_w}) begin
							// Valid 10-row barcode found!
							if (seq_r[56:46] == START_CODE && seq_r[12:0] == STOP_CODE) begin
								// Decode parameters
								ker_r <= decoded_input(seq_r[45:35]);
								str_r <= decoded_input(seq_r[34:24]);
								dil_r <= decoded_input(seq_r[23:13]);
								
								// Validate: K must be 3, S and D must be non-zero
								if (decoded_input(seq_r[45:35]) != 3'd3 || 
								    decoded_input(seq_r[34:24]) == 3'd0 || 
								    decoded_input(seq_r[23:13]) == 3'd0) begin
									ker_r <= 3'd0;
									str_r <= 3'd0;
									dil_r <= 3'd0;
								end
							end
							else begin
								// Invalid START/STOP
								ker_r <= 3'd0;
								str_r <= 3'd0;
								dil_r <= 3'd0;
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
					seq_cnt_r <= 7'd11;
					seq_r <= {46'b0, shift_lsb_r[10:0]};
					index_r <= pass_cnt_r - 9'd10;
					row_start_r <= 1'b1;
					hei_cnt_r <= 4'd0;
					curr_seq_r <= {46'b0, shift_lsb_r[10:0]};
				end
				else if (shift_lsb_r[11:1] == START_CODE) begin
					start_r <= 1'b1;
					seq_cnt_r <= 7'd12;
					seq_r <= {45'b0, shift_lsb_r[11:0]};
					index_r <= pass_cnt_r - 9'd11;
					row_start_r <= 1'b1;
					hei_cnt_r <= 4'd0;
					curr_seq_r <= {45'b0, shift_lsb_r[11:0]};
				end
				else if (shift_lsb_r[12:2] == START_CODE) begin
					start_r <= 1'b1;
					seq_cnt_r <= 7'd13;
					seq_r <= {44'b0, shift_lsb_r[12:0]};
					index_r <= pass_cnt_r - 9'd12;
					row_start_r <= 1'b1;
					hei_cnt_r <= 4'd0;
					curr_seq_r <= {44'b0, shift_lsb_r[12:0]};
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
			end
		end
		else if (state_r == S_IMG && i_in_valid) begin
			if (!flip_r) begin
				// Write to SRAM 0-3, load data for SRAM 4-7
				for (i = 0; i < 4; i = i + 1)
					wen_r[i] <= 1'b0;
				for (i = 4; i < 8; i = i + 1)
					wen_r[i] <= 1'b1;
				for (i = 0; i < 8; i = i + 1)
					img_addr_r[i] <= img_addr_r[i] + 9'd1;
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
	// SEQUENTIAL: SRAM Address Updates for Convolution
	// ============================================================================
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			for (i = 0; i < 8; i = i + 1)
				img_addr_r[i] <= 9'd0;
		end
		else if (state_r == S_WEI && wei_cnt_r == 2'd2 && i_in_valid) begin
			// Initialize SRAM addresses for convolution start
			img_addr_r[0] <= 9'd1;
			if (dil_r == 2'd1)
				img_addr_r[1] <= 9'd1;
			else
				img_addr_r[2] <= 9'd1;
		end
		else if (state_r == S_CONV_INIT || state_r == S_CONV) begin
			// Update from combinational logic
			for (i = 0; i < 8; i = i + 1)
				img_addr_r[i] <= img_addr_w[i];
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
		end
		else begin
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
				S_CONV_INIT: begin
					o_in_ready_r <= 1'b0;
				end
				S_CONV: begin
					o_in_ready_r <= 1'b0;
					// Output convolution results
					if (output_valid_r) begin
						o_out_valid1_r <= 1'b1;
						o_out_data1_r <= output_data_r;
						o_out_addr1_r <= output_addr_r;
					end
					else begin
						o_out_valid1_r <= 1'b0;
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
	function automatic [2:0] decoded_input;
		input [10:0] i_data;
		begin
			case (i_data)
				ONE_CODE:   decoded_input = 3'd1;
				TWO_CODE:   decoded_input = 3'd2;
				THREE_CODE: decoded_input = 3'd3;
				default:    decoded_input = 3'd0;
			endcase
		end
	endfunction

endmodule