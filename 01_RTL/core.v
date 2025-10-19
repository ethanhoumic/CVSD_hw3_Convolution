
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

	// FSM 
	localparam S_IDLE = 3'b000;
	localparam S_IMG  = 3'b001;
	localparam S_VAL  = 3'b010;
	localparam S_WEI  = 3'b011;
	localparam S_CONV = 3'b100;
	localparam S_DONE = 3'b101;

	// Code 128-C Barcode
	localparam START_CODE = 11'b11010011100;
	localparam ONE_CODE   = 11'b11001101100;
	localparam TWO_CODE   = 11'b11001100110;
	localparam THREE_CODE = 11'b10010011000;
	localparam STOP_CODE  = 13'b1100011101011;

	// Fetching LSB
	wire [3:0]  lsb_w = {i_in_data[24], i_in_data[16], i_in_data[8], i_in_data[0]};

	// Barcode decoder registers
	reg  [12:0] shift_lsb_r;       // For START_CODE detection
	reg  [6:0]  pass_cnt_r;        // Pixel counter in row (0,4,8,...,60)
	reg  [3:0]  chunk_idx_r;       // Chunk index (0-14) for 15 chunks
	reg  [3:0]  hei_cnt_r;         // Height counter (0-9)
	reg  [8:0]  barcode_start_r;   // Column where barcode starts
	reg  [59:0] full_seq_r;        // Full 60-bit sequence (for final decode)
	reg  [3:0]  ref_bits [0:14];   // Reference chunks from first row
	reg  [2:0]  ker_r;
	reg  [2:0]  str_r;
	reg  [2:0]  dil_r;
	reg         start_r;           // Barcode detection started
	reg         row_start_r;       // Collecting this row
	reg         barcode_done_r;    // Decoding complete

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			shift_lsb_r <= 13'b0;
			pass_cnt_r <= 7'b0;
			chunk_idx_r <= 4'b0;
			hei_cnt_r <= 4'b0;
			barcode_start_r <= 9'b0;
			full_seq_r <= 60'b0;
			ker_r <= 3'b0;
			str_r <= 3'b0;
			dil_r <= 3'b0;
			start_r <= 1'b0;
			row_start_r <= 1'b0;
			barcode_done_r <= 1'b0;
			// Initialize ref_bits array
			for (integer i = 0; i < 15; i = i + 1) begin
				ref_bits[i] <= 4'b0;
			end
		end
		else begin
			if (i_in_valid && !barcode_done_r) begin
				// Update shift register for START_CODE detection
				shift_lsb_r <= {shift_lsb_r[8:0], lsb_w};
				
				// Update pass counter (wraps at 64, but we use 0,4,8,...,60,0)
				pass_cnt_r <= (pass_cnt_r == 60) ? 7'd0 : pass_cnt_r + 7'd4;
				
				if (start_r) begin
					// Barcode detection already started
					
					// Check if we're at the start of the barcode region in this row
					if (!row_start_r && pass_cnt_r == barcode_start_r) begin
						row_start_r <= 1'b1;
						chunk_idx_r <= 4'd0;
					end
					
					// Collect barcode bits
					if (row_start_r && chunk_idx_r < 15) begin
						if (hei_cnt_r == 0) begin
							// First row: store reference and build full sequence
							ref_bits[chunk_idx_r] <= lsb_w;
							full_seq_r <= {full_seq_r[55:0], lsb_w};
							chunk_idx_r <= chunk_idx_r + 4'd1;
						end
						else begin
							// Subsequent rows: compare with reference
							if (ref_bits[chunk_idx_r] != lsb_w) begin
								// Mismatch detected - reset and search again
								start_r <= 1'b0;
								hei_cnt_r <= 4'd0;
								row_start_r <= 1'b0;
								chunk_idx_r <= 4'd0;
								full_seq_r <= 60'b0;
							end
							else begin
								chunk_idx_r <= chunk_idx_r + 4'd1;
							end
						end
					end
					
					// End of barcode sequence in this row
					if (chunk_idx_r == 15 && row_start_r) begin
						if (hei_cnt_r == 0) begin
							// First row completed - validate START and STOP
							if (full_seq_r[59:49] == START_CODE && full_seq_r[12:0] == STOP_CODE &&
							    decoded_input(full_seq_r[48:38]) == 3 && 
								decoded_input(full_seq_r[37:27]) != 0 &&
								decoded_input(full_seq_r[37:27]) <= 2 &&
								decoded_input(full_seq_r[26:16]) != 0 &&
								decoded_input(full_seq_r[26:16]) <= 2) begin
								// Valid barcode format
								hei_cnt_r <= 4'd1;
								row_start_r <= 1'b0;
								chunk_idx_r <= 4'd0;
							end
							else begin
								// Invalid START/STOP - reset
								start_r <= 1'b0;
								hei_cnt_r <= 4'd0;
								row_start_r <= 1'b0;
								chunk_idx_r <= 4'd0;
								full_seq_r <= 60'b0;
							end
						end
						else if (hei_cnt_r < 9) begin
							// Rows 2-9: matched successfully
							hei_cnt_r <= hei_cnt_r + 4'd1;
							row_start_r <= 1'b0;
							chunk_idx_r <= 4'd0;
						end
						else if (hei_cnt_r == 9) begin
							// 10th row matched - decode parameters
							// Barcode structure (60 bits, but only use 57), [59:49] = START_CODE, [48:38] = Kernel
							// [37:27] = Stride, [26:16] = Dilation, [15:13] = padding, [12:0]  = STOP_CODE
							
							// Decode and validate parameters
							if (decoded_input(full_seq_r[48:38]) == 3 && 
								decoded_input(full_seq_r[37:27]) != 0 &&
								decoded_input(full_seq_r[37:27]) <= 2 &&
								decoded_input(full_seq_r[26:16]) != 0 &&
								decoded_input(full_seq_r[26:16]) <= 2) begin
								// Valid configuration
								ker_r <= 3'd3;
								str_r <= decoded_input(full_seq_r[37:27]);
								dil_r <= decoded_input(full_seq_r[26:16]);
							end
							else begin
								// Invalid configuration - set all to 0
								ker_r <= 3'd0;
								str_r <= 3'd0;
								dil_r <= 3'd0;
							end
							barcode_done_r <= 1'b1;
						end
					end
				end
				else begin
					// Search for START_CODE in shift register
					// Check 3 possible alignments within the 13-bit shift register
					
					if (shift_lsb_r[10:0] == START_CODE && pass_cnt_r >= 10) begin
						start_r <= 1'b1;
						barcode_start_r <= pass_cnt_r - 9'd10;
						hei_cnt_r <= 4'd0;
						row_start_r <= 1'b0;
						chunk_idx_r <= 4'd0;
						full_seq_r <= {49'b0, START_CODE};
					end
					else if (shift_lsb_r[11:1] == START_CODE && pass_cnt_r >= 11) begin
						start_r <= 1'b1;
						barcode_start_r <= pass_cnt_r - 9'd11;
						hei_cnt_r <= 4'd0;
						row_start_r <= 1'b0;
						chunk_idx_r <= 4'd0;
						full_seq_r <= {49'b0, START_CODE};
					end
					else if (shift_lsb_r[12:2] == START_CODE && pass_cnt_r >= 12) begin
						start_r <= 1'b1;
						barcode_start_r <= pass_cnt_r - 9'd12;
						hei_cnt_r <= 4'd0;
						row_start_r <= 1'b0;
						chunk_idx_r <= 4'd0;
						full_seq_r <= {49'b0, START_CODE};
					end
				end
			end
			
			// Handle end of row - reset row_start_r
			if (pass_cnt_r == 60 && i_in_valid) begin
				row_start_r <= 1'b0;
			end
		end
	end

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