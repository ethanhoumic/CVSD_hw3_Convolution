
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
	reg  [12:0] shift_lsb_r;
	reg  [6:0]  pass_cnt_r;
	reg  [6:0]  seq_cnt_r;      // Changed to 7 bits to hold up to 57
	reg  [3:0]  hei_cnt_r;
	reg  [8:0]  index_r;
	reg  [56:0] seq_r;
	reg  [56:0] curr_seq_r;
	reg  [2:0]  ker_r;
	reg  [2:0]  str_r;
	reg  [2:0]  dil_r;
	reg         start_r, row_start_r;
	reg         barcode_done_r;  // Flag to indicate barcode decoding complete

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			seq_r <= 57'b0;
			curr_seq_r <= 57'b0;
			start_r <= 0;
			row_start_r <= 0;
			index_r <= 0;
			seq_cnt_r <= 0;
			hei_cnt_r <= 0;
			shift_lsb_r <= 0;
			pass_cnt_r <= 0;
			ker_r <= 0;
			str_r <= 0;
			dil_r <= 0;
			barcode_done_r <= 0;
		end
		else begin
			if (i_in_valid && !barcode_done_r) begin  // Stop decoding once done
				// Update shift register and pass counter
				shift_lsb_r <= (pass_cnt_r == 0) ? {9'b0, lsb_w} : {shift_lsb_r[8:0], lsb_w};
				pass_cnt_r <= (pass_cnt_r == 60) ? 0 : pass_cnt_r + 4;  // Fixed: 60 not 62 (0,4,8,...,60 then wrap)
				
				if (start_r) begin
					// Already found potential barcode start, now validating height
					
					// At the beginning of each row (when pass_cnt_r reaches index_r)
					if (!row_start_r && pass_cnt_r >= index_r && pass_cnt_r < index_r + 4) begin
						row_start_r <= 1;
						curr_seq_r <= 57'b0;  // Reset current sequence for this row
					end
					
					// Collect sequence bits in current row
					if (row_start_r && seq_cnt_r < 57) begin
						curr_seq_r <= {curr_seq_r[52:0], lsb_w};
						seq_cnt_r <= seq_cnt_r + 4;
					end
					
					// At end of row (when collected full sequence)
					if (seq_cnt_r >= 56 && row_start_r) begin
						// Check if current row matches the first row
						if (hei_cnt_r == 0) begin
							// First row - check if there's stop
							if ({curr_seq_r[8:0], lsb_w} != STOP_CODE) begin
								start_r <= 0;
								hei_cnt_r <= 0;
								row_start_r <= 0;
								index_r <= 0;
								seq_cnt_r <= 0;
								seq_r <= 57'b0;
							end
							else begin
								seq_r <= {curr_seq_r[52:0], lsb_w};
								hei_cnt_r <= 1;
								seq_cnt_r <= 0;
								row_start_r <= 0;
							end
						end
						else if (hei_cnt_r < 9) begin
							// Rows 2-9: verify they match first row
							if (seq_r == {curr_seq_r[52:0], lsb_w}) begin
								hei_cnt_r <= hei_cnt_r + 1;
								seq_cnt_r <= 0;
								row_start_r <= 0;
							end
							else begin
								// Mismatch - reset and search again
								start_r <= 0;
								hei_cnt_r <= 0;
								row_start_r <= 0;
								index_r <= 0;
								seq_cnt_r <= 0;
								seq_r <= 57'b0;
							end
						end
						else if (hei_cnt_r == 9) begin
							// 10th row: final validation and decode
							if (seq_r == {curr_seq_r[52:0], lsb_w}) begin
								// Valid 10-row barcode found!
								// Decode: seq_r[56:46]=START, [45:35]=K, [34:24]=S, [23:13]=D, [12:0]=STOP
								
								// Validate START and STOP
								if (seq_r[56:46] == START_CODE && seq_r[12:0] == STOP_CODE) begin
									// Decode parameters
									if (decoded_input(seq_r[45:35]) == 3 && 
										decoded_input(seq_r[34:24]) != 0 && 
										decoded_input(seq_r[23:13]) != 0) begin
										ker_r <= decoded_input(seq_r[45:35]);
										str_r <= decoded_input(seq_r[34:24]);
										dil_r <= decoded_input(seq_r[23:13]);
									end
									else begin
										// Invalid parameters (K!=3 or S=0 or D=0)
										ker_r <= 0;
										str_r <= 0;
										dil_r <= 0;
									end
								end
								else begin
									// Invalid START/STOP
									ker_r <= 0;
									str_r <= 0;
									dil_r <= 0;
								end
								barcode_done_r <= 1;  // Mark as complete
							end
							else begin
								// Mismatch on 10th row - reset
								start_r <= 0;
								hei_cnt_r <= 0;
								row_start_r <= 0;
								index_r <= 0;
								seq_cnt_r <= 0;
								seq_r <= 57'b0;
							end
						end
					end
				end
				else begin
					// Search for START_CODE in shift register
					// Check all 4 possible alignments
					if (shift_lsb_r[10:0] == START_CODE) begin
						start_r <= 1;
						seq_cnt_r <= 11;
						seq_r <= {46'b0, shift_lsb_r[10:0]};
						index_r <= pass_cnt_r - 10;
						row_start_r <= 1;
						hei_cnt_r <= 0;
						curr_seq_r <= {46'b0, shift_lsb_r[10:0]};
					end
					else if (shift_lsb_r[11:1] == START_CODE) begin
						start_r <= 1;
						seq_cnt_r <= 12;
						seq_r <= {45'b0, shift_lsb_r[11:1]};
						index_r <= pass_cnt_r - 11;
						row_start_r <= 1;
						hei_cnt_r <= 0;
						curr_seq_r <= {45'b0, shift_lsb_r[11:1]};
					end
					else if (shift_lsb_r[12:2] == START_CODE) begin
						start_r <= 1;
						seq_cnt_r <= 13;
						seq_r <= {44'b0, shift_lsb_r[12:2]};
						index_r <= pass_cnt_r - 12;
						row_start_r <= 1;
						hei_cnt_r <= 0;
						curr_seq_r <= {44'b0, shift_lsb_r[12:2]};
					end
				end
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