
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

	reg [7:0] o_out_data1_r;
	reg [7:0] o_out_data2_r;
	reg [7:0] o_out_data3_r;
	reg [7:0] o_out_data4_r;
	reg       o_in_ready_r;
	reg       o_out_valid1_r;
	reg       o_out_valid2_r;
	reg       o_out_valid3_r;
	reg       o_out_valid4_r;

	assign o_out_data1 = o_out_data1_r;
	assign o_out_data2 = o_out_data2_r;
	assign o_out_data3 = o_out_data3_r;
	assign o_out_data4 = o_out_data4_r;
	assign o_out_valid1 = o_out_valid1_r;
	assign o_out_valid2 = o_out_valid2_r;
	assign o_out_valid3 = o_out_valid3_r;
	assign o_out_valid4 = o_out_valid4_r;
	assign o_in_ready = o_in_ready_r;

	// FSM 
	localparam S_IDLE = 3'b000;
	localparam S_IMG  = 3'b001;
	localparam S_WEI  = 3'b010;
	localparam S_CONV = 3'b011;
	localparam S_DONE = 3'b100;
	localparam S_BUFF = 3'b101;

	reg [2:0] state_r, next_state_r;
	reg [1:0] wei_cnt_r;

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

	// Fetching Weight
	reg [7:0] weight_r [0:8];

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
			o_in_ready_r <= 0;
		end
		else begin
			state_r <= next_state_r;
			case (state_r)
				S_IDLE: begin
					
				end
				S_IMG: begin

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
				S_WEI: begin
					// optimization point: if statement or padding reg to 12
					wei_cnt_r <= (wei_cnt_r == 2) ? 0 : wei_cnt_r + 1;
					if (wei_cnt_r == 0) begin
						weight_r[0] <= i_in_data[31:24];
						weight_r[1] <= i_in_data[23:16];
						weight_r[2] <= i_in_data[15:8];
						weight_r[3] <= i_in_data[7:0];
					end
					else if (wei_cnt_r == 1) begin
						weight_r[4] <= i_in_data[31:24];
						weight_r[5] <= i_in_data[23:16];
						weight_r[6] <= i_in_data[15:8];
						weight_r[7] <= i_in_data[7:0];
					end
					else begin
						weight_r[8] <= i_in_data[31:24];
					end
				end
			endcase
		end
	end

	// Saving IMG
	reg         cen_r;
	reg         flip_r;
	reg         wen_r      [0:7];
	reg [8:0]   img_addr_r [0:7];
	reg [7:0]   img_in_r   [0:7];
	reg         img_done_r;
	
	wire       cen_w;
	wire       wen_w      [0:7];
	wire [8:0] img_addr_w [0:7];
	wire [7:0] img_in_w   [0:7];
	wire [7:0] img_out_w  [0:7];
	genvar j;
	integer i;
	
	assign cen_w = cen_r;

	generate
		for (j = 0; j < 8; j = j + 1) begin
			assign img_addr_w[j] = img_addr_r[j];
			assign img_in_w[j] = img_in_r[j];
			assign wen_w[j] = wen_r[j];
			sram_512x8 sram(
				.Q(img_out_w[j]),
				.CLK(i_clk),
				.CEN(cen_w),
				.WEN(wen_w[j]),
				.A(img_addr_w[j]),
				.D(img_in_w[j])
			);
		end
	endgenerate

	always @(*) begin
		case (state_r)
			S_IDLE: begin
				if (i_in_valid) next_state_r = S_IMG;
				else next_state_r = S_IDLE;
			end
			S_IMG:  begin
				if (img_addr_r[0] == 511 && flip_r == 1) next_state_r = S_BUFF;
				else next_state_r = S_IMG;
			end
			S_BUFF: begin
				// todo: check parameter recieved
			end
			S_WEI:  begin
				if (wei_cnt_r == 2) next_state_r = S_CONV;
				else next_state_r = S_WEI;
			end 
			S_CONV: next_state_r = S_CONV;  // todo 
			default: next_state_r = S_IDLE;
		endcase
	end

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			cen_r <= 1;
			flip_r <= 0;
			img_done_r <= 0;
			for (i = 0; i < 8; i = i + 1) begin
				wen_r[i]      <= 1;
				img_addr_r[i] <= 0;
				img_in_r[i]   <= 0;
			end
		end
		else begin
			case (state_r):
				S_IDLE: begin
					if (i_in_valid) begin
						cen_r <= 0;
						wen_r[0] <= 0;
						wen_r[1] <= 0;
						wen_r[2] <= 0;
						wen_r[3] <= 0;
						img_in_r[0] <= i_in_data[31:24];
						img_in_r[1] <= i_in_data[23:16];
						img_in_r[2] <= i_in_data[15:8];
						img_in_r[3] <= i_in_data[7:0];
						flip_r <= 1;
					end
				end 
				S_IMG: begin
					if (i_in_valid) begin
						if (flip_r) begin
							wen_r[0] <= 0;
							wen_r[1] <= 0;
							wen_r[2] <= 0;
							wen_r[3] <= 0;
							wen_r[4] <= 1;
							wen_r[5] <= 1;
							wen_r[6] <= 1;
							wen_r[7] <= 1;
							img_in_r[0] <= i_in_data[31:24];
							img_in_r[1] <= i_in_data[23:16];
							img_in_r[2] <= i_in_data[15:8];
							img_in_r[3] <= i_in_data[7:0];
							flip_r <= 0;
							img_addr_r[0] <= img_addr_r[0] + 1;
							img_addr_r[1] <= img_addr_r[1] + 1;
							img_addr_r[2] <= img_addr_r[2] + 1;
							img_addr_r[3] <= img_addr_r[3] + 1;
							img_addr_r[4] <= img_addr_r[4] + 1;
							img_addr_r[5] <= img_addr_r[5] + 1;
							img_addr_r[6] <= img_addr_r[6] + 1;
							img_addr_r[7] <= img_addr_r[7] + 1;
						end
						else begin
							img_in_r[4] <= i_in_data[31:24];
							img_in_r[5] <= i_in_data[23:16];
							img_in_r[6] <= i_in_data[15:8];
							img_in_r[7] <= i_in_data[7:0];
							wen_r[0] <= 1;
							wen_r[1] <= 1;
							wen_r[2] <= 1;
							wen_r[3] <= 1;
							wen_r[4] <= 0;
							wen_r[5] <= 0;
							wen_r[6] <= 0;
							wen_r[7] <= 0;
							flip_r <= 1;
						end
					end
				end
				S_BUFF: begin
					wen_r[4] <= 1;
					wen_r[5] <= 1;
					wen_r[6] <= 1;
					wen_r[7] <= 1;
				end
				default: 
			endcase
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