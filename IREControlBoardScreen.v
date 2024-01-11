
module IREControlBoardScreen(
	
	input			sys_clk,
	input			rst_n,


	output			key_out,
	
	input			UART_RX,
	output			UART_TX,
	output			DA_Clk1,
	output			DA_Clk2,
	output			DACA_WRT1,
	output			DACA_WRT2,
	output 	[13:0]			DA_Data1,
	output	[13:0]			DA_Data2
);



reg  [15:0] pulse_frequency=1000;//脉冲频率（Hz）
reg  [7:0] pulse_inteval=10;//脉冲间隔（us）
reg  [15:0] pulse_width=500;//脉冲宽度（ns）
reg  [15:0] pulse_number=20;//脉冲个数（个）
reg	 [9:0] pulses_number=20;//脉冲串数（串数）
reg	 [13:0] pulses_cycle=800;//脉冲串数（ms）

reg  sel;


reg  [31:0]Fword;/*频率控制字*/
reg  [15:0]Pword;/*相位控制字*/
reg  [15:0]Pwidth;


reg start=0;
reg stop=0;
reg signal=0;
reg [9:0] time_count; // 定时器计数

wire reset_n;

reset_module reset_module_inst
(
    .clk        (sys_clk),
    .rst_o      (),
    .rst_n_o    (reset_n)
);

wire    [15:0]  reg_03_01_o;
wire            reg_03_01_update;
wire            response_done;
wire rs485_oe;
wire [15:0]		addr_r;
modbus_rtu_slave_top #
(
    .CLK_FREQ       ('d50000000    ),  //system clock
    .BAUD_RATE      ('d9600       )
)modbus_rtu_slave_top_inst0
(
    .clk                    (sys_clk            ),			// system clock
    .rst_n                  (reset_n            ),		// system reset, active low
    
    .dev_addr               ( 8'h01 ),  
    .read_04_01             (16'h5347           ),
    .read_04_02             (16'h7414           ),
    .read_04_03             (16'h2021           ),
    .read_04_04             (16'h0402           ),

    .reg_03_01_o            (reg_03_01_o        ),
    .reg_03_01_update       (reg_03_01_update   ),
    .addr_r					(addr_r				),
    .rs485_rx               (UART_RX           ),
    .rs485_tx               (UART_TX           ),
    .rs485_oe               ( rs485_oe),
    
    .response_done          (response_done      )
);




//只使用
always@(posedge sys_clk)
begin
    if(reg_03_01_update == 1'b1)   //发送请求来了，暂存请求
		case(addr_r)
		16'd1:
		begin
			pulse_frequency<=reg_03_01_o;
		end
		16'd2:
		begin
			pulse_inteval<=reg_03_01_o[7:0];
		end
		16'd3:
		begin
			pulse_width<=reg_03_01_o;
		end
		16'd4:
		begin
			pulse_number<=reg_03_01_o;
		end
		16'd5:
		begin
			pulses_number<=reg_03_01_o[9:0];
		end
		16'd6:
		begin
			pulses_cycle<=reg_03_01_o[13:0];
		end
		16'd7:
		begin
			if(reg_03_01_o[0] == 1'b1)
				start <= 1;
			else
				stop <= 1;
		end
		default:;
        endcase
	else
	begin
			
			pulse_frequency <=		pulse_frequency ;
			pulse_inteval 	<=		pulse_inteval 	;
			pulse_width 	<=		pulse_width 	;
			pulse_number 	<=		pulse_number 	;
			pulses_number 	<=		pulses_number 	;
			pulses_cycle 	<=		pulses_cycle 	;
			start           <=      1'b0;
			stop			<=      1'b0;
	end	
       
end

/*脉冲信号源参数设计
	系统频率 ：Fclk=50MHZ(20ns);
	频率字：FWord 32bit，最小分辨率Fout(min)=Fclk/2^(32)=0.0116Hz;最大频率25MHz
	输出频率：Fout=Fword*50MHZ/2^(32);关系Fword=2^32/50M*Fout=[85.8993459*Fout]
	输出脉宽：脉宽精度 1/2^(14)*(1/Fout)*10^6=61.03ns/kHZ,关系width=Pwidth*1/2^(14)*(1/Fout)
	Pwidth=2^(14)*Fout*width=16.384*Fout*width /us.Khz
	相位：interval=Pword/2^(14)*1/Fout-width;Pword=2^14-(interval+width)*Fout*2^(14);
	信号源设计(上位机控制）
	给定参数：脉宽 100ns-9.999us
			正负间隔：1-99us
			频率 ：10-9999HZ
			个数：1-999	
	
	*/
//42,949.67  





assign Fword=(48'hffffffff+1)*pulse_frequency/(50*1000000);
assign Pwidth=(48'hffff+1)*pulse_frequency*pulse_width/1000000000;
assign Pword=(17'hffff+1)-(48'hffff+1)*(pulse_inteval*1000+pulse_width)*pulse_frequency/1000000000;
assign	DACA_WRT1=DA_Clk1;
assign	DACA_WRT2=DA_Clk2;



DDS_Module DDS_Module_inst1(
	.Clk	(sys_clk)	,
	.Rst_n	(1)	,
	.EN	(time_done),
	.Fword	(Fword)	,
	.Pword	(24'd0)	,
	.Pwidth	(Pwidth)	,
	.DA_Clk	(DA_Clk1)	,
	.DA_Data(DA_Data1),
	.num	(pulse_number),
	.sel(sel),
	.ensig(1)
);


DDS_Module DDS_Module_inst2(
	.Clk	(sys_clk)	,
	.Rst_n	(1)	,
	.EN	(time_done),
	.Fword	(Fword)	,
	.Pword	(Pword)	,
	.Pwidth	(Pwidth)	,
	.DA_Clk	(DA_Clk2)	,
	.DA_Data(DA_Data2),
	.num	(pulse_number+1),
	.sel(),
	.ensig(1)
);




reg time_done;//定时器触发到来
timer timer_init(
    .rst		(rst_n),
	.en     (signal),
    .period	(50000*pulses_cycle), 
    .clk		(sys_clk), 
    .done	(time_done), 
    .count 	(time_count)
);


always@(posedge sys_clk)
begin
	if(start==1)
		signal<=1;
	else
	begin
		if(time_count==pulses_number || stop == 1)
		begin 
			signal <=0;
		end
		else 
			signal<=signal;
	end			
end



endmodule 