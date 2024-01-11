`timescale 1ns/1ps

/*
���� : ValentineHP
��ϵ��ʽ : ΢�Ź��ں�  FPGA֮��
*/






/*���ڽ���ģ��*/
module UART_RX(
    input          sys_clk,        /*ϵͳʱ�� 50M*/
    input          rst_n,          /*ϵͳ��λ*/

    output         uart_rx_done,   /*���ڽ������*/
    output[7:0]    odat,           /*��������*/

    input          uartrx         /*uart rx������*/
);

parameter  UARTBaud = 'd115200;     /*������*/
localparam UARTCLKPer = (('d1000_000_000 / UARTBaud) /20) -1;   /*ÿBit��ռ��ʱ������*/

localparam  UART_Idle       =   4'b0001;    /*����̬*/
localparam  UART_Start      =   4'b0010;    /*��ʼ̬*/
localparam  UART_Data       =   4'b0100;    /*����̬*/
localparam  UART_Stop       =   4'b1000;    /*ֹ̬ͣ*/

reg[3:0]    state , next_state;
reg[19:0]   UARTCnt;           /*����ʱ�����ڼ�*/


reg[7:0]    UART_RxData_Reg;   /*���ڽ������ݼĴ���*/
reg[2:0]    UART_Bit;          /*���ڽ���bit������*/

/*����rx����*/
reg uartrxd0,uartrxd1,uartrxd2;
/*���rx ���±���*/
wire  uartrxPosedge , uartrxNegedge;

assign uartrxPosedge = (uartrxd1) & ( ~uartrxd2);
assign uartrxNegedge = (~uartrxd1) & ( uartrxd2);


assign uart_rx_done = (UART_Bit == 'd7 && UARTCnt == UARTCLKPer) ? 1'b1 : 1'b0;
assign odat         = UART_RxData_Reg;

/*����rxʱ����*/
always@(posedge sys_clk or negedge rst_n)
begin
    if(rst_n == 1'b0) 
    begin
        uartrxd0 <= 1'b1;
        uartrxd1 <= 1'b1;
        uartrxd2 <= 1'b1;
    end
    else 
    begin
        uartrxd0 <= uartrx;
        uartrxd1 <= uartrxd0;
        uartrxd2 <= uartrxd1;
    end
end


always@(posedge sys_clk or negedge rst_n)
begin
    if(rst_n == 1'b0)
        state <= UART_Idle;
    else
        state <= next_state;
end
/*״̬��*/
always@(*)
begin
    case(state)
    UART_Idle:
        if(uartrxNegedge == 1'b1)
            next_state <= UART_Start;
        else
            next_state <= UART_Idle;
    UART_Start:
        if(UARTCnt == UARTCLKPer)
            next_state <= UART_Data;
        else
            next_state <= UART_Start;
    UART_Data:
        if(UART_Bit == 'd7 && UARTCnt == UARTCLKPer)
            next_state <= UART_Stop;
        else
            next_state <= UART_Data;
    UART_Stop:
        if(UARTCnt == (UARTCLKPer / 2))
            next_state <= UART_Idle;
        else
            next_state <= UART_Stop;
    default: next_state <= UART_Idle;
    endcase
end



/*����Bitʹ�����ڼ���*/
always@(posedge sys_clk or negedge rst_n)
begin
    if(rst_n == 1'b0)
        UARTCnt <= 'd0;
    else if(UARTCnt == UARTCLKPer)
        UARTCnt <= 'd0;
    else if(state == UART_Data)
        UARTCnt <= UARTCnt + 1'b1;
    else if(state == UART_Stop)
        UARTCnt <= UARTCnt + 1'b1;
    else if(state == UART_Start)
        UARTCnt <= UARTCnt + 1'b1;
    else
        UARTCnt <= 'd0;
end


/*��������bit����*/
always@(posedge sys_clk or negedge rst_n)
begin
    if(rst_n == 1'b0)
        UART_Bit <= 'd0;
    else if(state == UART_Data && UARTCnt == UARTCLKPer)
        UART_Bit <= UART_Bit + 1'b1;
    else if(state == UART_Stop)
        UART_Bit <= 'd0;
    else 
        UART_Bit <= UART_Bit;
end


/*��������*/
always@(posedge sys_clk or negedge rst_n)
begin
    if(rst_n == 1'b0)
        UART_RxData_Reg <= 'd0;
    else if(state == UART_Data && UARTCnt == (UARTCLKPer / 2))
        UART_RxData_Reg <= {uartrx,UART_RxData_Reg[7:1]};   /*�Ƚ��յ�λ*/
    else if(state == UART_Idle)
        UART_RxData_Reg <= 'd0;
    else
        UART_RxData_Reg <= UART_RxData_Reg;
end


endmodule