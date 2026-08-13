#include "int_SI24R1.h"

// 定义一个静态发送地址（发送地址与接收地址相同，第一位地址0x0A不要乱改）
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01};

// SPI读写一个字节。参数为要写入的字节，返回值为读取（交换）的字节：
static uint8_t SPI_RW(uint8_t byte)
{
	uint8_t rx_data;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_data, 1, 1000);
	return rx_data;
}
/****************************************************
函数功能：清空RX FIFO队列
入口参数：无
返回值：无
****************************************************/
void Int_SI24R1_FlushRX(void)
{
    CS_LOW();
    SPI_RW(FLUSH_RX);  // 仅发送单字节指令，不额外写数据
    CS_HIGH();
}

/****************************************************
函数功能：清空TX FIFO队列
入口参数：无
返回值：无
****************************************************/
void Int_SI24R1_FlushTX(void)
{
    CS_LOW();
    SPI_RW(FLUSH_TX);  // 仅发送单字节指令，不额外写数据
    CS_HIGH();
}

/********************************************************
函数功能：SI24R1引脚初始化
入口参数：无
返回  值：无
*********************************************************/

/********************************************************
函数功能：写寄存器的值（单字节）
入口参数：reg:寄存器映射地址（格式：SI24R1_WRITE_REG｜reg）
					value:寄存器的值
返回  值：状态寄存器的值
*********************************************************/
uint8_t Int_SI24R1_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CS_LOW();
	status = SPI_RW(reg);
	SPI_RW(value);
	CS_HIGH();

	return (status);
}

/********************************************************
函数功能：写寄存器的值（多字节）
入口参数：reg:寄存器映射地址（格式：SI24R1_WRITE_REG｜reg）
					pBuf:写数据首地址
					size:写数据字节数
返回值： 状态寄存器的值
*********************************************************/
uint8_t Int_SI24R1_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t size)
{
	uint8_t status, byte_ctr;

	CS_LOW();
	status = SPI_RW(reg);
	for (byte_ctr = 0; byte_ctr < size; byte_ctr++)
	{
		SPI_RW(*pBuf++);
	}
	CS_HIGH();
	return (status);
}

/********************************************************
函数功能：读取寄存器的值（单字节）
入口参数：reg:寄存器映射地址（格式：SI24R1_READ_REG｜reg）
返回  值：寄存器值
*********************************************************/
uint8_t Int_SI24R1_Read_Reg(uint8_t reg)
{
	uint8_t value;

	CS_LOW();
	SPI_RW(reg);
	value = SPI_RW(0);
	CS_HIGH();

	return (value);
}

/********************************************************
函数功能：读取寄存器的值（多字节）
入口参数：reg:寄存器映射地址（SI24R1_READ_REG｜reg）
					pBuf:接收缓冲区的首地址
					size:读取字节数
返回  值：状态寄存器的值
*********************************************************/
uint8_t Int_SI24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t size)
{
	uint8_t status, byte_ctr;

	CS_LOW();
	status = SPI_RW(reg);
	for (byte_ctr = 0; byte_ctr < size; byte_ctr++)
	{
		pBuf[byte_ctr] = SPI_RW(0);
	}
	CS_HIGH();

	return (status);
}

/********************************************************
函数功能：SI24R1接收模式初始化
入口参数：无
返回  值：无
*********************************************************/
void Int_SI24R1_RX_Mode(void)
{
	CE_LOW();
	/* 以下配置函数的参数皆是 地址 + 配置值。地址代表寄存器的地址，配置值代表要配置该寄存器的值。地址与值的含义请参考SI24R1数据手册 */
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 接收设备接收通道0使用和发送设备相同的发送地址
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_AA, 0x01);						   // 使能接收通道0自动应答
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_RXADDR, 0x01);					   // 使能接收通道0
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_CH, 40);							   // 选择射频通道40
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);			   // 接收通道0选择和发送通道相同有效数据宽度
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_SETUP, 0x06);					   // 数据传输率1Mbps，发射功率4dBm
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + CONFIG, 0x0f);						   // CRC使能，16位CRC校验，上电，接收模式
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, 0xff);						   // 清除所有的中断标志位

	//清空FIFO队列，防止接收数据时，FIFO队列中有残留数据导致接收失败：
	Int_SI24R1_FlushTX();
    Int_SI24R1_FlushRX();
    Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + 0x07, 0xFF);
	
	CE_HIGH();																	   // 拉高CE启动接收设备
}

/********************************************************
函数功能：SI24R1发送模式初始化
入口参数：无
返回  值：无
*********************************************************/
void Int_SI24R1_TX_Mode(void)
{
	CE_LOW();
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);	   // 写入发送地址
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 为了应答接收设备，接收通道0地址和发送地址相同
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_AA, 0x01);						   // 使能接收通道0自动应答
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_RXADDR, 0x01);					   // 使能接收通道0
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + SETUP_RETR, 0x0a);					   // 自动重发延时等待250us+86us，自动重发10次
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_CH, 40);							   // 选择射频通道0x40
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_SETUP, 0x06);					   // 数据传输率1Mbps，发射功率4dBm
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + CONFIG, 0x0e);						   // CRC使能，16位CRC校验，上电

	//清空FIFO队列，防止接收数据时，FIFO队列中有残留数据导致发送失败：
	Int_SI24R1_FlushTX();
    Int_SI24R1_FlushRX();
    Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + 0x07, 0xFF);

	CE_HIGH();																	   // 拉高CE启动发送设备
}

/********************************************************
函数功能：读取接收数据。硬件直接接收数据保存到 FIFO队列中，可通过标志位判断队列中是否有数据
入口参数：rxbuf:接收数据存放首地址
返回  值：0:接收到数据
		  1:没有接收到数据
*********************************************************/
uint8_t Int_SI24R1_RxPacket(uint8_t *rxbuf)
{

	uint8_t state;
	state = Int_SI24R1_Read_Reg(STATUS); // 读取状态寄存器的值
	// RX_DR代表接收到数据标志位。只要接收到了数据，硬件会自动将数据放入FIFO队列，并将RX_DR标志位置1
	// 硬件规定：如果之前状态寄存器的值为1，则将状态寄存器的值写入状态寄存器后，会清除RX_DR标志位。
	// 如果不清零RX_DR，下次调用函数还会误判 “还有数据”，重复读取同一包
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, state); // 清零RX_DR标志位
	if (state & RX_DR)	// 接收到数据。RX_DR是宏，为0x40。转化为二进制，第6位为1，即01000000。若接收到数据，状态寄存器的第6位会被置1，则state与RX_DR的与运算结果为0x40，不为0，代表接收到数据
	{
		Int_SI24R1_Read_Buf(RD_RX_PLOAD, rxbuf, TX_PLOAD_WIDTH); // 读取数据
		Int_SI24R1_FlushRX();					 // 清除RX FIFO寄存器（接收到的数据）
		return 0;
	}
	return 1; // 没收到任何数据
}

//读取指令无法正常读取数据时，可直接使用读取FIFO队列的方式读取数据，规避异常：
// uint8_t Int_SI24R1_RxPacket(uint8_t *rxbuf)
// {
//     uint8_t status;
//     CS_LOW();
//     // 发送读RX FIFO命令，同时获取实时STATUS
//     status = SPI_RW(RD_RX_PLOAD);
//     // 读取17字节载荷
//     for(uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
//     {
//         rxbuf[i] = SPI_RW(0);
//     }
//     CS_HIGH();
        
//     if (status & RX_DR)
//     {
//         Int_SI24R1_FlushRX();
//         // 清除RX_DR标志
//         Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, RX_DR);
//         return 0;
//     }
//     return 1;
// }

//更稳定的写法，避免时序错乱或宏定义冲突导致读取失败：
// uint8_t Int_SI24R1_RxPacket(uint8_t *rxbuf)
// {
//     // 1. 先读状态寄存器，确认有新数据包（直接用地址0x07，规避宏冲突）
//     uint8_t state = Int_SI24R1_Read_Reg(0x07);
    
//     // 没有新数据，直接返回，绝不操作FIFO
//     if (!(state & 0x40)) // 0x40 = RX_DR标志位
//     {
//         return 1;
//     }

//     // 2. 确认有数据，再读取载荷
//     CS_LOW();
//     SPI_RW(RD_RX_PLOAD); // 发送读载荷命令
//     for(uint8_t i = 0; i < TX_PLOAD_WIDTH; i++)
//     {
//         rxbuf[i] = SPI_RW(0);
//     }
//     CS_HIGH();

//     // 3. 仅清除RX_DR中断标志（写1清零）
//     Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + 0x07, 0x40);

//     // 注意：正常接收绝不调用FlushRX！硬件自动移除已读数据包
//     return 0;
// }
/********************************************************
函数功能：发送一个数据包
入口参数：txbuf:要发送的数据
返回  值：0:发送成功 1:发送失败
*********************************************************/
uint8_t Int_SI24R1_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
	CE_LOW();												  // CE拉低，使能SI24R1配置（进入到配置模式，方便下面往发送队列即 TX FIFO 中写数据）
	Int_SI24R1_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); // 写数据到TX FIFO,32个字节（WR_TX_PLOAD代表往发送队列中写数据的命令）
	CE_HIGH();												  // CE置高，使能发送
	//此后，SI24R1会自动发送数据，并根据配置的重发次数和重发延时时间来重发数据，后续只需等待与判断发送是否成功：

	// while (IRQ == 1)
	// 	;										 // 等待发送完成(中断标志方式判断，这里使用轮询方式判断)

	state = Int_SI24R1_Read_Reg(STATUS); // 读取状态寄存器的值
	// TX_DS代表发送完成标志位。只要发送完成，硬件会自动将TX_DS标志位置1
	// MAX_RT代表达到最大重发次数标志位。只要达到最大重发次数，硬件会自动将MAX_RT标志位置1
	while (((state & TX_DS) == 0) && ((state & MAX_RT) == 0)) // 轮询状态寄存器，直到TX_DS或MAX_RT中断标志位被置1，证明发送完成或达到最大重发次数
	{
		state = Int_SI24R1_Read_Reg(STATUS); // 更新state
		vTaskDelay(1);//vtask延时1ms，以免函数阻塞导致低优先级任务无法执行
	}
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, state); // 清除TX_DS或MAX_RT中断标志
	if (state & MAX_RT)	// 达到最大重发次数
	{
		Int_SI24R1_FlushTX(); // 手动清除TX FIFO寄存器，因为达到最大重发次数不会自动清除TX FIFO寄存器
		return 1;//达到最大重发次数，发送失败，返回1
	}
	if (state & TX_DS) // 发送完成
	{
		// 发送完成，硬件会自动清除TX FIFO寄存器
		return 0;//发送完成，返回0
	}
	return 1; // 发送失败
}

uint8_t si24r1_rx_buf[TX_ADR_WIDTH] = {0};//测试数组

/********************************************************
函数功能：SI24R1初始化检查
入口参数：无
返回  值：0:初始化成功 1:初始化失败
*********************************************************/
uint8_t Iny_SI24R1_Check(void)//检验是否初始化完成
{
	//1.测试SPI能否正常读写寄存器
	//1.0由于si24ri的奇怪设定，必须先读取一次寄存器，才能正常写入寄存器，否则写入的值会被丢弃。所以这里先读取一次寄存器
	Int_SI24R1_Read_Buf(SI24R1_READ_REG + TX_ADDR, si24r1_rx_buf, TX_ADR_WIDTH);

	//1.1 将数据写入到寄存器，这里就写入 发送地址(TX_ADDRESS)
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);

	//1.2 读取寄存器的值，这里将值读取到测试数组中
	Int_SI24R1_Read_Buf(SI24R1_READ_REG + TX_ADDR, si24r1_rx_buf, TX_ADR_WIDTH);

	//2.检查读取到的数据是否正确
	for(uint8_t i = 0; i < TX_ADR_WIDTH; i++)
	{
		if(si24r1_rx_buf[i] != TX_ADDRESS[i])
		{
			return 1; //初始化失败
		}
	}
	return 0; //初始化成功
}

//硬件接口层SI24R1的初始化函数:
void Int_SI24R1_Init(void) 
{
	HAL_Delay(500);//芯片上电延时，应大于100ms
	while(Iny_SI24R1_Check() == 1)//检验是否初始化完成
	{
		HAL_Delay(10);//每两次检测间隔10ms
	}
	
	//SI24R1默认进入接收模式。若要发送，需手动切换为发送模式：
    Int_SI24R1_RX_Mode();
	debug_printf("SI24R1初始化完成\r\n");
}
