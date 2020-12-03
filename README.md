# M480BSP_SPI_Master_PDMA
 M480BSP_SPI_Master_PDMA

update @ 2020/12/03

1. SPI hw config : PB4MFP_SPI1_MOSI , PB5MFP_SPI1_MISO , PB3MFP_SPI1_CLK , PB2MFP_SPI1_SS

2. SPI clock : 800K , transmit length : 8 bytes , transmit timing : 50ms , per packet

3. USE define ENABLE_SPI_NON_AUTO_SS , to check SS pin by GPIO control 

4. Below is use SPI SS hardware auto control capture

![image](https://github.com/released/M480BSP_SPI_Master_PDMA/blob/main/SS_Auto.jpg)

Below is with SS auto ,measure SS pin HIGH TO LOW (1.25us) and LOW to HIGH (1.75us) timing 

![image](https://github.com/released/M480BSP_SPI_Master_PDMA/blob/main/SS_Auto_Timing.jpg)

Below is with SS GPIO control ,measure SS pin HIGH TO LOW (7.5us) and LOW to HIGH (3.5us) timing 

![image](https://github.com/released/M480BSP_SPI_Master_PDMA/blob/main/SS_GPIO_Timing.jpg)

Below is transmit timing capture 

![image](https://github.com/released/M480BSP_SPI_Master_PDMA/blob/main/Transmit_timing.jpg)

