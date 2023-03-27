# M480BSP_FreeRTOS_CAN
 M480BSP_FreeRTOS_CAN

update @ 2023/03/27

1. initial FreeRTOS , with CAN1 TX /RX 

2. enable loop back and silent mode for test TX / RX

    CAN_EnterTestMode(CAN1, CAN_TEST_SILENT_Msk);
    
	CAN_EnterTestMode(CAN1, CAN_TEST_LBACK_Msk); 

3. below is log capture 

![image](https://github.com/released/M480BSP_FreeRTOS_CAN/blob/main/log.jpg)	


