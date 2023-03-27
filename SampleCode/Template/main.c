/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "misc_config.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "Queue.h"
#include "timers.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "cpu_utils.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_REVERSE1                   			(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 				(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

// volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;

STR_CANMSG_T rrMsg;

// #define printf(...)	\
// {	\
// 	vTaskSuspendAll();	\
// 	printf(__VA_ARGS__);	\
// 	fflush(stdout); \
// 	xTaskResumeAll();	\
// }	\

unsigned int sRxCount = 0;
#define mainNORMAL_TASK_PRIORITY           	            ( tskIDLE_PRIORITY + 0UL )
#define mainABOVENORMAL_TASK_PRIORITY     	            ( mainNORMAL_TASK_PRIORITY + 1UL )

#define mainCHECK_TASK_PRIORITY                         ( mainNORMAL_TASK_PRIORITY + 3UL )

#define DELAY_MS					                    (1)

#define M483_CHECK                                      ((uint32_t)0x601101) 
/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

// unsigned int get_systick(void)
// {
// 	return (counter_systick);
// }

// void set_systick(unsigned int t)
// {
// 	counter_systick = t;
// }

// void systick_counter(void)
// {
// 	counter_systick++;
// }

// void SysTick_Handler(void)
// {

//     systick_counter();

//     if (get_systick() >= 0xFFFFFFFF)
//     {
//         set_systick(0);      
//     }

//     // if ((get_systick() % 1000) == 0)
//     // {
       
//     // }

//     #if defined (ENABLE_TICK_EVENT)
//     TickCheckTickEvent();
//     #endif    
// }

// void SysTick_delay(unsigned int delay)
// {  
    
//     unsigned int tickstart = get_systick(); 
//     unsigned int wait = delay; 

//     while((get_systick() - tickstart) < wait) 
//     { 
//     } 

// }

// void SysTick_enable(unsigned int ticks_per_second)
// {
//     set_systick(0);
//     if (SysTick_Config(SystemCoreClock / ticks_per_second))
//     {
//         /* Setup SysTick Timer for 1 second interrupts  */
//         printf("Set system tick error!!\n");
//         while (1);
//     }

//     #if defined (ENABLE_TICK_EVENT)
//     TickInitTickEvent();
//     #endif
// }

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

void CAN1_Sender(uint32_t can_id , uint8_t* can_data , uint8_t dlc)
{
    STR_CANMSG_T msg1;
    uint8_t i = 0;
 
    msg1.FrameType= CAN_DATA_FRAME;
    msg1.IdType   = CAN_EXT_ID;
    msg1.Id       = can_id;
    msg1.DLC      = dlc;

    for(i = 0 ; i < dlc; i++)
    {
        msg1.Data[i] = can_data[i];
    }
    
    if (CAN_Transmit(CAN1, MSG(1) , &msg1) == FALSE)
    {
        printf("set tx msg failed\r\n");
        return;
    }
}

void CAN1_transmit(void)
{
    uint8_t can_data[8] = {0};

    static uint8_t cnt = 0;

    can_data[0] = 0x5A;
    can_data[1] = 0x5A;

    can_data[2] = cnt;

    can_data[6] = 0xA5;
    can_data[7] = 0xA5;

    CAN1_Sender(M483_CHECK,can_data,8);

    cnt++;
}

void CAN1_SetRxMsg(void)
{
    CAN_T *tCAN = CAN1;

    if(CAN_SetRxMsg(tCAN, MSG(8),CAN_EXT_ID, M483_CHECK) == FALSE)
    {
        printf("Set Rx Msg Object failed\n");
        return;
    }

    if(CAN_SetRxMsg(tCAN, MSG(0),CAN_STD_ID, 0x7FF) == FALSE)
    {
        printf("Set Rx Msg Object failed\n");
        return;
    }

    if(CAN_SetRxMsg(tCAN, MSG(5),CAN_EXT_ID, 0x12345) == FALSE)
    {
        printf("Set Rx Msg Object failed\n");
        return;
    }

    if(CAN_SetRxMsg(tCAN, MSG(31),CAN_EXT_ID, 0x7FF01) == FALSE)
    {
        printf("Set Rx Msg Object failed\n");
        return;
    }

}

void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;
    printf("Read ID=%8X, Type=%s, DLC=%d,Data=",Msg->Id,Msg->IdType?"EXT":"STD",Msg->DLC);
    for(i=0; i<Msg->DLC; i++)
        printf("%02X,",Msg->Data[i]);
    printf("\n\n");
}

void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    #if 0

    switch(u32IIDR) {
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
            //printf("Msg-<%d> INT and Callback\n", u32IIDR-1);
            // printf("\n\nMsg-<%d> Rx INT\n\n", u32IIDR-1);
            CAN_Receive(tCAN, u32IIDR-1, &rrMsg);
            CAN_ShowMsg(&rrMsg);
            break; //Rx

        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF:
        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13:
        case 0x14:
        case 0x15:
        case 0x16:
        case 0x17:
        case 0x18:
            // printf("\n\nMsg-<%d> Tx INT\n\n", u32IIDR-1);
            CAN_CLR_INT_PENDING_BIT(tCAN, (u32IIDR -1));      /* Clear Interrupt Pending */
            break; //Tx

        case 0x19:
        case 0x1A:
        case 0x1B:
        case 0x1C:
        case 0x1D:
        case 0x1E:
        case 0x1F:
        case 0x20:
        default:
            CAN_CLR_INT_PENDING_BIT(tCAN, (u32IIDR -1));      /* Clear Interrupt Pending */
            break; //No used
    }
    #else
    if(u32IIDR==1)
    {
        printf("Msg-0 INT and Callback\n");
        CAN_Receive(tCAN, 0, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }

    if(u32IIDR==8+1)
    {
        printf("Msg-8 INT and Callback\n");
        CAN_Receive(tCAN, 8, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }

    if(u32IIDR==5+1)
    {
        printf("Msg-5 INT and Callback \n");
        CAN_Receive(tCAN, 5, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
    if(u32IIDR==31+1)
    {
        printf("Msg-31 INT and Callback \n");
        CAN_Receive(tCAN, 31, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
    #endif
}

void CAN1_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN1->IIDR;

    if(u8IIDRstatus == 0x00008000)        /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN1->STATUS & CAN_STATUS_RXOK_Msk)
        {
            CAN1->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear Rx Ok status*/

            printf("RX OK INT\n") ;
        }

        if(CAN1->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN1->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear Tx Ok status*/

            printf("TX OK INT\n") ;
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN1->STATUS & CAN_STATUS_EWARN_Msk)
        {
            printf("EWARN INT\n") ;
        }

        if(CAN1->STATUS & CAN_STATUS_BOFF_Msk)
        {
            printf("BOFF INT\n") ;

            /* Do Init to release busoff pin */
            CAN1->CON = (CAN_CON_INIT_Msk | CAN_CON_CCE_Msk);
            CAN1->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
            while(CAN1->CON & CAN_CON_INIT_Msk);
        }
    }
    else if (u8IIDRstatus!=0)
    {
        // printf("=> Interrupt Pointer = %d\n",CAN1->IIDR -1);
        sRxCount++; 
        CAN_MsgInterrupt(CAN1, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN1, ((CAN1->IIDR) -1));  /* Clear Interrupt Pending */
    }
    else if(CAN1->WU_STATUS == 1)
    {
        printf("Wake up\n");

        CAN1->WU_STATUS = 0;                       /* Write '0' to clear */
    }

}


void CAN1_Init(void)
{
    CAN_Open(CAN1,  500000, CAN_NORMAL_MODE);

    #if 1
    CAN_EnterTestMode(CAN1, CAN_TEST_SILENT_Msk);
    CAN_EnterTestMode(CAN1, CAN_TEST_LBACK_Msk);
    #endif

    CAN_EnableInt(CAN1, CAN_CON_IE_Msk|CAN_CON_SIE_Msk | CAN_CON_EIE_Msk);
    NVIC_SetPriority(CAN1_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(CAN1_IRQn);    
}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    // if ((get_systick() % 1000) == 0)
    // {
    //     // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    // }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PH0 ^= 1;             
    }
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());    	

//    printf("Product ID 0x%8X\n", SYS->PDID);
	
	#endif	

    #if 0
    printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH0MFP_Msk)) | (SYS_GPH_MFPL_PH0MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH1MFP_Msk)) | (SYS_GPH_MFPL_PH1MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH2MFP_Msk)) | (SYS_GPH_MFPL_PH2MFP_GPIO);

	//EVM LED
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    CLK_EnableModuleClock(CAN1_MODULE);
    /* Set PE multi-function pins for CAN1 TXD(PE.7) and RXD(PE.6) */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE6MFP_Msk | SYS_GPE_MFPL_PE7MFP_Msk)) |
                    (SYS_GPE_MFPL_PE6MFP_CAN1_RXD | SYS_GPE_MFPL_PE7MFP_CAN1_TXD);

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}



void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

#if 0   // under cpu_utils.c
void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
#endif

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

#if 0   // under cpu_utils.c
void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()).  */

#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 0 )
    {
        /* In this case the tick hook is used as part of the queue set test. */
        vQueueSetAccessQueueSetFromISR();
    }
#endif /* mainCREATE_SIMPLE_BLINKY_DEMO_ONLY */
}
#endif
/*-----------------------------------------------------------*/


portBASE_TYPE xAreTasCANStillRunning( void )
{
    static short sLastRxCount = 0;
    portBASE_TYPE xReturn;

    /* Not too worried about mutual exclusion on these variables as they are 16
     * bits and we are only reading them.  We also only care to see if they have
     * changed or not. */

    if ( sRxCount == sLastRxCount ) 
    {
        xReturn = pdFALSE;
    }
    else
    {
        xReturn = pdTRUE;
    }

    sLastRxCount = sRxCount;

    return xReturn;
}


void vTask_CAN_flow( void *pvParameters )
{	 

	CAN1_Init();
    CAN1_SetRxMsg();

    printf("%s is running ...\r\n",__FUNCTION__);

	(void) pvParameters;
	for( ;; )
	{ 
        
        /* Add data processing code */

	}  
}

void vTask_logger( void *pvParameters )
{	
 	// static uint32_t cnt = 0;   
	uint32_t millisec = DELAY_MS*1000;
	portTickType xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

    printf("%s is running ...\r\n",__FUNCTION__);

	(void) pvParameters;
	for( ;; )
	{
        vTaskDelayUntil( &xLastWakeTime, ( ( portTickType ) millisec / portTICK_RATE_MS));
        
        #if 0 // debug
        printf("Task logger :%4d (heap:%3dbytes ,CPU:%3d)\r\n" ,cnt++,xPortGetFreeHeapSize(),osGetCPUUsage());
        #endif

        CAN1_transmit();
		
        PH1 ^= 1;
	}  
}

void vTask_check( void *pvParameters )
{	
    static uint32_t timecount = 0;
	uint32_t millisec = DELAY_MS*5000;
    uint32_t res = 0;

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

    timecount = get_tick();

    printf("%s is running ...\r\n",__FUNCTION__);

	(void) pvParameters;
	for( ;; )
	{
        vTaskDelayUntil( &xLastWakeTime, ( ( portTickType ) millisec / portTICK_RATE_MS));
        res = get_tick();
        if( xAreTasCANStillRunning() != pdTRUE )
        {
            #if 0 // debug
            printf( "ERROR IN TaskUart (prev:%5d,current:%5d,diff:%5d)\r\n" , timecount , res , res-timecount);
            timecount = get_tick();
            #endif            
        }
	}  
}


/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();

	// CAN1_Init();
    // CAN1_SetRxMsg();

    // SysTick_enable(1000);
    // #if defined (ENABLE_TICK_EVENT)
    // TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    // TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    // #endif

    xTaskCreate(vTask_CAN_flow      ,"CAN"      ,configMINIMAL_STACK_SIZE*2 ,NULL	,mainNORMAL_TASK_PRIORITY       ,NULL);
    xTaskCreate(vTask_check         ,"check"    ,configMINIMAL_STACK_SIZE   ,NULL   ,mainNORMAL_TASK_PRIORITY       ,NULL);
    xTaskCreate(vTask_logger        ,"logger"   ,configMINIMAL_STACK_SIZE   ,NULL	,mainNORMAL_TASK_PRIORITY       ,NULL);

    vTaskStartScheduler();

    for( ;; );
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
