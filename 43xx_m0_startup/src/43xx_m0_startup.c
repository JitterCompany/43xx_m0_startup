
#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias(# f)))

extern int main(void);
void ResetISR(void);
WEAK void IntDefaultHandler(void);
WEAK extern void __valid_user_code_checksum();


/* cortex-M0 handlers */
void NMI_Handler(void) ALIAS(IntDefaultHandler);
void HardFault_Handler(void) ALIAS(IntDefaultHandler);
void MemManage_Handler(void) ALIAS(IntDefaultHandler);
void BusFault_Handler(void) ALIAS(IntDefaultHandler);
void UsageFault_Handler(void) ALIAS(IntDefaultHandler);
void SVC_Handler(void) ALIAS(IntDefaultHandler);
void DebugMon_Handler(void) ALIAS(IntDefaultHandler);
void PendSV_Handler(void) ALIAS(IntDefaultHandler);
void SysTick_Handler(void) ALIAS(IntDefaultHandler);

/* LPC43xx M0 specific handlers */
void RTC_IRQHandler(void) ALIAS(IntDefaultHandler);
void M4_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA_IRQHandler(void) ALIAS(IntDefaultHandler);
void FLASHEEPROM_IRQHandler(void) ALIAS(IntDefaultHandler);
void ETH_IRQHandler(void) ALIAS(IntDefaultHandler);
void SDIO_IRQHandler(void) ALIAS(IntDefaultHandler);
void LCD_IRQHandler(void) ALIAS(IntDefaultHandler);
void USB0_IRQHandler(void) ALIAS(IntDefaultHandler);
void USB1_IRQHandler(void) ALIAS(IntDefaultHandler);
void SCT_IRQHandler(void) ALIAS(IntDefaultHandler);
void RIT_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER0_IRQHandler(void) ALIAS(IntDefaultHandler);
void GINT1_IRQHandler(void) ALIAS(IntDefaultHandler);
void GPIO4_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER3_IRQHandler(void) ALIAS(IntDefaultHandler);
void MCPWM_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC0_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C0_IRQHandler(void) ALIAS(IntDefaultHandler);
void SGPIO_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI_IRQHandler (void) ALIAS(IntDefaultHandler);
void ADC1_IRQHandler(void) ALIAS(IntDefaultHandler);
void SSP0_IRQHandler(void) ALIAS(IntDefaultHandler);
void EVRT_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART1_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART2_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART3_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2S0_IRQHandler(void) ALIAS(IntDefaultHandler);
void CAN0_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPIFI_ADCHS_IRQHandler(void) ALIAS(IntDefaultHandler);
void M0SUB_IRQHandler(void) ALIAS(IntDefaultHandler);

extern void _vStackTop(void);
extern void _pvHeapStart(void);

extern void(*const g_pfnVectors[]) (void);
__attribute__ ((section(".isr_vector")))
void(*const g_pfnVectors[]) (void) = {
    // Core Level - CM0
    &_vStackTop,                    // The initial stack pointer
    ResetISR,						// The reset handler
    NMI_Handler,					// The NMI handler
    HardFault_Handler,				// The hard fault handler
    0,								// Reserved
    0,								// Reserved
    0,								// Reserved
    __valid_user_code_checksum,	    // Flash checksum (see linker script)
    0, 								// Reserved
    0, 								// Reserved
    0,								// Reserved
    SVC_Handler,					// SVCall handler
    DebugMon_Handler,				// Debug monitor handler
    0,								// Reserved
    PendSV_Handler,					// The PendSV handler
    SysTick_Handler,				// The SysTick handler

    // Chip Level - 43xx M0 core
    RTC_IRQHandler,					// 16 RTC
    M4_IRQHandler,					// 17 CortexM4/M0 (LPC43XX ONLY)
    DMA_IRQHandler,					// 18 General Purpose DMA
    0,								// 19 Reserved
    FLASHEEPROM_IRQHandler,			// 20 ORed flash Bank A, flash Bank B, EEPROM interrupts
    ETH_IRQHandler,					// 21 Ethernet
    SDIO_IRQHandler,				// 22 SD/MMC
    LCD_IRQHandler,					// 23 LCD
    USB0_IRQHandler,				// 24 USB0
    USB1_IRQHandler,				// 25 USB1
    SCT_IRQHandler,					// 26 State Configurable Timer
    RIT_IRQHandler,					// 27 ORed Repetitive Interrupt Timer, WWDT
    TIMER0_IRQHandler,				// 28 Timer0
    GINT1_IRQHandler,				// 29 GINT1
    GPIO4_IRQHandler,				// 30 GPIO4
    TIMER3_IRQHandler,				// 31 Timer 3
    MCPWM_IRQHandler,				// 32 Motor Control PWM
    ADC0_IRQHandler,				// 33 A/D Converter 0
    I2C0_IRQHandler,				// 34 ORed I2C0, I2C1
    SGPIO_IRQHandler,				// 35 SGPIO (LPC43XX ONLY)
    SPI_IRQHandler,					// 36 ORed SPI, DAC (LPC43XX ONLY)
    ADC1_IRQHandler,				// 37 A/D Converter 1
    SSP0_IRQHandler,				// 38 ORed SSP0, SSP1
    EVRT_IRQHandler,				// 39 Event Router
    UART0_IRQHandler,				// 40 UART0
    UART1_IRQHandler,				// 41 UART1
    UART2_IRQHandler,				// 42 UART2
    UART3_IRQHandler,				// 43 USRT3
    I2S0_IRQHandler,				// 44 ORed I2S0, I2S1
    CAN0_IRQHandler,				// 45 C_CAN0
    SPIFI_ADCHS_IRQHandler,			// 46 SPIFI OR ADCHS interrupt
    M0SUB_IRQHandler,			    // 47 M0SUB core
};

__attribute__ ((section(".after_vectors")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len)
{
    unsigned int *pulDest = (unsigned int *) start;
    unsigned int *pulSrc = (unsigned int *) romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors")))
void bss_init(unsigned int start, unsigned int len)
{
    unsigned int *pulDest = (unsigned int *) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}

__attribute__ ((section(".after_vectors")))
void stack_init(unsigned int start, unsigned int len, unsigned int val)
{
    unsigned int *pulDest = (unsigned int *) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4) {
        *pulDest++ = val;
    }
}

extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;

extern unsigned int stack_value;

void ResetISR(void)
{

    unsigned int LoadAddr, ExeAddr, SectionLen;
    unsigned int *SectionTableAddr;

    // Load base address of Global Section Table
    SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
    while (SectionTableAddr < &__data_section_table_end) {
        LoadAddr = *SectionTableAddr++;
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        data_init(LoadAddr, ExeAddr, SectionLen);
    }
    // At this point, SectionTableAddr = &__bss_section_table;
    // Zero fill the bss segment
    while (SectionTableAddr < &__bss_section_table_end) {
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        bss_init(ExeAddr, SectionLen);
    }

    // init stack 
    stack_init((int)&_pvHeapStart, 
            (unsigned int)((unsigned int)&_vStackTop - 
                (unsigned int)&_pvHeapStart), stack_value);

    main();
    while(1);
}

__attribute__ ((section(".after_vectors")))
void IntDefaultHandler(void)
{
    while (1) {}
}

