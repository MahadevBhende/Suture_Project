#include<stdint.h>
#include<stdio.h>
 int ADC_Val , ADC_Val1;
#define PERIPH_BASE        0x40000000
#define AHB1PERIPH_BASE    (PERIPH_BASE + 0x20000)
#define APB2PERIPH_BASE    (PERIPH_BASE + 0x10000)
#define APB1PERIPH_BASE   0X40000000
//#define RCC_APB1ENR
#define RCC_BASE           (AHB1PERIPH_BASE + 0x3800)
#define GPIOA_BASE         (AHB1PERIPH_BASE + 0x0000)
#define GPIOG_BASE         (AHB1PERIPH_BASE + 0x1800)
#define USART1_BASE        (APB2PERIPH_BASE + 0x1000)

// RCC registers
#define RCC_AHB1ENR        (*(volatile unsigned int*)(RCC_BASE + 0x30))
#define RCC_APB2ENR        (*(volatile unsigned int*)(RCC_BASE + 0x44))
#define RCC_APB1ENR         (*(volatile unsigned int*)(RCC_BASE + 0x40))
// GPIOA registers
#define GPIOA_MODER        (*(volatile unsigned int*)(GPIOA_BASE + 0x00))
#define GPIOA_AFRH         (*(volatile unsigned int*)(GPIOA_BASE + 0x24))
#define GPIOA_IDR         (*(volatile unsigned int*)(GPIOA_BASE + 0x10))
#define RCC_AHB1ENR_GPIOAEN   (1 << 0)
//GPIOG registers
#define GPIOG_MODER        (*(volatile unsigned int *)(GPIOG_BASE + 0x00))
#define GPIOG_ODR          (*(volatile unsigned int *)(GPIOG_BASE + 0x14))
#define GPIOG_BSRR            (*(volatile unsigned int *)(GPIOG_BASE + 0x18))
#define RCC_AHB1ENR_GPIOGEN   (1 << 6)

// USART1 registers
#define USART1_SR          (*(volatile unsigned int*)(USART1_BASE + 0x00))
#define USART1_DR          (*(volatile unsigned int*)(USART1_BASE + 0x04))
#define USART1_BRR         (*(volatile unsigned int*)(USART1_BASE + 0x08))
#define USART1_CR1         (*(volatile unsigned int*)(USART1_BASE + 0x0C))

// TIM2
#define TIM2_BASE      0x40000000
#define TIM2_CR1       (*(volatile uint32_t *)(TIM2_BASE + 0x00))
#define TIM2_CNT       (*(volatile uint32_t *)(TIM2_BASE + 0x24))
#define TIM2_PSC       (*(volatile uint32_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR       (*(volatile uint32_t *)(TIM2_BASE + 0x2C))

//ADC Register Address
#define ADC1    0x40012000
#define  ADC1_SR           (*(volatile unsigned int*)(ADC1+0x00))
#define  ADC1_CR2          (*(volatile unsigned int*)(ADC1+0x08))
#define  ADC1_SMPR2         (*(volatile unsigned int*)(ADC1+0x10))
#define  ADC1_SQR          (*(volatile unsigned int*)(ADC1+0x34))
#define  ADC1_DR            (*(volatile unsigned int*)(ADC1+0x4C))

void ADC_Init_chan2(void)
{
	//Enable clock to GPIOA and ADC1 clock
	   RCC_AHB1ENR |=(1<<0);
	RCC_APB2ENR |=(1<<8);
	//configure PA2 for analog intput
	GPIOA_MODER |=(3<<4);
	//set the sampling time
	ADC1_SMPR2  |= (0x7<<6);
	//config ADC regular sequence register channel 2
	ADC1_SQR =2;
	//setting channel 2 i.e PA2 for continous converstion mode
	ADC1_CR2 |= (1 << 1) ;
	//Enable the ADC channel 2
	ADC1_CR2 |= (1 << 0) ;

	//Start ADC Conversion
	ADC1_CR2 |= (1 << 30) ;
}
unsigned int ADC1_Read(void)
{
while (!(ADC1_SR & (1 << 1))) ;

//get the value
return ADC1_DR;
}
int adc_to_pressure(int adc_value) {
    float Vref = 3.3;       // reference voltage
    float Vmin = 0.5;       // sensor output at 0 PSI
    float Vmax = 4.5;       // sensor output at full scale
    float Pmax = 100.0;     // sensor maximum pressure in PSI

    // Convert ADC value to voltage
    float Vout = ((float)adc_value / 4095.0) * Vref;

    // Convert voltage to pressure
    if (Vout < Vmin) Vout = Vmin; // clamp lower
    if (Vout > Vmax) Vout = Vmax; // clamp upper

    float pressure = ((Vout - Vmin) / (Vmax - Vmin)) * Pmax;
    return pressure;
}


void delay(volatile int t) {
    while (t--);
}
void GPIO_Init(void) {
    RCC_AHB1ENR |= (1 << 0) | (1 << 6); // Enable GPIOA and GPIOG clocks

    // PA0 = Input (Button)
    GPIOA_MODER &= ~(3 << (0 * 2));

    // PG13 = Output (Green LED)
    GPIOG_MODER &= ~(3 << (13 * 2));
    GPIOG_MODER |=  (1 << (13 * 2));
    // PG13 = Output (Green LED)
       GPIOG_MODER &= ~(3 << (14 * 2));
       GPIOG_MODER |=  (1 << (14 * 2));
}

// TIM2 initialization for 1 ms tick
void TIM2_Init(void) {
    RCC_APB1ENR |= (1 << 0);     // TIM2 clock enable
    TIM2_PSC = 16000 - 1;        // 16 MHz / 16000 = 1 kHz (1 ms)
    TIM2_ARR = 0xFFFF;           // Max auto‑reload
    TIM2_CNT = 0;                // Reset counter
    TIM2_CR1 |= 1;               // Enable TIM2
}

void delay_ms(uint32_t ms) {
    TIM2_CNT = 0;                // Reset counter
    while (TIM2_CNT < ms);       // Wait until counter reaches ms
}


void USART1_Init(void) {
    // 1. Enable clocks
    RCC_AHB1ENR |= (1 << 0);    // GPIOAEN
    RCC_APB2ENR |= (1 << 4);    // USART1EN

    // 2. Set PA9 (TX) and PA10 (RX) to alternate function mode (10)
    GPIOA_MODER &= ~((3 << (9 * 2)) | (3 << (10 * 2)));  // clear bits
    GPIOA_MODER |=  (2 << (9 * 2)) | (2 << (10 * 2));    // set AF mode

    // 3. Set alternate function 7 (USART1) for PA9/PA10
    GPIOA_AFRH &= ~((0xF << (4 * 1)) | (0xF << (4 * 2)));  // clear AFRH for PA9/10
    GPIOA_AFRH |=  (7 << (4 * 1)) | (7 << (4 * 2));        // AF7

    // 4. Set baud rate: assuming 16 MHz clock, 9600 baud
    // USARTDIV = 16000000 / 9600 = 1666.67 ≈ 0x0683
    USART1_BRR = 0x0683;

    // 5. Enable USART1: UE, TE, RE
    USART1_CR1 = (1 << 13) | (1 << 3) | (1 << 2);
}

void USART1_Write(char c) {
    while (!(USART1_SR & (1 << 7)));  // wait until TXE

    USART1_DR = c;
}

char USART1_Read(void) {
    while (!(USART1_SR & (1 << 5)));  // wait until RXNE
    return (char)USART1_DR;
}

void print_str(const char *str)
{
    while (*str) {
        USART1_Write(*str++);
    }
}

int main(void)
{
	char buffer[64];
    USART1_Init();
    GPIO_Init();
    TIM2_Init();
    ADC_Init_chan2();
    while (1) {

        if (GPIOA_IDR & 0x01) { // Button pressed?
            print_str("ENTER THE VALUE\r\n");

            char c = USART1_Read(); // Wait for any character
           // (void)c; // We don’t check what was received
            if(c=='1')
            {
            	print_str("FRISRT MCU IS READY FOR COMMUNICATION\r\n");
            	 GPIOG_BSRR = (1 << 13); // Turn on Green LED
            	 delay_ms(1000);         // Delay 1 second
            	GPIOG_BSRR = (1 << (13 + 16)); // Turn off Green LED
            	print_str("AIR FLOW INTO GLOVES\r\n");
            	GPIOG_BSRR = (1 << 13); // Turn on Green LED
            	delay_ms(5000);         // Delay 1 second
            	GPIOG_BSRR = (1 << (13 + 16)); // Turn off Green LED
            	print_str("AIR FLOW IS COMPLETED\r\n");
            	GPIOG_BSRR = (1 << 14 ); // Turn on red LED
            	delay_ms(1000);
            	GPIOG_BSRR = (1 << (14 +16)); // Turn on red LED
            	print_str("TAKE THE INITIAL READING\r\n");
            	ADC_Val = ADC1_Read();
             //  int pressure = adc_to_pressure(ADC_Val);
            	sprintf(buffer, "Pressure: %d PSI\r\n",ADC_Val);
            	print_str(buffer);
            	GPIOG_BSRR = (1 << 13); // Turn on Green LED
            	            	delay_ms(5000);         // Delay 1 second
            	            	GPIOG_BSRR = (1 << (13 + 16)); // Turn off Green LED
            	            	 ADC_Val1= ADC1_Read();
            	//float pressure1 = adc_to_pressure(ADC_Val1);
            	sprintf(buffer, "Pressure: %d PSI\r\n", ADC_Val1);
            	print_str(buffer);

            	int avg = (ADC_Val+ ADC_Val) / 2;

            	if (avg>=50 ||avg <= 65)
            	{
            	    print_str("Gloves is good\r\n");
            	}
            	else
            	{
            	    print_str("Gloves is bad\r\n");
            	}


            }
        }
    }
}



