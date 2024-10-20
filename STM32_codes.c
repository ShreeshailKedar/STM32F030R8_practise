  // prog to count the number of press done by inbuilt push button 

  
// this code worked and we able to inc the value of the variable after some delay using timer

#include "stm32f0xx.h"

volatile uint32_t button_press_count = 0;

void GPIO_Init(void);
void EXTI_Init(void);
void TIM3_Init(void);

int main(void)
{
    // Initialize GPIO, EXTI, and Timer
    GPIO_Init();
    EXTI_Init();
    TIM3_Init();
    
    while (1)
    {
        // Main loop can be used for other tasks
    }
}

// Initialize GPIO (PC13 as input)
void GPIO_Init(void)
{
    // Enable GPIOC clock (PC13 is in GPIOC)
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    
    // Set PC13 as input (default is input)
    GPIOC->MODER &= ~(GPIO_MODER_MODER13);
}

// Configure external interrupt for PC13
void EXTI_Init(void)
{
    // Enable SYSCFG clock (needed for external interrupt configuration)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    // Configure EXTI line 13 for PC13
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    
    // Enable falling edge trigger for EXTI line 13
    EXTI->FTSR |= EXTI_FTSR_TR13;
    
    // Enable interrupt for EXTI line 13
    EXTI->IMR |= EXTI_IMR_MR13;
    
    // Enable EXTI interrupt in NVIC
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

// Configure TIM3 for basic timing (optional for debounce)
void TIM3_Init(void)
{
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    // Configure TIM3 as a basic timer (no output compare)
    TIM3->PSC = 48000 - 1; // Prescaler to 1ms at 48MHz
    TIM3->ARR = 10 -1;    // Auto-reload for 10ms (debounce time)->waiting time after press ->10=10ms  , 100=100ms,  1000=1000ms-> 6s ,    5000->30s
    TIM3->CR1 |= TIM_CR1_CEN; // Enable timer
}

// Interrupt handler for EXTI line 13 (PC13)
void EXTI4_15_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR13)
    {
        // Clear interrupt pending bit
        EXTI->PR |= EXTI_PR_PR13;
        
        // Check for debounce using TIM3
        if ((TIM3->CNT == 0) || (TIM3->SR & TIM_SR_UIF))
        {
            // Increment button press count
            button_press_count++;
            
            // Clear update flag for TIM3
            TIM3->SR &= ~TIM_SR_UIF;
            
            // Restart TIM3 for debounce
            TIM3->CNT = 0;
        }
    }
}
