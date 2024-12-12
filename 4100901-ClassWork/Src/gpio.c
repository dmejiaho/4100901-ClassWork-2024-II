#include "gpio.h"
#include "rcc.h"

#define EXTI_BASE 0x40010400
#define EXTI ((EXTI_t *)EXTI_BASE)

#define EXTI15_10_IRQn 40
#define NVIC_ISER1 ((uint32_t *)(0xE000E104)) // NVIC Interrupt Set-Enable Register

#define SYSCFG_BASE 0x40010000
#define SYSCFG ((SYSCFG_t *)SYSCFG_BASE)

#define GPIOA ((GPIO_t *)0x48000000) // Base address of GPIOA
#define GPIOC ((GPIO_t *)0x48000800) // Base address of GPIOC

#define LED_PIN 5       // Pin 5 of GPIOA
#define LED2_PIN 4      // Pin 4 of GPIOC (New LED 1)
#define LED3_PIN 8      // Pin 8 of GPIOC (New LED 2)
#define BUTTON_PIN 13   // Pin 13 of GPIOC

#define BUTTON_IS_PRESSED()    (!(GPIOC->IDR & (1 << BUTTON_PIN)))
#define BUTTON_IS_RELEASED()   (GPIOC->IDR & (1 << BUTTON_PIN))
#define TOGGLE_LED()           (GPIOA->ODR ^= (1 << LED_PIN))
#define TOGGLE_LED2()          (GPIOC->ODR ^= (1 << LED2_PIN)) // Macro for toggling LED2
#define TOGGLE_LED3()          (GPIOC->ODR ^= (1 << LED3_PIN)) // Macro for toggling LED3

volatile uint8_t button_pressed = 0; // Flag to indicate button press

void configure_gpio_for_usart() {
    *RCC_AHB2ENR |= (1 << 0);

    GPIOA->MODER &= ~(3U << (2 * 2));
    GPIOA->MODER |= (2U << (2 * 2));

    GPIOA->MODER &= ~(3U << (3 * 2));
    GPIOA->MODER |= (2U << (3 * 2));

    GPIOA->AFR[0] &= ~(0xF << (4 * 2));
    GPIOA->AFR[0] |= (7U << (4 * 2));
    GPIOA->AFR[0] &= ~(0xF << (4 * 3));
    GPIOA->AFR[0] |= (7U << (4 * 3));

    GPIOA->OSPEEDR |= (3U << (2 * 2));
    GPIOA->OSPEEDR |= (3U << (3 * 2));

    GPIOA->PUPDR &= ~(3U << (2 * 2));
    GPIOA->PUPDR &= ~(3U << (3 * 2));
}

void init_gpio_pin(GPIO_t *GPIOx, uint8_t pin, uint8_t mode) {
    GPIOx->MODER &= ~(0x3 << (pin * 2));
    GPIOx->MODER |= (mode << (pin * 2));
}

void configure_gpio(void) {
    *RCC_AHB2ENR |= (1 << 0) | (1 << 2); // Enable clock for GPIOA and GPIOC

    *RCC_APB2ENR |= (1 << 0); // RCC_APB2ENR_SYSCFGEN

    SYSCFG->EXTICR[3] &= ~(0xF << 4);
    SYSCFG->EXTICR[3] |= (0x2 << 4);

    EXTI->FTSR1 |= (1 << BUTTON_PIN);
    EXTI->RTSR1 &= ~(1 << BUTTON_PIN);

    EXTI->IMR1 |= (1 << BUTTON_PIN);

    init_gpio_pin(GPIOA, LED_PIN, 0x1);
    init_gpio_pin(GPIOC, BUTTON_PIN, 0x0);
    init_gpio_pin(GPIOC, LED2_PIN, 0x1); // Configure PC4 as output for LED2
    init_gpio_pin(GPIOC, LED3_PIN, 0x1); // Configure PC8 as output for LED3

    *NVIC_ISER1 |= (1 << (EXTI15_10_IRQn - 32));

    configure_gpio_for_usart();
}

uint8_t gpio_button_is_pressed(void) {
    return BUTTON_IS_PRESSED();
}

void gpio_toggle_led(void) {
    TOGGLE_LED();
}

void gpio_toggle_led2(void) { // Function to toggle LED2
    TOGGLE_LED2();
}

void gpio_toggle_led3(void) { // Function to toggle LED3
    TOGGLE_LED3();
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR1 & (1 << BUTTON_PIN)) {
        EXTI->PR1 = (1 << BUTTON_PIN); // Clear pending bit
        button_pressed = 1;

        gpio_toggle_led();
        gpio_toggle_led2(); // Toggle LED2
        gpio_toggle_led3(); // Toggle LED3
    }
}
