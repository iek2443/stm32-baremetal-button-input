/**
 * STM32F4 Bare-Metal LED Control using GPIO_WritePin function
 *
 * - Blinks all 4 user LEDs (LD3–LD6) on the STM32F4 Discovery board.
 * - Uses a custom GPIO_WritePin() function and register structures (typedef struct).
 * - No HAL, no CMSIS — pure register-level programming.
 */

#include <stdint.h>

// ==============================
// Base Address Definitions
// ==============================

// STM32 peripheral base addresses
#define PERIPH_BASE             (0x40000000UL)
#define AHB1PERIPH_OFFSET       (0x00020000UL)
#define AHB1PERIPH_BASE         (PERIPH_BASE + AHB1PERIPH_OFFSET)

#define GPIOA_OFFSET            (0x0000UL)
#define GPIOA_BASE              (AHB1PERIPH_BASE + GPIOA_OFFSET)

#define GPIOD_OFFSET            (0x0C00UL)
#define GPIOD_BASE              (AHB1PERIPH_BASE + GPIOD_OFFSET)

#define RCC_OFFSET              (0x3800UL)
#define RCC_BASE                (AHB1PERIPH_BASE + RCC_OFFSET)

#define GPIOAEN                 (1U << 0)
#define GPIODEN                 (1U << 3)  // Bit 3 in RCC AHB1ENR enables GPIOD clock

// ==============================
// Typedef Struct Definitions
// ==============================

#define __IO volatile  // Mark registers as volatile (prevent compiler optimization)

// Structure for GPIO peripheral registers
typedef struct {
	__IO uint32_t MODER;     // Mode register: sets pin direction (input/output)
	__IO uint32_t OTYPER;    // Output type register
	__IO uint32_t OSPEEDR;   // Output speed register
	__IO uint32_t PUPDR;     // Pull-up/pull-down register
	__IO uint32_t IDR;       // Input data register
	__IO uint32_t ODR;       // Output data register (not used here)
	__IO uint32_t BSRR;      // Bit set/reset register
	__IO uint32_t LCKR;      // Lock register
	__IO uint32_t AFRL;      // Alternate function low
	__IO uint32_t AFRH;      // Alternate function high
} GPIO_TypeDef;

// Structure for RCC (clock control)
typedef struct {
	__IO uint32_t CR;
	__IO uint32_t PLLCFGR;
	__IO uint32_t CFGR;
	__IO uint32_t CIR;
	__IO uint32_t AHB1RSTR;
	__IO uint32_t AHB2RSTR;
	__IO uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__IO uint32_t APB1RSTR;
	__IO uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__IO uint32_t AHB1ENR;    // Enables AHB1 peripherals (GPIOD here)
	__IO uint32_t AHB2ENR;
	__IO uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__IO uint32_t APB1ENR;
	__IO uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__IO uint32_t AHB1LPENR;
	__IO uint32_t AHB2LPENR;
	__IO uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__IO uint32_t APB1LPENR;
	__IO uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__IO uint32_t BDCR;
	__IO uint32_t CSR;
	uint32_t RESERVED6[2];
	__IO uint32_t SSCGR;
	__IO uint32_t PLLI2SCFGR;
	__IO uint32_t PLLSAICFGR;
	__IO uint32_t DCKCFGR;
} RCC_TypeDef;

// Simple enum for digital pin states
typedef enum {
	RESET,
	SET
} pin_status;

// ==============================
// Peripheral Access Macros
// ==============================

#define RCC     ((RCC_TypeDef *) RCC_BASE)
#define GPIOD   ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOA   ((GPIO_TypeDef *) GPIOA_BASE)

// ==============================
// GPIO Pin Definitions
// ==============================

// Each pin represented as a 16-bit mask (corresponds to ODR/BSRR bit positions)
#define GPIO_PIN_0  ((uint16_t)0x0001)
#define GPIO_PIN_12  ((uint16_t)0x1000)  // LD4 (Green)
#define GPIO_PIN_13  ((uint16_t)0x2000)  // LD3 (Orange)
#define GPIO_PIN_14  ((uint16_t)0x4000)  // LD5 (Red)
#define GPIO_PIN_15  ((uint16_t)0x8000)  // LD6 (Blue)

// ==============================
// Function Declaration
// ==============================

void GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, pin_status status);

// ==============================
// Main Function
// ==============================

int main(void) {
	// 1. Enable GPIOD clock (bit 3 of AHB1ENR)
	RCC->AHB1ENR |= GPIODEN;
	RCC->AHB1ENR |= GPIOAEN;
	// 2. Configure GPIOD pins 12–15 as general-purpose output (MODER = 01 for each)
	GPIOD->MODER |= (1U << 24);  // Pin 12
	GPIOD->MODER |= (1U << 26);  // Pin 13
	GPIOD->MODER |= (1U << 28);  // Pin 14
	GPIOD->MODER |= (1U << 30);  // Pin 15
	GPIOA->MODER &= ~(0x3 << 0);

	// 3. Main loop: turn LEDs on one by one, then off one by one
	while (1) {

		if(GPIOA->IDR & GPIO_PIN_0){
		// Turn ON each LED with a delay
		GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);  // Green
		for (volatile int i = 0; i < 200000; i++);

		GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);  // Orange
		for (volatile int i = 0; i < 200000; i++);

		GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);  // Red
		for (volatile int i = 0; i < 200000; i++);

		GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);  // Blue
		for (volatile int i = 0; i < 200000; i++);

		// Turn OFF each LED with a delay
		GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		for (volatile int i = 0; i < 200000; i++);

		GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
		for (volatile int i = 0; i < 200000; i++);

		GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
		for (volatile int i = 0; i < 200000; i++);

		GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
		for (volatile int i = 0; i < 200000; i++);
		}
	}
}

// ==============================
// GPIO Write Function Definition
// ==============================

/**
 * @brief  Writes a digital value to a GPIO pin.
 *         Uses the BSRR register for atomic bit-level access.
 *
 * @param  GPIOx      Pointer to the target GPIO port (e.g., GPIOD)
 * @param  GPIO_Pin   Pin mask (e.g., GPIO_PIN_13)
 * @param  status     SET or RESET
 */
void GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, pin_status status) {
	if (status == SET) {
		GPIOx->BSRR = GPIO_Pin;            // Lower 16 bits: set pin
	} else {
		GPIOx->BSRR = (GPIO_Pin << 16);    // Upper 16 bits: reset pin
	}
}
