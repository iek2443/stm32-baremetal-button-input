# STM32 Bare-Metal: Button-Controlled LED Blink

---

##🧭 Summary

This project demonstrates how to use a **push button input (GPIOA pin 0)** on the STM32F4 Discovery board to trigger an LED blinking sequence.  
When the user button is pressed, all 4 onboard LEDs (LD3–LD6) are turned on and off sequentially.  
This example introduces **GPIO input reading** and shows how to combine it with structured output control using `GPIO_WritePin()`.

---

## 🔁 Previous Lesson

If you haven’t completed the previous step where we used `GPIO_WritePin()` and the `BSRR` register for structured LED control, check it out first:

👉 [Previous Lesson: GPIO Control using BSRR and GPIO_WritePin](https://github.com/iek2443/stm32-baremetal-gpio-bsrr)
---

## 🧠 What You Will Learn

- How to configure GPIO pins as digital input and output
- How to read the state of a push button using the `IDR` register
- How to conditionally control outputs based on input state
- How to modularize GPIO logic using a reusable `GPIO_WritePin()` function

---

## ⚙️ Key Registers Used

- `RCC->AHB1ENR` → Enables clocks to GPIOA and GPIOD
- `GPIOA->MODER` → Configures GPIOA pin 0 as input
- `GPIOA->IDR`   → Reads the state of pin 0 (user button)
- `GPIOD->MODER` → Configures pins 12–15 as output
- `GPIOD->BSRR`  → Sets and resets LED pins atomically

---

## 🔧 Requirements

- STM32F4 Discovery Board
- ARM GCC Toolchain
- ST-Link programmer
- STM32CubeProgrammer or OpenOCD
- USB Mini-B cable

---

📁 Project Structure
--------------------

stm32-blink-led/\
├── src/\
│   └── main.c         --> Bare-metal LED toggle code\
├── inc/               --> (Optional: header files)\
└── README.md

💡 LED and Button Info (STM32F4 Discovery)

| Function     | Port | Pin | Description         |
|--------------|------|-----|---------------------|
| User Button  | A    |  0  | Reads input state   |
| LD4 (Green)  | D    | 12  | Output LED          |
| LD3 (Orange) | D    | 13  | Output LED          |
| LD5 (Red)    | D    | 14  | Output LED          |
| LD6 (Blue)   | D    | 15  | Output LED          |
