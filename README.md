# stm32_arm-none-eabi-gcc

using cubemx and arm-none-eabi-gcc to develop stm32 project(with rtos and without rtos). stm32开发的全开源解决方案。

cd F103ZET

make

用于 STM32 产品编程的 STM32CubeProgrammer 软件
(https://www.st.com/resource/en/data_brief/stm32cubeprog.pdf "Download datasheet")

STM32Cube 初始化代码生成器
(https://www.st.com/resource/en/data_brief/stm32cubemx.pdf "Download datasheet")

所有外设硬件的初始化代码都可以通过去抄 CubeMX 自动生成的初始化代码。重要的是搞懂硬件逻辑，比如硬件连接的是哪个 GPIO 口，板子上一共有几组 GPIO，几个 PIN 供你使用。
