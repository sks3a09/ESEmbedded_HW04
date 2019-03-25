HW04
===
This is the hw04 sample. Please follow the steps below.

# Build the Sample Program

1. Fork this repo to your own github account.

2. Clone the repo that you just forked.

3. Under the hw04 dir, use:

	* `make` to build.

	* `make flash` to flash the binary file into your STM32F4 device.

	* `make clean` to clean the ouput files.

# Build Your Own Program

1. Edit or add files if you need to.

2. Make and run like the steps above.

3. Please avoid using hardware dependent C standard library functions like `printf`, `malloc`, etc.

# HW04 Requirements

1. Please practice to reference the user manuals mentioned in [Lecture 04], and try to use the user button (the blue one on the STM32F4DISCOVERY board).

2. After reset, the device starts to wait until the user button has been pressed, and then starts to blink the blue led forever.

3. Try to define a macro function `READ_BIT(addr, bit)` in reg.h for reading the value of the user button.

4. Push your repo to your github. (Use .gitignore to exclude the output files like object files, executable files, or binary files)

5. The TAs will clone your repo, build from your source code, and flash to see the result.

[Lecture 04]: http://www.nc.es.ncku.edu.tw/course/embedded/04/

--------------------

- [ ] **If you volunteer to give the presentation (demo) next week, check this.**

--------------------

Take your note here if you want. (Optional)

HW04
===

1.實驗題目
---

After reset, the device starts to wait until the user button has been pressed, and then starts to blink the blue led forever.

2.Control Buttons and LEDs on the STM32F4DISCOVERY
---

**1. LEDs on the STM32F4DISCOVERY**

1) To light up the LED on the STM32F4DISCOVERY, we have to know which GPIO pin the LED was connected to.

2) We can find this in [UM1472 User manual Discovery kit with STM32F407VG MCU](http://www.nc.es.ncku.edu.tw/course/embedded/pdf/STM32F4DISCOVERY.pdf): section 6.3 (p.16), which defines the connections of LEDS.

3) Here we will use the **blue LED(LD6)** in the HW04 program, so we have to control the GPIO pin **PD15(15th pin of GPIOD)**.

![](https://i.imgur.com/GvoEK5E.png)

**2. Push buttons on the STM32F4DISCOVERY**

1) To use push button on the STM32F4DISCOVERY, we have to know which GPIO pin the LED was connected to.

2) We can find this in [UM1472 User manual Discovery kit with STM32F407VG MCU](http://www.nc.es.ncku.edu.tw/course/embedded/pdf/STM32F4DISCOVERY.pdf): section 6.3 (p.16), which defines the connections of Push buttons.

3) Here we will use the **User and Wake-Up buttons** in the HW04 program, so we have to control the GPIO pin **PA0(0th pin of GPIOA)**.

![](https://i.imgur.com/Ccukb6G.png)

**3. General-purpose I/Os (GPIO)**

1) We can find hardware configuration details in [RM0090 Reference manual STM32F407](http://www.nc.es.ncku.edu.tw/course/embedded/pdf/STM32F407_Reference_manual.pdf)

2) In section 2.3(p.64), there is a **memory map of peripherals**.

3) The **base address of GPIOD is** ```0x40020C00``` and the details are described in section 8(p.267).

4) The **base address of GPIOA is** ```0x40020000``` and the details are described in section 8(p.267).

5) we will have to setup **RCC(Reset and clock control) register** to enable the clock for GPIOD and GPIOA.

6) The **base address of RCC is** ```0x40023800``` and the details are described in section 7(p.213).

![](https://i.imgur.com/P49hGpn.png)

**4. RCC - GPIO Enable**

1) To enable GPIOD, [RM0090 Reference manual STM32F407](http://www.nc.es.ncku.edu.tw/course/embedded/pdf/STM32F407_Reference_manual.pdf) tells us to **set the bit3 at** ```0x40023800 + 0x30```. The address ```0x40023800``` is defined in the memory map.

![](https://i.imgur.com/NK2d7OC.png)
![](https://i.imgur.com/NGJDZrQ.png)

2) To enable GPIOA, [RM0090 Reference manual STM32F407](http://www.nc.es.ncku.edu.tw/course/embedded/pdf/STM32F407_Reference_manual.pdf) tells us to **set the bit0 at** ```0x40023800 + 0x30```. The address ```0x40023800``` is defined in the memory map.

![](https://i.imgur.com/XEMRtSg.png)
![](https://i.imgur.com/twN64PE.png)

**5. GPIO Configurations**

1) Section 8.3 described that each of the GPIOs has four 32-bit memory-mapped control registers:
* MODER (Mode of pins operation)
* OTYPER (Mode for pin’s output type)
* OSPEEDR (Select GPIO speed)
* PUPDR (Select pull resistors or disable it)

2) GPIOA0 ~ GPIOA15, GPIOB0 ~ GPIOB15, … , GPIOK0 ~ GPIOK15, **evrey pin** has its own settings.

![](https://i.imgur.com/J2aOCRW.png)

![](https://i.imgur.com/lDCi5QF.png)

3) We want the pin GPIOD15 to be output PP(push pull) without pull-up or pull-down resistors, so we have to configure the GPIOD:
* ```MODER(15) = 01```
* ```OTYPER(15) = 0```
* ```OSPEEDER(15)``` can be ignored (update frequency is not that important in this case.)
* ```PUPDR(15) = 00```

4) We want the pin GPIOA0 to be input floating without pull-up or pull-down resistors, so we have to configure the GPIOA:
* ```MODER(0) = 00```
* ```OTYPER(0) = x```
* ```OSPEEDER(0) = xx``` 
* ```PUPDR(0) = 00```

5) To control GPIO output, we can use **GPIO port output data register (GPIOx_ODR)** or **GPIO port bit set/reset register (GPIOx_BSRR)**.

![](https://i.imgur.com/RMxHNI8.png)
![](https://i.imgur.com/sZuRz3x.png)

6) To control GPIO input, we can use **GPIO port input data register (GPIOx_IDR)**.

![](https://i.imgur.com/epsAZUd.png)

3.實驗步驟
---

**1. 初始化Button**

```c
SET_BIT(RCC_BASE + RCC_AHB1ENR_OFFSET, GPIO_EN_BIT(GPIO_PORTA));

//MODER led pin = 00 => input mode
CLEAR_BIT(GPIO_BASE(GPIO_PORTA) + GPIOx_MODER_OFFSET, MODERy_1_BIT(BUTTON));
CLEAR_BIT(GPIO_BASE(GPIO_PORTA) + GPIOx_MODER_OFFSET, MODERy_0_BIT(BUTTON));

//OT led pin = x 
//OSPEEDR led pin = xx 

//PUPDR led pin = 00 => No pull-up, pull-down
CLEAR_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_PUPDR_OFFSET, PUPDRy_1_BIT(BUTTON));
CLEAR_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_PUPDR_OFFSET, PUPDRy_0_BIT(BUTTON));
```

**2. 初始化LED**
```c
SET_BIT(RCC_BASE + RCC_AHB1ENR_OFFSET, GPIO_EN_BIT(GPIO_PORTD));

//MODER led pin = 01 => General purpose output mode
CLEAR_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_MODER_OFFSET, MODERy_1_BIT(led));
SET_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_MODER_OFFSET, MODERy_0_BIT(led));

//OT led pin = 0 => Output push-pull
CLEAR_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_OTYPER_OFFSET, OTy_BIT(led));

//OSPEEDR led pin = 00 => Low speed
CLEAR_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_OSPEEDR_OFFSET, OSPEEDRy_1_BIT(led));
CLEAR_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_OSPEEDR_OFFSET, OSPEEDRy_0_BIT(led));

//PUPDR led pin = 00 => No pull-up, pull-down
CLEAR_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_PUPDR_OFFSET, PUPDRy_1_BIT(led));
CLEAR_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_PUPDR_OFFSET, PUPDRy_0_BIT(led));
```

**3. create a macro function READ_BIT(addr, bit)**

```
#define READ_BIT(addr, bit) (REG(addr) >> (bit) & UINT32_1 )
```

**4. define GPIO_IDR_OFFSET**

```
#define GPIOx_IDR_OFFSET 0x10
#define IDRy_BIT(y) (y)
```
**5. 設置while迴圈，當按下user button時，藍燈閃爍**

```
while(1){
    if(READ_BIT(GPIO_BASE(GPIO_PORTA) + GPIOx_IDR_OFFSET,IDRy_BIT(BUTTON))){
        blink(LED_BLUE);
    }
    else{
        
    }
}
```
