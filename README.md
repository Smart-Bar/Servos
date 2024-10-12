
# SmartBar Servos

Servo motor controllers for the SmartBar project. Designed to be deployed on a **_STM32F303RE_** microcontroller.

The servo motors used for this project are the **_SG90_**. For further information, please refer to the corresponding [datasheet](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf).

## Microcontroller Configuration:

To adjust the microcontroller output frequency to the 50Hz operating frequency of the SG90 servo motors, the following configuration was used:

- TIM3: PWM Generation CH2
    - Clock Frequency: 72 MHz
    - Prescaler: 5625
    - Auto-Reload Register: 255

## Deployment:

Consider using the **_SMT32 VS Code Extension_**, since the project was created using the **_STM32CubeMX_** software and configured **_CMake_** as toolchain to build the project.

Clone the repository and open the project on **_VS Code_** and add it via the STM32 extension. Then, select `Build + Flash` task in the `Run Task` from the `Terminal` tab. Or select the `Build & Debug Microcontroller` option from the `Run and Debug` tab.

## Usage:

The program is designed to control the servo motors using the terminal via **_USART2_** peripheral, which corresponds to the board's **_ST-Link_** port.

To toggle the board's built-in LED:

    LED

To control the servo motor, the following commands are available:

    PWM <duty_cycle>
    PWM -p <percentage>

For example, to set the servo motor to a 90° position using a percentage:

    PWM -p 50

Alternatively, to set it to a 45° position using a duty cycle value:

    PWM 5

Since the servo motor requires a pulse width between **_1ms and 2ms_** to work as expected, the duty cycle should be between **_5% and 10%_**.

The implementation of the PWM handles this by clamping the values to a range between 2.5 (0° or 1ms) and 12.5 (180° or 2ms). [Source](https://apmonitor.com/dde/index.php/Main/ServoControl#:~:text=The%20SG90%20servo%200°,(range%200-1023).)

This values can be calculated using the following formula:

$$ CCR = ARR \cdot \frac{\text{Duty Cycle}}{100} $$

Where **_CCR_** is the Capture/Compare Register and **_ARR_** is the Auto-Reload Register, the latter set to 255.

Which derives from the equation:

$$ \text{Pulse Width} = T \cdot \frac{\text{Duty Cycle}}{100} $$

Where **_P_** is the pulse width, **_T_** is the period of the PWM signal and **_DC_** is the duty cycle.
