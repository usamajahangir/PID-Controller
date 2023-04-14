# PID-Controller
PID Controller for abstacle avoiding robot

## PID Specifications
- Zero Steady State error <br>
- Settling time (Ts) less than 0.25s<br>
- Overshoot (Mp) less than 5%<br>
 
## Specifications
Microcontroller: ESP32<br>
Feedback Module: Digital IR sensors (5)<br>
Other Module: LCD, LM298 <br>

## Working
Five digital IR sensors are used for poition and obstacle feedback of robot which then is sent to ESP32 and PID is implimented finally the results are displayed on the LCD.
