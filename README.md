# ElevatorModel-2.0
Software and hardware for elevator model (STM32F3)

![](https://badgen.net/badge/Status/Completed/green?icon=github)
![](https://badgen.net/badge/Version/2.0.0/cyan?icon=bitcoin-lightning)

<p align="center">
  <img src="https://github.com/DeltaVetal26/ElevatorModel-2.0/blob/main/readmeImages/demo.gif?raw=true">
</p>

The first version of the elevator controller was assembled on the Arduino Pro Mini. Reed switches were used as floor sensors. The AC motor was controlled via a relay block.

The first version often freeze due to the lack of galvanic isolation

It was decided to develop a new version of the controller using galvanic isolation and based on the STM32F334K8T6 microcontroller.

Features:
1) The inputs on the board are isolated via optocouplers
2) The I2C bus responsible for outputting the current floor and reading the buttons is isolated through a digital isolator
3) A stepper motor is used as a drive
4) Optocouplers are used as floor sensors
5) The board has a step down converter (12V/3V3)


<p align="center">
  <img src="https://github.com/DeltaVetal26/ElevatorModel-2.0/blob/main/readmeImages/Controller_V1-min.jpg?raw=true" width="500" height="667">
  <img src="https://github.com/DeltaVetal26/ElevatorModel-2.0/blob/main/readmeImages/ControllerV2-min.jpg?raw=true" width="500" height="667">  
</p>
<p align="center">
<img src="https://github.com/DeltaVetal26/ElevatorModel-2.0/blob/main/readmeImages/MainV.jpg?raw=true" width="500" height="1162">
<img src="https://github.com/DeltaVetal26/ElevatorModel-2.0/blob/main/readmeImages/FloorIndicator-min.jpg?raw=true" width="600" height="809">
</p>

