# Ball-Balancing-Robot-KALMAN-FILTER-MPU6050-
BALL BALANCE BOT Can be used with Arduino ESP32 ARMs by changing configuration

#Notice
You cannot just copy paste the code

First of all change pinouts accordingly to your board and be mindful not to install pwn analog pin towards digital output pins.
Then, you need to build body and check the steadystate's Roll (X) and pitch (Y).


Thanks to kalman filter, you dont need to calibrate the mcu6050 like you did in various tutorials from internet. Here, you get accurate real time feedback and use it to control the loop.


AFter that go to loop and change the roll and pitch reading for motor control accordingly by replacing threshold values.(50)in.program.

#BODY Architecture
WE have 2 wheel in parallel (180 degreees apart) and between them, is the third motor (90 Dergrees apart between). on the opposing side is the rocker wheel without motor

If your ball balancer is setup of 66.6 degress apart separated wheels architecture, include a logic for motor 1 and motor 2 going opposite side (USE IMNI WHEELS or you cant build 66.6 degree configuration)
