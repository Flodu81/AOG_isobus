Code to use the Keya encoder instead of the angle sensor. To synchronize the encoder and the actual wheel angle, the WAS (Wheel Angle Sensor) automatically resets to 0 based on the GPS and IMU headings. To activate Keya WAS mode, you must first activate Danfoss mode. To return to normal mode, deactivate it. Then, an initial zeroing is performed at startup; until this is complete, guidance is not possible. Afterward, it continuously zeroes based on the GPS and IMU headings. Tolerances are adjustable via a web interface.

Here is the link to the basic PCB: https://github.com/buched/Ecu_box_v2.01_autosteer4.1_ecu

How the web interface works: 
To access it, enter the PCB's IP address in your browser. 
The yaw rate (BNO) is the maximum course change allowed for zeroing. 
The same applies to the GPS heading. If you find that zeroing is too easily achieved, lower these values.

PCB de base
<img width="3060" height="4080" alt="20260428_105213" src="https://github.com/user-attachments/assets/034539bc-0827-4e93-bcbd-d16263b01353" />
<img width="3456" height="4608" alt="20260223_144233" src="https://github.com/user-attachments/assets/6e946b45-2e5e-4752-becd-a2a39db2c7ec" />

Link to the CAN board for the section control

Carte d'expansion IO de PLC de Modbus RTU NPN/PNP entrée numérique LilContrmatérielle DC 12V/24V 4-16CH DI-DO LilFibandbus cite Tech RS485
https://a.aliexpress.com/_ExNAeaw

From 4 to 16 relays and NPN or PNP input 

<img width="1079" height="463" alt="1000005454" src="https://github.com/user-attachments/assets/2ad86f5f-3ffe-409f-af55-76c547381ec2" /> 
