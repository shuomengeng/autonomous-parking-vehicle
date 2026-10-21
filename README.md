3.2 PCB Design and Integration
3.21 PCB Design and Components 
The PCB was designed to connect the main electronic components of the Smart Self-Parking Vehicle (SSV). These included an ESP32 microcontroller, a XIAO ESP32S3 Sense camera, a dual motor driver, four distance sensors, two DC gear motors, two microservo motors, and two voltage regulators.
The ESP32 controlled the motors and servos, while the camera provided images for parking decisions. The four distance sensors measured the distance to nearby objects, and the motor driver controlled the two DC motors.
The main aim of the PCB design was to organise the power supply, provide reliable connections, reduce unnecessary wiring, and fit the components into the vehicle.
3.22 Power Supply and Ground Design 
The vehicle was powered by two 18650 lithium-ion batteries connected in series, providing a nominal voltage of 7.4 V. Two voltage regulators were used to provide 5 V power to different groups of components. One supplied the ESP32 and camera, while the other supplied the motor driver and servos. The Pololu S13V30F5 regulator was used, with a maximum rated output current of 3 A under suitable operating conditions.
The four distance sensors were powered by the ESP32 development board's 3.3 V supply. A common ground network connected the main circuits and provided a shared voltage reference.
3.23 PCB Layout and Trace Widths 
The PCB used a two-layer layout with copper ground pours to provide ground connections across the board. The layout also considered the separation of motor power connections from sensitive sensor signals.
Different trace widths were used for different parts of the PCB. The main power paths used 60 mil traces, the regulated power paths used 30 mil traces, and the motor connections used 50 mil traces. Signal traces used a width of 15 mil.
The PCB used 1 oz copper. A preliminary current-capacity check was based on the IPC-2221 formula, assuming external copper traces and a temperature rise of 10°C. Under these assumptions, the estimated current capacities were approximately 3.2 A for 60 mil traces, 2.0 A for 30 mil traces, and 2.9 A for 50 mil traces. These values were theoretical estimates rather than measured results.

 
Fig. 6. PCB
3.24 GPIO Connections and PCB Integration 
The ESP32 GPIO pins were assigned according to the control requirements. GPIO 14 and GPIO 19 were used for motor control, while GPIO 26 controlled the servo signal. GPIO 33, GPIO 32, GPIO 12, and GPIO 17 were connected to the four distance sensors.
The signal traces connected the ESP32 to the motor driver, servos, and distance sensors. The camera and sensors were positioned according to the vehicle layout so they could face the correct directions.
Jumper wires were used to connect the rear distance sensors because of their physical positions at the back of the vehicle. This provided flexibility during assembly. The completed PCB was integrated into the SSV and connected to the main electronic components.
3.25 Testing and Evaluation 
Repeated parking tests were carried out using the assembled vehicle. During testing, no noticeable heating of the PCB traces was observed. This suggested that the traces performed well under the tested operating conditions. However, the motor and servo currents were not measured, so the current-carrying capacity of all traces could not be fully verified.
The tests showed that the PCB could be used as part of the integrated SSV. However, the parking performance depended on the complete vehicle system, so the individual contribution of the PCB could not be measured separately.
 Fig. 7. The PCB schematic
 
Fig. 8. The PCB layout
3.3 Overall System Integration
The final vehicle combines the mechanical chassis, PCB, ESP32, camera, distance sensors, motor driver, motor, and servo. Each system has a different role, which each is required for the vehicle to complete the task and park autonomously.
The ESP32-DevKitC-32E is the main controller on the board. It receives state machine commands from the laptop, reads distance sensors, adjusts the servo for steering, and controls the direction of the motors through the DRV8834 motor driver. The XIAO ESP32-S3 provides the camera stream used by the laptop to detect parking spots.
The laptop and car communicated over Wi-Fi using UDP communication on port 3333.  The laptop sends commands that are numbers which correspond to DRIVE, ALIGN, PARK, UNPARK, and STOP. It also requests sensor readings and receives status updates from the ESP32, including when parking or unparking is complete. Before all movements begin, the laptop sends the parking and steering parameters to the ESP32 and waits for acknowledgement, which prevents the program from starting the ESP32 before confirming the settings.
The batteries are connected in series, providing a power supply of approximately 8V when tested with a multimeter. Voltage regulators provide the required supply of 5V traces for electronic components. The PCB connects to the main electronic components and reduces the need for jumper wires, although some jumper wires were needed during assembly.
