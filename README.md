# Arboc
# Areas of implementation:
  •	Underwater surveillance
  
  •	Ship propeller/lower deck inspection.
  
  •	Detecting oil spills in Semi-submersible/Off shore oil drilling platforms. 
# Platforms to develop novel solutions:
  •	Implementing Reinforcement learning for gait generation of the snake robot and to study the optimal gait for goal-based locomotion.
  
  •	Underwater path planning.
  
  •	Optimal control algorithms with feedback from IMUs for snake robots.
# Challenges:
  •	To find an innovative way to wirelessly communicate underwater. If the developed communication system is not robust and unreliable,
  tethers can be used as most underwater surveillance vehicles are tethered.
  
  •	Gait generation and mathematical modelling.
  
  •	Fabrication.
# Progress till date:
  •	Designed/Developed a 3D CAD model of the snake-robot using Autodesk’s Fusion 360.
  
  ![alt text](https://github.com/imsenthur/Arboc/blob/master/CADmodel.png)
  
  •	Simulation of a minimalistic design of the bot has been done with V-rep.
  
  •	3D printed all the individual parts and assembled it together.
  
  ![alt text](https://github.com/imsenthur/Arboc/blob/master/Gaits.png)
  
# Things to be done:
  •	Feedback control with IMUs [Each module holding one].

  •	Fabrication of the bot, almost 80% of the bot is already waterproof by design. The head and tail modules need to water proofed.

  •	Eliminate the Bluetooth communication and come up with some other way of underwater communication or prepare the tail of the bot
    ready to be tethered.

  •	Eliminate the use of micro controller which makes it nearly impossible to implement reinforcement learning and autonomous path planning.
  
# Components to be used:
  •	Beaglebone Blue:
  
    • Battery: 2-cell LiPo support with balancing, 9-18V charger input.
  
    • Features 9-axis IMU and barometer sensors.
    
    • Wi-Fi 802.11 b/g/n, Bluetooth 4.1 and BLE enabled.
    
    • 8x 6V servo outputs [Eliminates the need for MCU]
    
    • Better than Beaglebone black as it has the robotic cape integrated within it. In case of using Beaglebone black, robotics cape 
      needs to be bought separately which would again be nearly impossible to accommodate.
    
    • Can be easily accommodated in the tail of the bot. [RPI3 was bulkier and needed a MCU which was nearly impossible to accommodate.]
    
  •	IMU - MPU6050: Inertial measurement unit for accelerometer and gyroscpe data inoder to implement PID control over the bot. 
  
  •	Ultrasound Range Finder: For collision avoidance.
  
  •	Endoscope camera: To image underwater pipelines and to retrieve frames for further image processing.

  
