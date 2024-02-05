# robotic-maze-ecape

## Project Overview

For this project me and my teammates customized and programmed a set of Romi robots to work together to escape from a grid-based maze that uses a variety of indicators, like an infrared beacon and Apriltags, to help one of the robots find the escape door. We customized three robots: the first one with an infrared light position finder (IR Positioning Camera), the second one with a visual camera (Open MV Camera), and the third with an infrared light emitter to allow it to open the escape door. For the final demonstration, we placed each robot at the designated starting area – cell (0,0) – and started them on their tasks pressing a button on the IR remote. After receiving the start command, the first robot with the IR Positioning Camera will drive around the maze to find the IR beacon. Once it finds it, it will communicate to the second robot with the Open MV Camera, using an MQTT broker connected to the ESP32 on each robot, the column of the arena on which the ramp is. Once the second robot receives this information it will start driving first at the right column and then up the ramp. Once it reaches the top of the ramp, it will communicate to the first robot that it's in position to read the AprilTag, which will indicate the position of the escape door, so the first robot will press the button and the second one will read the AprilTag. At this point the first robot will send the location of the IR beacon, which determines the door code that will be used to open the door, and the second robot will send the position of the escape door to the third robot, which will drive to the escape door, flash the code, and escape.

See the system in the demo video below created and edited by my teammate Shivangi Sirsiwal:

https://github.com/Shivangi-Sirsiwal/tri-robot_maze-solving_system/assets/152037538/4449ab9a-d629-4a69-9218-d85e56ac5b8f

