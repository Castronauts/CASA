# ARLO GUI Control

##### The Matlab `ARLO_GUI.mlapp` file contains the code used to generate a GUI for controlling ARLO.

- The commands created and sent in the GUI follow the format shown in `ArloControl.cpp`.

##### `ArloControl.c` and `ArloRobot.cpp` contain the code used to interface with the [DHB-10] motor controller.

- The C++ file was created by [Parallax] (as an Arduino library) to be used with an Arduino and their motor controller.

- `ArloControl.c` is Arduino example code that uses the serial port connection to a computer to send commands to the motor controller. (`ArloRobot.cpp` should be referenced for the correct format of commands)

[DHB-10]: https://www.parallax.com/product/28231
[Parallax]: https://www.parallax.com
