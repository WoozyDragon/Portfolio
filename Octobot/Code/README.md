A collection of code, both for this year's robot and last year's. All code is written entirely by me. 

List of files:
- Arepikov_Mechanum_Teleop.java: The main driver control file for last year. The main loop starts on line 189.
- Arepikov_Mechanum_Auton.java: The main autonamous file for last year. There were other copies with minor changes for any of the four possible starting positions (red or blue, caruousel or not). The main loop starts on line 453, and above that are the motor control, setup, and miscellaneous functions. Of note are the motor access functions on lines 244-417. These are quite heavily commented. They are designed to be used as blocks, allowing the programmer to focus on writing the code instead of how to write it. They worked very well last year, allowing the design of a full autonamous program in less than two hours. 
- Octotest.java: The current code to control octobot (this year's robot). The main loop starts on line 33. Most features aren't implemented yet (such as lift control) because the robot isn't finished. Eventually this will have a similar amount of (hopefully better) code to Arepikov_Mechanum_Teleop.java
