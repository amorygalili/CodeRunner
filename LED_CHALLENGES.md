## LED Challenges

I'd like to make a lesson module for teaching programming to students by having them program LEDs. I'd like to structure the robot code kind of like this:

src\main\java\frc\robot\subsystems\LEDS

Each challenge will be its own class that implements the Led interface. Students should only have to update the code in these files to complete the challenges. There should be challenges that cover the very basics of programming, and get progressively more challenging. The focus should be less on learning WPILib and more learning programming through writing robot LED code.

The repo with the lesson modules is in C:\repos\robotics\CodeRunner-Lessons. Please add a new lesson module as well as a custom web dashboard which contains an interface for running each led challenge. It should have something to select the challenge to run, a button to run it, something to visualize the running LEDs, and a description of the selected challenge with helpful information like what file they need to edit, what the challenge is intended to teach, functions they might need to use, documentation, etc.

To get LED data halsim data might be needed. I believe theres code in this repo already which allows the frontend to get this data.