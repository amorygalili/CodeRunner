# Chapter 3: Application advice

## 3.1 Mechanical vs software solutions

While this book focuses on controls engineering applications in FRC, controls isn’t necessary to have a successful robot; FRC is primarily a mechanical competition, not a software competition. We spend over six weeks designing and building a robot, but we often allocate just two days for software testing. Despite this, teams still field competitive robots, which is a testament to how simple and easy to use the FRC software ecosystem is nowadays. Focus on a reliable mechanical design first because a good mechanical design can succeed with simple software, but the robot can’t succeed with unreliable hardware.

The solution to a design problem may be a tradeoff between mechanical and software complexity. For example, for a mechanism that only needs two positions, a solenoid connected to a pneumatic actuator may take less effort and be more reliable than a motor, rotary encoder, and software feedback control. If one can get the software solution working though, the robot may not need the added space and weight of a compressor and air tanks.

My rule of thumb for evaluating designs is to prefer elegant mechanical solutions over comprehensive software solutions because it’s easier to make mechanical solutions reliable in competition. Well-placed sensors can also drastically improve robot performance and reduce driver cognitive load. An example would be a limit switch and match timer for automatically deploying minibots in the 2011 FRC game as soon as the endgame starts. In many cases, manual processes can be automated later and given a manual fallback if the associated software or sensor fails. Be cautious with designs that require closed-loop control to function.

When should problems be solved in hardware instead of software with clever controls? Controls can handle disturbances like battery voltage drop or measurement noise, but there are limits. For example, there’s nothing software can do to work around a drivetrain gearbox seizing up or throwing a chain. Sometimes, you’re better off just fixing the root cause in hardware.

Design robot mechanisms for controllability. FRC team 971’s “Mechanical Design for Controllability” seminar goes into more detail. Two of the important takeaways from it are:

- Reduce gearbox backlash

- Choose motors and gear reductions that provide adequate control authority

> **Video:** [“Spartan Series / Mechanical Design for Controllability” (1 hour, 31 minutes)](https://youtu.be/VNfFn-gcfFI) — Travis Schuh

Remember, “fix it in software” isn’t always the answer. The remaining chapters of this book assume you’ve done the engineering analysis and concluded that your chosen design would benefit from more sophisticated controls, and you have the time or expertise to make it work.

## 3.2 Actuator saturation

A feedback controller calculates its output based on the error between the reference and the current state. Plant in the real world don’t have unlimited control authority available for the feedback controller to apply. When the actuator limits are reached, the feedback controller acts as if the gain has been temporarily reduced (i.e., it will have reduced control authority).

We’ll try to explain this through a bit of math. Let’s say we have a controller $u = k(r - x)$ where $u$ is the control effort, $k$ is the gain, $r$ is the reference, and $x$ is the current state. Let $u_{max}$ be the limit of the actuator’s output which is less than the uncapped value of $u$ and $k_{max}$ be the associated maximum gain. We will now compare the capped and uncapped controllers for the same reference and current state.

$$
\begin{aligned}
u_{max} &< u \\
k_{max}(r - x) &< k(r - x) \\
k_{max} &< k
\end{aligned}
$$

For the inequality to hold, $k_{max}$ must be less than the original value for $k$.
