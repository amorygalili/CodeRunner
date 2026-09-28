# Chapter 0: Notes to the reader

## 0.1 Prerequisites

Knowledge of basic algebra and complex numbers is assumed. Some introductory physics and calculus will be taught as necessary.

## 0.2 Structure of this book

This book consists of five parts and a collection of appendices that address the four tasks a controls engineer carries out: derive a model of the system (kinematics), design a controller for the model (control theory), design an observer to estimate the current state of the model (localization), and plan how the controller is going to drive the model to a desired state (motion planning).

Part I “Fundamentals of control theory” introduces the basics of control theory and teaches the fundamentals of PID controller design.

Part II “Modern control theory” first provides a crash course in the geometric intuition behind linear algebra and covers enough of the mechanics of evaluating matrix algebra for the reader to follow along in later chapters. It covers state-space representation, controllability, and observability. The intuition gained in part I and the notation of linear algebra are used to model and control linear multiple-input, multiple-output (MIMO) systems and covers discretization, LQR controller design, LQE observer design, and feedforwards. Then, these concepts are applied to design and implement controllers for real systems. The examples from part IV are converted to state-space representation, implemented, and tested with a discrete controller.

Part II also introduces the basics of nonlinear control system analysis with Lyapunov functions. It presents an example of a nonlinear controller for a unicycle-like vehicle as well as how to apply it to a two-wheeled vehicle. Since nonlinear control isn’t the focus of this book, we mention other books and resources for further reading.

Part III “Estimation and localization” introduces the field of stochastic control theory. The Luenberger observer and the probability theory behind the Kalman filter is taught with several examples of creative applications of Kalman filter theory.

Part IV “System modeling” introduces the basic calculus and physics concepts required to derive the models used in the previous chapters. It walks through the derivations for several common FRC subsystems. Then, methods for system identification are discussed for empirically measuring model parameters.

Part V “Motion planning” covers planning how the robot will get from its current state to some desired state in a manner achievable by its dynamics. It introduces motion profiles with one degree of freedom for simple maneuvers. Trajectory optimization methods are presented for generating profiles with higher degrees of freedom.

The appendices provide further enrichment that isn’t required for a passing understanding of the material. This includes derivations for many of the results presented and used in the mainmatter of the book.

The Python scripts used to generate the plots in the case studies double as reference implementations of the techniques discussed in their respective chapters. They are available in this book’s Git repository. Its location is listed on the copyright page.

## 0.3 Ethos of this book

This book is intended as both a tutorial for new students and as a reference manual for more experienced readers who need to review a thing or two. While it isn’t comprehensive, the reader will hopefully learn enough to either implement the concepts presented themselves or know where to look for more information.

Some parts are mathematically rigorous, but I believe in giving students a solid theoretical foundation with emphasis on intuition so they can apply it to new problems. To achieve deep understanding of the topics in this book, math is unavoidable. With that said, I try to provide practical and intuitive explanations whenever possible.

Most teaching resources separate linear and nonlinear control with the latter being reserved for a different course. Here, they are introduced together because the concepts of nonlinear control apply often, and it isn’t that much of a leap (if Lyapunov stability isn’t included). The control and estimation chapters cover relevant tools for dealing with nonlinearities like linearization when appropriate.

## 0.4 Mindset of an egoless engineer

Engineering has a mindset, not just a skillset. Engineers have a unique way of approaching problems, and the following maxim summarizes what I hope to teach my robotics students (with examples drawn from controls engineering).

> “Engineer based on requirements, not an ideology.”

Engineering is filled with trade-offs. The tools should fit the job, and not every problem is a nail waiting to be struck by a hammer. Instead, assess the minimum requirements (min specs) for a solution to the task at hand and do only enough work to satisfy them; exceeding your specifications is a waste of time and money. If you require performance or maintainability above the min specs, your min specs were chosen incorrectly by definition.

Controls engineering is pragmatic in a similar respect: *solve. the. problem*. For control of nonlinear systems, [plant inversion](https://faculty.washington.edu/devasia/Inversion.html) is elegant on paper but doesn’t work with an inaccurate model, yet using a theoretically incorrect solution like linear approximations of the nonlinear system works well enough to be used industry-wide. There are more sophisticated controllers than PID, but we use PID anyway for its versatility and simplicity. Sometimes the inferior solutions are more effective or have a more desirable cost-benefit ratio than what the control system designer considers ideal or clean. Choose the tool that is most effective.

Solutions need to be good enough, but do not need to be perfect. We want to avoid integrators as they introduce instability, but we use them anyway because they work well for meeting tracking specifications. One should not blindly defend a design or follow an ideology, because there is always a case where its antithesis is a better option. The engineer should be able to determine when this is the case, set aside their ego, and do what will meet the specifications of their client (e.g., system response characteristics, maintainability, usability). Preferring one solution over another for pragmatic or technical reasons is fine, but the engineer should not care on a personal level which sufficient solution is chosen.

## 0.5 Request for feedback

While we have tried to write a book that makes the topics of control theory approachable, it still may be dense or fast-paced for some readers (it covers three classes of feedback control, two of which are for graduate students, in one short book). Please send us feedback, corrections, or suggestions through the GitHub link listed on the copyright page. New examples that demonstrate key concepts and make them more accessible are also appreciated.
