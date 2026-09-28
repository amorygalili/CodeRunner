# Chapter 14: System identification

First, we’ll cover a system identification procedure for generating a velocity system feedforward model from performance data. Then, we’ll use that feedforward model to create state-space models for several common mechanisms.

## 14.1 Ordinary least squares

The parameter estimation in this chapter will be performed using ordinary least squares (OLS). Let there be a linear equation of the form $y = \beta_1 x_1 + \ldots + \beta_p x_p$. We want to find the constants $\beta_1, \ldots, \beta_p$ that best fit a set of observations represented by $x_{1,\ldots,n}$-$y$ tuples. Consider the overdetermined system of $n$ linear equations $\mathbf{y} = \mathbf{X}\boldsymbol{\beta}$ where

$$
\mathbf{y} = \begin{bmatrix}
    y_1 \\
    \vdots \\
    y_n
  \end{bmatrix}
  \quad
  \mathbf{X} = \begin{bmatrix}
    x_{11} & \cdots & x_{1p} \\
    \vdots & \ddots & \vdots \\
    x_{n1} & \cdots & x_{np}
  \end{bmatrix}
  \quad
  \boldsymbol{\beta} = \begin{bmatrix}
    \beta_1 \\
    \vdots \\
    \beta_p
  \end{bmatrix}
$$

each row corresponds to a datapoint, and $n > p$ (more datapoints than unknowns). OLS is a type of linear least squares method that estimates the unknown parameters $\boldsymbol{\beta}$ in a linear regression model $\mathbf{y} = \mathbf{X}\boldsymbol{\beta} + \boldsymbol{\epsilon}$ where $\boldsymbol{\epsilon}$ is the error in the observations $\mathbf{y}$. $\boldsymbol{\beta}$ is chosen by the method of least squares: minimizing the sum of the squares of the difference between $\mathbf{y}$ (observations of the dependent variable) and $\mathbf{X}\boldsymbol{\beta}$ (predictions of $\mathbf{y}$ using a linear function of the independent variable $\boldsymbol{\beta}$).

Geometrically, this is the sum of the squared distances, parallel to the y-axis, between each value in $\mathbf{y}$ and the corresponding value in $\mathbf{X}\boldsymbol{\beta}$. Smaller differences mean a better model fit.

To find the $\boldsymbol{\beta}$ that fits best, we can solve the following quadratic minimization problem

$$
\hat{\boldsymbol{\beta}} = \operatorname*{arg\,min}_{\boldsymbol{\beta}} (\mathbf{y} - \mathbf{X}\boldsymbol{\beta})^{\mathsf{T}}
    (\mathbf{y} - \mathbf{X}\boldsymbol{\beta})
$$

$\operatorname*{arg\,min}_{\boldsymbol{\beta}}$ means “find the value of $\boldsymbol{\beta}$ that minimizes the following function of $\boldsymbol{\beta}$”. Given corollary 5.15.3, take the partial derivative of the cost function with respect to $\boldsymbol{\beta}$ and set it equal to zero, then solve for $\hat{\boldsymbol{\beta}}$.

$$
\begin{aligned}
\mathbf{0} &= -2\mathbf{X}^{\mathsf{T}} (\mathbf{y} - \mathbf{X}\hat{\boldsymbol{\beta}}) \\
\mathbf{0} &= \mathbf{X}^{\mathsf{T}} (\mathbf{y} - \mathbf{X}\hat{\boldsymbol{\beta}}) \\
\mathbf{0} &= \mathbf{X}^{\mathsf{T}} \mathbf{y} - \mathbf{X}^{\mathsf{T}}\mathbf{X}\hat{\boldsymbol{\beta}} \\
\mathbf{X}^{\mathsf{T}}\mathbf{X}\hat{\boldsymbol{\beta}} &= \mathbf{X}^{\mathsf{T}} \mathbf{y} \\
\hat{\boldsymbol{\beta}} &= (\mathbf{X}^{\mathsf{T}}\mathbf{X})^{-1} \mathbf{X}^{\mathsf{T}} \mathbf{y}
\end{aligned} \tag{14.1}
$$

### 14.1.1 Examples

While this is a linear regression, we can fit nonlinear functions by making the contents of $\mathbf{X}$ nonlinear functions of the independent variables. For example, we can find the quadratic equation $y = ax^2 + bx + c$ that best fits a set of $x$-$y$ tuples. Lets assume we have a set of observations as follows.

$$
\begin{aligned}
y_1 &= ax_1^2 + bx_1 + c \\
&\ \ \vdots \\
y_n &= ax_n^2 + bx_n + c
\end{aligned}
$$

We want to find $a$, $b$, and $c$, so let’s factor those out.

$$
\begin{aligned}
\begin{bmatrix}
    y_1 \\
    \vdots \\
    y_n
  \end{bmatrix} &=
  \begin{bmatrix}
    x_1^2 & x_1 & 1 \\
    \vdots & \vdots & \vdots \\
    x_n^2 & x_n & 1
  \end{bmatrix}
  \begin{bmatrix}
    a \\
    b \\
    c
  \end{bmatrix}
\end{aligned}
$$

Plug these matrices into equation (14.1) to obtain the coefficients $a$, $b$, and $c$.

$$
\mathbf{y} = \begin{bmatrix}
    y_1 \\
    \vdots \\
    y_n
  \end{bmatrix}
  \quad
  \mathbf{X} = \begin{bmatrix}
    x_1^2 & x_1 & 1 \\
    \vdots & \vdots & \vdots \\
    x_n^2 & x_n & 1
  \end{bmatrix}
  \quad
  \boldsymbol{\beta} = \begin{bmatrix}
    a \\
    b \\
    c
  \end{bmatrix}
$$

## 14.2 1-DOF mechanism state-space model

$$
\dot{x} = \alpha x + \beta u + \gamma \operatorname{sgn}(x)
$$

$$
\dot{x} = \text{acceleration}
  \quad
  x = \text{velocity}
  \quad
  u = \text{voltage}
$$

We can determine $\alpha$, $\beta$, and $\gamma$ by applying ordinary least squares (OLS) to vectors of recorded input voltage, velocity, and acceleration data from quasistatic velocity tests and acceleration tests, where acceleration is the dependent variable. If acceleration isn’t directly available, it can be computed numerically from the velocity data and filtered.

Solving this model for $u$ gives the following feedforward model.

$$
\begin{aligned}
u &= -\frac{\gamma}{\beta} \operatorname{sgn}(x) - \frac{\alpha}{\beta}x +
    \frac{1}{\beta}\dot{x}
\end{aligned}
$$

Let $K_s = -\frac{\gamma}{\beta}$, $K_v = -\frac{\alpha}{\beta}$, and $K_a = \frac{1}{\beta}$.

$$
\begin{aligned}
u &= K_s \operatorname{sgn}(x) + K_v x + K_a \dot{x}
\end{aligned} \tag{14.2}
$$

$K_s$ is a constant that describes how much voltage is required to overcome friction and start moving.

$K_v$ is a proportional constant that describes how much voltage is required to maintain a given constant velocity by offsetting the electromagnetic resistance of the motor and any friction that increases linearly with speed (viscous drag). The relationship between speed and voltage (for a given initial acceleration) is linear for permanent-magnet DC motors in the FRC operating regime.

$K_a$ is a proportional constant that describes how much voltage is required to induce a given acceleration in the motor shaft. As with $K_v$, the relationship between voltage and acceleration (for a given initial velocity) is linear.

Convert equation (14.2) to state-space form by solving for $\dot{x}$.

$$
\dot{x} = -\frac{K_v}{K_a}x + \frac{1}{K_a}u - \frac{K_s}{K_a}\operatorname{sgn}(x)
$$

By inspection, $\mathbf{A} = -\frac{K_v}{K_a}$, $\mathbf{B} = \frac{1}{K_a}$, and $\mathbf{c} = -\frac{K_s}{K_a}\operatorname{sgn}(\mathbf{x})$ in the state-space model $\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u} + \mathbf{c}$. A model with position and velocity states would be

> **Theorem 14.2.1 — 1-DOF mechanism position model.**
>
> $$
> \dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u} + \mathbf{c}
> $$
>
> $$
> \mathbf{x} =
>     \begin{bmatrix}
>       \text{position} \\
>       \text{velocity}
>     \end{bmatrix}
>     \quad
>     \mathbf{u} =
>     \begin{bmatrix}
>       \text{voltage}
>     \end{bmatrix}
> $$
>
> $$
> \dot{\mathbf{x}} =
>     \begin{bmatrix}
>       0 & 1 \\
>       0 & -\frac{K_v}{K_a}
>     \end{bmatrix}
>     \mathbf{x} +
>     \begin{bmatrix}
>       0 \\
>       \frac{1}{K_a}
>     \end{bmatrix}
>     \mathbf{u} +
>     \begin{bmatrix}
>       0 \\
>       -\frac{K_s}{K_a}\operatorname{sgn}(\text{velocity})
>     \end{bmatrix}
> $$
>

The model in theorem 14.2.1 is undefined when $K_a = 0$. To design an LQR for such a system, use the model in theorem 14.2.2 instead. As $K_a$ tends to zero, acceleration requires no effort and velocity becomes an input for position control.

> **Theorem 14.2.2 — 1-DOF mechanism position model ($K_a = 0$).**
>
> $$
> \dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}
> $$
>
> $$
> \mathbf{x} =
>     \begin{bmatrix}
>       \text{position}
>     \end{bmatrix}
>     \quad
>     \mathbf{u} =
>     \begin{bmatrix}
>       \text{velocity}
>     \end{bmatrix}
> $$
>
> $$
> \dot{\mathbf{x}} =
>     \begin{bmatrix}
>       0
>     \end{bmatrix}
>     \mathbf{x} +
>     \begin{bmatrix}
>       1
>     \end{bmatrix}
>     \mathbf{u}
> $$
>

### 14.2.1 Simple system identification

This system identification method is easier but less accurate than OLS. It requires two tests.

1.  Starting from rest, slowly increase the voltage from zero until the robot moves.

2.  Starting from rest, apply a constant voltage $u$ sufficiently larger than $K_s$ until the steady-state velocity $v$ is reached, and measure the instantaneous acceleration from rest $a$.

The feedforward gains from equation (14.2) are given by

$$
\begin{aligned}
K_s &= \text{minimum voltage causing movement in test 1} \\
K_v &= \frac{u - K_s}{v} \\
K_a &= \frac{u - K_s - K_v v}{a}
\end{aligned}
$$

Acceleration may be difficult to accurately measure in test 2. A reasonable estimate is the largest observed value of $\frac{v_k - v_{k-1}}{\Delta T}$ where $v_k$ and $v_{k-1}$ are adjacent velocity samples and $\Delta T$ is the sample period. Systems with very high accelerations may have a negligibly small $K_a$.

## 14.3 Drivetrain left/right velocity state-space model

$$
\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}
$$

$$
\mathbf{x} = \begin{bmatrix}
    \text{left velocity} \\
    \text{right velocity}
  \end{bmatrix}
  \quad
  \dot{\mathbf{x}} = \begin{bmatrix}
    \text{left acceleration} \\
    \text{right acceleration}
  \end{bmatrix}
  \quad
  \mathbf{u} = \begin{bmatrix}
    \text{left voltage} \\
    \text{right voltage}
  \end{bmatrix}
$$

We want to derive what $\mathbf{A}$ and $\mathbf{B}$ are from linear and angular feedforward models. Since the left and right dynamics are symmetric, we’ll guess the model has the form

$$
\mathbf{A} = \begin{bmatrix}
    A_1 & A_2 \\
    A_2 & A_1
  \end{bmatrix}
  \quad
  \mathbf{B} = \begin{bmatrix}
    B_1 & B_2 \\
    B_2 & B_1
  \end{bmatrix}
$$

Let $K_{v,lin}$ be the linear velocity gain, $K_{a,lin}$ be the linear acceleration gain, $K_{v,ang}$ be the angular velocity gain, and $K_{a,ang}$ be the angular acceleration gain. Let $\mathbf{u} = \begin{bmatrix} K_{v,lin} v & K_{v,lin} v \end{bmatrix}^{\mathsf{T}}$ be the input that makes both sides of the drivetrain move at a constant velocity $v$. Therefore, $\mathbf{x} = \begin{bmatrix} v & v \end{bmatrix}^{\mathsf{T}}$ and $\dot{\mathbf{x}} = \begin{bmatrix} 0 & 0 \end{bmatrix}^{\mathsf{T}}$. Substitute these into the state-space model.

$$
\begin{aligned}
\begin{bmatrix}
    0 \\
    0
  \end{bmatrix} &=
  \begin{bmatrix}
    A_1 & A_2 \\
    A_2 & A_1
  \end{bmatrix}
  \begin{bmatrix}
    v \\
    v
  \end{bmatrix} +
  \begin{bmatrix}
    B_1 & B_2 \\
    B_2 & B_1
  \end{bmatrix}
  \begin{bmatrix}
    K_{v,lin} v \\
    K_{v,lin} v
  \end{bmatrix}
\end{aligned}
$$

Since the column vectors contain the same element, the elements in the second row can be rearranged.

$$
\begin{aligned}
\begin{bmatrix}
    0 \\
    0
  \end{bmatrix} &=
  \begin{bmatrix}
    A_1 & A_2 \\
    A_1 & A_2
  \end{bmatrix}
  \begin{bmatrix}
    v \\
    v
  \end{bmatrix} +
  \begin{bmatrix}
    B_1 & B_2 \\
    B_1 & B_2
  \end{bmatrix}
  \begin{bmatrix}
    K_{v,lin} v \\
    K_{v,lin} v
  \end{bmatrix}
\end{aligned}
$$

Since the rows are linearly dependent, we can use just one of them.

$$
\begin{aligned}
0 &=
    \begin{bmatrix}
      A_1 & A_2
    \end{bmatrix} v +
    \begin{bmatrix}
      B_1 & B_2
    \end{bmatrix} K_{v,lin} v \\
  0 &=
    \begin{bmatrix}
      v & v & K_{v,lin} v & K_{v,lin} v
    \end{bmatrix}
    \begin{bmatrix}
      A_1 \\
      A_2 \\
      B_1 \\
      B_2
    \end{bmatrix} \\
  0 &=
    \begin{bmatrix}
      1 & 1 & K_{v,lin} & K_{v,lin}
    \end{bmatrix}
    \begin{bmatrix}
      A_1 \\
      A_2 \\
      B_1 \\
      B_2
    \end{bmatrix}
\end{aligned}
$$

Let $\mathbf{u} = \begin{bmatrix} K_{v,lin} v + K_{a,lin} a & K_{v,lin} v + K_{a,lin} a \end{bmatrix}^{\mathsf{T}}$ be the input that accelerates both sides of the drivetrain by $a$ from an initial velocity of $v$. Therefore, $\mathbf{x} = \begin{bmatrix} v & v \end{bmatrix}^{\mathsf{T}}$ and $\dot{\mathbf{x}} = \begin{bmatrix} a & a \end{bmatrix}^{\mathsf{T}}$. Substitute these into the state-space model.

$$
\begin{aligned}
\begin{bmatrix}
    a \\
    a
  \end{bmatrix} &=
  \begin{bmatrix}
    A_1 & A_2 \\
    A_2 & A_1
  \end{bmatrix}
  \begin{bmatrix}
    v \\
    v
  \end{bmatrix} +
  \begin{bmatrix}
    B_1 & B_2 \\
    B_2 & B_1
  \end{bmatrix}
  \begin{bmatrix}
    K_{v,lin} v + K_{a,lin} a \\
    K_{v,lin} v + K_{a,lin} a
  \end{bmatrix}
\end{aligned}
$$

Since the column vectors contain the same element, the elements in the second row can be rearranged.

$$
\begin{aligned}
\begin{bmatrix}
    a \\
    a
  \end{bmatrix} &=
  \begin{bmatrix}
    A_1 & A_2 \\
    A_1 & A_2
  \end{bmatrix}
  \begin{bmatrix}
    v \\
    v
  \end{bmatrix} +
  \begin{bmatrix}
    B_1 & B_2 \\
    B_1 & B_2
  \end{bmatrix}
  \begin{bmatrix}
    K_{v,lin} v + K_{a,lin} a \\
    K_{v,lin} v + K_{a,lin} a
  \end{bmatrix}
\end{aligned}
$$

Since the rows are linearly dependent, we can use just one of them.

$$
\begin{aligned}
a &=
    \begin{bmatrix}
      A_1 & A_2
    \end{bmatrix} v +
    \begin{bmatrix}
      B_1 & B_2
    \end{bmatrix}
    \begin{bmatrix}
      K_{v,lin} v + K_{a,lin} a
    \end{bmatrix} \\
  a &=
    \begin{bmatrix}
      v & v & K_{v,lin} v + K_{a,lin} a & K_{v,lin} + K_{a,lin} a
    \end{bmatrix}
    \begin{bmatrix}
      A_1 \\
      A_2 \\
      B_1 \\
      B_2
    \end{bmatrix}
\end{aligned}
$$

Let $\mathbf{u} = \begin{bmatrix} -K_{v,ang} v & K_{v,ang} v \end{bmatrix}^{\mathsf{T}}$ be the input that rotates the drivetrain in place where each wheel has a constant velocity $v$. Therefore, $\mathbf{x} = \begin{bmatrix} -v & v \end{bmatrix}^{\mathsf{T}}$ and $\dot{\mathbf{x}} = \begin{bmatrix} 0 & 0 \end{bmatrix}^{\mathsf{T}}$.

$$
\begin{aligned}
\begin{bmatrix}
    0 \\
    0
  \end{bmatrix} &=
    \begin{bmatrix}
      A_1 & A_2 \\
      A_2 & A_1
    \end{bmatrix}
    \begin{bmatrix}
      -v \\
      v
    \end{bmatrix} +
    \begin{bmatrix}
      B_1 & B_2 \\
      B_2 & B_1
    \end{bmatrix}
    \begin{bmatrix}
      -K_{v,ang} v \\
      K_{v,ang} v
    \end{bmatrix} \\
  \begin{bmatrix}
    0 \\
    0
  \end{bmatrix} &=
    \begin{bmatrix}
      -A_1 & A_2 \\
      -A_2 & A_1
    \end{bmatrix}
    \begin{bmatrix}
      v \\
      v
    \end{bmatrix} +
    \begin{bmatrix}
      -B_1 & B_2 \\
      -B_2 & B_1
    \end{bmatrix}
    \begin{bmatrix}
      K_{v,ang} v \\
      K_{v,ang} v
    \end{bmatrix} \\
  \begin{bmatrix}
    0 \\
    0
  \end{bmatrix} &=
    \begin{bmatrix}
      -A_1 & A_2 \\
      A_1 & -A_2
    \end{bmatrix}
    \begin{bmatrix}
      v \\
      v
    \end{bmatrix} +
    \begin{bmatrix}
      -B_1 & B_2 \\
      B_1 & -B_2
    \end{bmatrix}
    \begin{bmatrix}
      K_{v,ang} v \\
      K_{v,ang} v
    \end{bmatrix}
\end{aligned}
$$

Since the column vectors contain the same element, the elements in the second row can be rearranged.

$$
\begin{aligned}
\begin{bmatrix}
    0 \\
    0
  \end{bmatrix} &=
  \begin{bmatrix}
    -A_1 & A_2 \\
    -A_1 & A_2
  \end{bmatrix}
  \begin{bmatrix}
    v \\
    v
  \end{bmatrix} +
  \begin{bmatrix}
    -B_1 & B_2 \\
    -B_1 & B_2
  \end{bmatrix}
  \begin{bmatrix}
    K_{v,ang} v \\
    K_{v,ang} v
  \end{bmatrix}
\end{aligned}
$$

Since the rows are linearly dependent, we can use just one of them.

$$
\begin{aligned}
0 &=
    \begin{bmatrix}
      -A_1 & A_2
    \end{bmatrix} v +
    \begin{bmatrix}
      -B_1 & B_2
    \end{bmatrix} K_{v,ang} v \\
  0 &= -v A_1 + v A_2 - K_{v,ang} v B_1 + K_{v,ang} v B_2 \\
  0 &=
    \begin{bmatrix}
      -v & v & -K_{v,ang} v & K_{v,ang} v
    \end{bmatrix}
    \begin{bmatrix}
      A_1 \\
      A_2 \\
      B_1 \\
      B_2
    \end{bmatrix} \\
  0 &=
    \begin{bmatrix}
      -1 & 1 & -K_{v,ang} & K_{v,ang}
    \end{bmatrix}
    \begin{bmatrix}
      A_1 \\
      A_2 \\
      B_1 \\
      B_2
    \end{bmatrix}
\end{aligned}
$$

Let $\mathbf{u} = \begin{bmatrix} -K_{v,ang} v - K_{a,ang} a & K_{v,ang} v + K_{a,ang} a \end{bmatrix}^{\mathsf{T}}$ be the input that rotates the drivetrain in place where each wheel has an initial speed of $v$ and accelerates by $a$. Therefore, $\mathbf{x} = \begin{bmatrix} -v & v \end{bmatrix}^{\mathsf{T}}$ and $\dot{\mathbf{x}} = \begin{bmatrix} -a & a \end{bmatrix}^{\mathsf{T}}$.

$$
\begin{aligned}
\begin{bmatrix}
    -a \\
    a
  \end{bmatrix} &=
    \begin{bmatrix}
      A_1 & A_2 \\
      A_2 & A_1
    \end{bmatrix}
    \begin{bmatrix}
      -v \\
      v
    \end{bmatrix} +
    \begin{bmatrix}
      B_1 & B_2 \\
      B_2 & B_1
    \end{bmatrix}
    \begin{bmatrix}
      -K_{v,ang} v - K_{a,ang} a \\
      K_{v,ang} v + K_{a,ang} a
    \end{bmatrix} \\
  \begin{bmatrix}
    -a \\
    a
  \end{bmatrix} &=
    \begin{bmatrix}
      -A_1 & A_2 \\
      -A_2 & A_1
    \end{bmatrix}
    \begin{bmatrix}
      v \\
      v
    \end{bmatrix} +
    \begin{bmatrix}
      -B_1 & B_2 \\
      -B_2 & B_1
    \end{bmatrix}
    \begin{bmatrix}
      K_{v,ang} v + K_{a,ang} a \\
      K_{v,ang} v + K_{a,ang} a
    \end{bmatrix} \\
  \begin{bmatrix}
    -a \\
    a
  \end{bmatrix} &=
    \begin{bmatrix}
      -A_1 & A_2 \\
      A1 & -A_2
    \end{bmatrix}
    \begin{bmatrix}
      v \\
      v
    \end{bmatrix} +
    \begin{bmatrix}
      -B_1 & B_2 \\
      B_1 & -B_2
    \end{bmatrix}
    \begin{bmatrix}
      K_{v,ang} v + K_{a,ang} a \\
      K_{v,ang} v + K_{a,ang} a
    \end{bmatrix}
\end{aligned}
$$

Since the column vectors contain the same element, the elements in the second row can be rearranged.

$$
\begin{aligned}
\begin{bmatrix}
    -a \\
    -a
  \end{bmatrix} &=
  \begin{bmatrix}
    -A_1 & A_2 \\
    -A_1 & A_2
  \end{bmatrix}
  \begin{bmatrix}
    v \\
    v
  \end{bmatrix} +
  \begin{bmatrix}
    -B_1 & B_2 \\
    -B_1 & B_2
  \end{bmatrix}
  \begin{bmatrix}
    K_{v,ang} v + K_{a,ang} a \\
    K_{v,ang} v + K_{a,ang} a
  \end{bmatrix}
\end{aligned}
$$

Since the rows are linearly dependent, we can use just one of them.

$$
\begin{aligned}
-a &=
    \begin{bmatrix}
      -A_1 & A_2
    \end{bmatrix} v +
    \begin{bmatrix}
      -B_1 & B_2
    \end{bmatrix}
    \begin{bmatrix}
      K_{v,ang} v + K_{a,ang} a
    \end{bmatrix} \\
  -a &= -v A_1 + v A_2 - (K_{v,ang} v + K_{a,ang} a) B_1 + (K_{v,ang} v + K_{a,ang} a) B_2 \\
  -a &=
    \begin{bmatrix}
      -v & v & -(K_{v,ang} v + K_{a,ang} a) & K_{v,ang} v+ K_{a,ang} a
    \end{bmatrix}
    \begin{bmatrix}
      A_1 \\
      A_2 \\
      B_1 \\
      B_2
    \end{bmatrix} \\
  a &=
    \begin{bmatrix}
      v & -v & K_{v,ang} v + K_{a,ang} a & -(K_{v,ang} v + K_{a,ang} a)
    \end{bmatrix}
    \begin{bmatrix}
      A_1 \\
      A_2 \\
      B_1 \\
      B_2
    \end{bmatrix}
\end{aligned}
$$

Now stack the rows.

$$
\begin{bmatrix}
    0 \\
    a \\
    0 \\
    a
  \end{bmatrix} =
  \begin{bmatrix}
    1 & 1 & K_{v,lin} & K_{v,lin} \\
    v & v & K_{v,lin} v + K_{a,lin} a & K_{v,lin} v + K_{a,lin} a \\
    -1 & 1 & -K_{v,ang} & K_{v,ang} \\
    v & -v & K_{v,ang} v + K_{a,ang} a & -(K_{v,ang} v + K_{a,ang} a)
  \end{bmatrix}
  \begin{bmatrix}
    A_1 \\
    A_2 \\
    B_1 \\
    B_2
  \end{bmatrix}
$$

Solve for matrix elements with Wolfram Alpha. Let $b = K_{v,lin}$, $c = K_{a,lin}$, $d = K_{v,ang}$, and $f = K_{a,ang}$.

```
inverse of {{1, 1, b, b},
            {v, v, b v + c a, b v + c a},
            {-1, 1, -d, d},
            {v, -v, d v + f a, -(d v + f a)}}
           * {{0}, {a}, {0}, {a}}
```

$$
\begin{aligned}
\begin{bmatrix}
    A_1 \\
    A_2 \\
    B_1 \\
    B_2
  \end{bmatrix} &= \frac{1}{2cf}
  \begin{bmatrix}
    -cd - bf \\
    cd - bf \\
    c + f \\
    f - c
  \end{bmatrix} \\
  \begin{bmatrix}
    A_1 \\
    A_2 \\
    B_1 \\
    B_2
  \end{bmatrix} &= \frac{1}{2 K_{a,lin} K_{a,ang}}
  \begin{bmatrix}
    -K_{a,lin} K_{v,ang} - K_{v,lin} K_{a,ang} \\
    K_{a,lin} K_{v,ang} - K_{v,lin} K_{a,ang} \\
    K_{a,lin} + K_{a,ang} \\
    K_{a,ang} - K_{a,lin}
  \end{bmatrix} \\
  \begin{bmatrix}
    A_1 \\
    A_2 \\
    B_1 \\
    B_2
  \end{bmatrix} &= \frac{1}{2}
  \begin{bmatrix}
    -\frac{K_{a,lin} K_{v,ang}}{K_{a,lin} K_{a,ang}} -
      \frac{K_{v,lin} K_{a,ang}}{K_{a,lin} K_{a,ang}} \\
    \frac{K_{a,lin} K_{v,ang}}{K_{a,lin} K_{a,ang}} -
      \frac{K_{v,lin} K_{a,ang}}{K_{a,lin} K_{a,ang}} \\
    \frac{K_{a,lin}}{K_{a,lin} K_{a,ang}} +
      \frac{K_{a,ang}}{K_{a,lin} K_{a,ang}} \\
    \frac{K_{a,ang}}{K_{a,lin} K_{a,ang}} -
      \frac{K_{a,lin}}{K_{a,lin} K_{a,ang}}
  \end{bmatrix} \\
  \begin{bmatrix}
    A_1 \\
    A_2 \\
    B_1 \\
    B_2
  \end{bmatrix} &= \frac{1}{2}
  \begin{bmatrix}
    -\frac{K_{v,ang}}{K_{a,ang}} -
      \frac{K_{v,lin}}{K_{a,lin}} \\
    \frac{K_{v,ang}}{K_{a,ang}} -
      \frac{K_{v,lin}}{K_{a,lin}} \\
    \frac{1}{K_{a,ang}} + \frac{1}{K_{a,lin}} \\
    \frac{1}{K_{a,lin}} - \frac{1}{K_{a,ang}}
  \end{bmatrix} \\
  \begin{bmatrix}
    A_1 \\
    A_2 \\
    B_1 \\
    B_2
  \end{bmatrix} &= \frac{1}{2}
  \begin{bmatrix}
    -\left(\frac{K_{v,lin}}{K_{a,lin}} +
      \frac{K_{v,ang}}{K_{a,ang}}\right) \\
    -\left(\frac{K_{v,lin}}{K_{a,lin}} -
      \frac{K_{v,ang}}{K_{a,ang}}\right) \\
    \frac{1}{K_{a,lin}} + \frac{1}{K_{a,ang}} \\
    \frac{1}{K_{a,lin}} - \frac{1}{K_{a,ang}}
  \end{bmatrix}
\end{aligned}
$$

To summarize,

> **Theorem 14.3.1 — Drivetrain left/right velocity model.**
>
> $$
> \dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}
> $$
>
> $$
> \mathbf{x} =
>     \begin{bmatrix}
>       \text{left velocity} \\
>       \text{right velocity}
>     \end{bmatrix}
>     \quad
>     \mathbf{u} =
>     \begin{bmatrix}
>       \text{left voltage} \\
>       \text{right voltage}
>     \end{bmatrix}
> $$
>
> $$
> \dot{\mathbf{x}} =
>     \begin{bmatrix}
>       A_1 & A_2 \\
>       A_2 & A_1
>     \end{bmatrix} \mathbf{x} +
>     \begin{bmatrix}
>       B_1 & B_2 \\
>       B_2 & B_1
>     \end{bmatrix} \mathbf{u}
> $$
>
> $$
> \begin{bmatrix}
>       A_1 \\
>       A_2 \\
>       B_1 \\
>       B_2
>     \end{bmatrix} = \frac{1}{2}
>     \begin{bmatrix}
>       -\left(\frac{K_{v,lin}}{K_{a,lin}} +
>         \frac{K_{v,ang}}{K_{a,ang}}\right) \\
>       -\left(\frac{K_{v,lin}}{K_{a,lin}} -
>         \frac{K_{v,ang}}{K_{a,ang}}\right) \\
>       \frac{1}{K_{a,lin}} + \frac{1}{K_{a,ang}} \\
>       \frac{1}{K_{a,lin}} - \frac{1}{K_{a,ang}}
>     \end{bmatrix}
> $$
>
> $K_{v,ang}$ and $K_{a,ang}$ have units of $\frac{V}{L/T}$ and $\frac{V}{L/T^2}$ respectively where $V$ means voltage units, $L$ means length units, and $T$ means time units.

If $K_v$ and $K_a$ are the same for both the linear and angular cases, it devolves to the one-dimensional case. This means the left and right sides are decoupled.

$$
\begin{aligned}
\begin{bmatrix}
    A_1 \\
    A_2 \\
    B_1 \\
    B_2
  \end{bmatrix} &= \frac{1}{2}
  \begin{bmatrix}
    -\left(\frac{K_v}{K_a} + \frac{K_v}{K_a}\right) \\
    -\left(\frac{K_v}{K_a} - \frac{K_v}{K_a}\right) \\
    \frac{1}{K_a} + \frac{1}{K_a} \\
    \frac{1}{K_a} - \frac{1}{K_a}
  \end{bmatrix} \\
  \begin{bmatrix}
    A_1 \\
    A_2 \\
    B_1 \\
    B_2
  \end{bmatrix} &= \frac{1}{2}
  \begin{bmatrix}
    -2\frac{K_v}{K_a} \\
    0 \\
    \frac{2}{K_a} \\
    0
  \end{bmatrix} \\
  \begin{bmatrix}
    A_1 \\
    A_2 \\
    B_1 \\
    B_2
  \end{bmatrix} &=
  \begin{bmatrix}
    -\frac{K_v}{K_a} \\
    0 \\
    \frac{1}{K_a} \\
    0
  \end{bmatrix}
\end{aligned}
$$

## 14.4 Drivetrain linear/angular velocity state-space model

Let the linear and angular voltage contributions be

$$
\begin{aligned}
u_{lin} &= K_{v,lin} v + K_{a,lin} a \\
u_{ang} &= K_{v,ang} \omega + K_{a,ang} \alpha
\end{aligned}
$$

Solve for acceleration.

$$
\begin{aligned}
a &= -\frac{K_{v,lin}}{K_{a,lin}} v + \frac{1}{K_{a,lin}} u_{lin} \\
\alpha &= -\frac{K_{v,ang}}{K_{a,ang}} \omega + \frac{1}{K_{a,ang}} u_{lin}
\end{aligned}
$$

Factor them into a matrix equation.

$$
\dot{\mathbf{x}} =
  \begin{bmatrix}
    -\frac{K_{v,lin}}{K_{a,lin}} & 0 \\
    0 & -\frac{K_{v,ang}}{K_{a,ang}}
  \end{bmatrix}
  \begin{bmatrix}
    v \\
    \omega
  \end{bmatrix} +
  \begin{bmatrix}
    \frac{1}{K_{a,lin}} & 0 \\
    0 & \frac{1}{K_{a,ang}}
  \end{bmatrix}
  \begin{bmatrix}
    u_{lin} \\
    u_{ang}
  \end{bmatrix}
$$

We want to write this model in terms of left and right voltages. The linear and angular voltages have the following relations.

$$
\begin{aligned}
u_{left} &= u_{lin} - u_{ang} \\
u_{right} &= u_{lin} + u_{ang}
\end{aligned}
$$

Factor them into a matrix equation, then solve for the linear and angular voltages.

$$
\begin{aligned}
\begin{bmatrix}
    u_{left} \\
    u_{right}
  \end{bmatrix} &=
  \begin{bmatrix}
    1 & -1 \\
    1 & 1
  \end{bmatrix}
  \begin{bmatrix}
    u_{lin} \\
    u_{ang}
  \end{bmatrix} \\
  \begin{bmatrix}
    u_{lin} \\
    u_{ang}
  \end{bmatrix} &=
  \begin{bmatrix}
    0.5 & 0.5 \\
    -0.5 & 0.5
  \end{bmatrix}
  \begin{bmatrix}
    u_{left} \\
    u_{right}
  \end{bmatrix}
\end{aligned}
$$

Plug this into the model input.

$$
\begin{aligned}
\dot{\mathbf{x}} &=
  \begin{bmatrix}
    -\frac{K_{v,lin}}{K_{a,lin}} & 0 \\
    0 & -\frac{K_{v,ang}}{K_{a,ang}}
  \end{bmatrix}
  \begin{bmatrix}
    v \\
    \omega
  \end{bmatrix} +
  \begin{bmatrix}
    \frac{1}{K_{a,lin}} & 0 \\
    0 & \frac{1}{K_{a,ang}}
  \end{bmatrix}
  \begin{bmatrix}
    0.5 & 0.5 \\
    -0.5 & 0.5
  \end{bmatrix}
  \begin{bmatrix}
    u_{left} \\
    u_{right}
  \end{bmatrix} \\
  \dot{\mathbf{x}} &=
  \begin{bmatrix}
    -\frac{K_{v,lin}}{K_{a,lin}} & 0 \\
    0 & -\frac{K_{v,ang}}{K_{a,ang}}
  \end{bmatrix}
  \begin{bmatrix}
    v \\
    \omega
  \end{bmatrix} +
  \begin{bmatrix}
    \frac{0.5}{K_{a,lin}} & \frac{0.5}{K_{a,lin}} \\
    -\frac{0.5}{K_{a,ang}} & \frac{0.5}{K_{a,ang}}
  \end{bmatrix}
  \begin{bmatrix}
    u_{left} \\
    u_{right}
  \end{bmatrix}
\end{aligned}
$$

> **Theorem 14.4.1 — Drivetrain linear/angular velocity model.**
>
> $$
> \dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}
> $$
>
> $$
> \mathbf{x} =
>     \begin{bmatrix}
>       \text{linear velocity} \\
>       \text{angular velocity}
>     \end{bmatrix}
>     \quad
>     \mathbf{u} =
>     \begin{bmatrix}
>       \text{left voltage} \\
>       \text{right voltage}
>     \end{bmatrix}
> $$
>
> $$
> \dot{\mathbf{x}} =
>     \begin{bmatrix}
>       -\frac{K_{v,lin}}{K_{a,lin}} & 0 \\
>       0 & -\frac{K_{v,ang}}{K_{a,ang}}
>     \end{bmatrix} \mathbf{x} +
>     \begin{bmatrix}
>       \frac{0.5}{K_{a,lin}} & \frac{0.5}{K_{a,lin}} \\
>       -\frac{0.5}{K_{a,ang}} & \frac{0.5}{K_{a,ang}}
>     \end{bmatrix} \mathbf{u}
> $$
>
> $K_{v,ang}$ and $K_{a,ang}$ have units of $\frac{V}{A/T}$ and $\frac{V}{A/T^2}$ respectively where $V$ means voltage units, $A$ means angle units, and $T$ means time units.

## 14.5 Online resources

ReCalc’s[^1] mechanism calculators can compute feedforward and feedback gains from physical constants.

[^1]: <https://www.reca.lc/>
