# Chapter 4: Calculus

This book uses derivatives and integrals occasionally to represent small changes in values over small changes in time and the infinitesimal sum of values over time respectively. The formulas and tables presented here are all you’ll need to carry through with the math in later chapters.

If you are interested in more information after reading this chapter, 3Blue1Brown does a fantastic job of introducing them in his *Essence of calculus* series.

> **Video:** [“Essence of calculus”](https://www.3blue1brown.com/?topic=calculus) — 3Blue1Brown

## 4.1 Derivatives

Derivatives are expressions for the slope of a curve at arbitrary points. Common notations for this operation on a function like $f(x)$ include

*Table 4.1: Notation for derivatives of $f(x)$*

|  **Leibniz notation**   | **Lagrange notation** |    **Newton notation**    |
|:-----------------------:|:---------------------:|:-------------------------:|
|   $\frac{d}{dx} f(x)$   |        $f'(x)$        |       $\dot{f}(x)$        |
| $\frac{d^2}{dx^2} f(x)$ |       $f''(x)$        |       $\ddot{f}(x)$       |
| $\frac{d^3}{dx^3} f(x)$ |       $f'''(x)$       |      $\dddot{f}(x)$       |
| $\frac{d^4}{dx^4} f(x)$ |     $f^{(4)}(x)$      | $\overset{4}{\dot{f}}(x)$ |
| $\frac{d^n}{dx^n} f(x)$ |     $f^{(n)}(x)$      | $\overset{n}{\dot{f}}(x)$ |

Lagrange notation is voiced as “f prime of x”, “f double-prime of x”, etc. Newton notation is voiced as “f dot of x”, “f double-dot of x”, etc.

### 4.1.1 Power rule

$$
\begin{aligned}
f(x) &= x^n \\
f'(x) &= nx^{n - 1}
\end{aligned}
$$

### 4.1.2 Product rule

This is for taking the derivative of the product of two expressions.

$$
\begin{aligned}
h(x) &= f(x)g(x) \\
h'(x) &= f'(x)g(x) + f(x)g'(x)
\end{aligned}
$$

### 4.1.3 Chain rule

This is for taking the derivative of nested expressions.

$$
\begin{aligned}
h(x) &= f(g(x)) \\
h'(x) &= f'(g(x)) \cdot g'(x)
\end{aligned}
$$

For example,

$$
\begin{aligned}
h(x) &= \left(3x + 2\right)^5 \\
h'(x) &= 5\left(3x + 2\right)^4 \cdot \left(3\right) \\
h'(x) &= 15\left(3x + 2\right)^4
\end{aligned}
$$

### 4.1.4 Partial derivatives

A partial derivative of a function of several variables is its derivative with respect to one of those variables, with the others held constant (as opposed to the total derivative, in which all variables are allowed to vary). Partial derivatives use $\partial$ instead of $d$ in Leibniz notation.

For example, let $h(x, y) = 3xy + 2x$. For the partial derivative with respect to $x$, $y$ is treated as a constant.

$$
\frac{\partial h(x, y)}{\partial x} = 3y + 2
$$

For the partial derivative with respect to $y$, $x$ is treated as a constant, so the second term becomes zero.

$$
\frac{\partial h(x, y)}{\partial y} = 3x
$$

## 4.2 Integrals

The integral is the inverse operation of the derivative and calculates the area under a curve. Here is an example of one based on table 4.2.

$$
\begin{aligned}
\int e^{at} \,dt \\
\frac{1}{a}e^{at} + C
\end{aligned}
$$

The arbitrary constant $C$ is needed because when you take a derivative, constants are discarded because vertical offsets don’t affect the slope. When performing the inverse operation, we don’t have enough information to determine the constant.

However, we can provide bounds for the integration.

$$
\begin{aligned}
&\int_0^t e^{at} \,dt \\
&\left.\left(\frac{1}{a}e^{at} + C\right)\right\vert_0^t \\
&\left(\frac{1}{a}e^{at} + C\right) -
    \left(\frac{1}{a}e^{a \cdot 0} + C\right) \\
&\left(\frac{1}{a}e^{at} + C\right) - \left(\frac{1}{a} + C\right) \\
&\frac{1}{a}e^{at} + C - \frac{1}{a} - C \\
&\frac{1}{a}e^{at} - \frac{1}{a}
\end{aligned}
$$

When we do this, the constant cancels out.

## 4.3 Change of variables

Change of variables is a technique for simplifying problems in which expressions are replaced with new variables to make the problem more tractable. This can mean either the problem is more straightforward or it matches a common form for which tools for finding solutions are readily available. Here’s an example of integration which utilizes it.

$$
\int \cos\omega t \,dt
$$

Let $u = \omega t$.

$$
\begin{aligned}
du &= \omega \,dt \\
dt &= \frac{1}{\omega} \,du
\end{aligned}
$$

Now substitute the expressions for $u$ and $dt$ in.

$$
\begin{aligned}
&\int \cos u \,\frac{1}{\omega} \,du \\
&\frac{1}{\omega} \int \cos u \,du \\
&\frac{1}{\omega} \sin u + C \\
&\frac{1}{\omega} \sin\omega t + C
\end{aligned}
$$

Another example, which will be relevant when we actually cover state-space notation ($\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$), is a closed-loop state-space system.

$$
\begin{aligned}
\dot{\mathbf{x}} &= (\mathbf{A} - \mathbf{B}\mathbf{K})\mathbf{x} + \mathbf{B}\mathbf{K}\mathbf{r} \\
\dot{\mathbf{x}} &= \mathbf{A}_{cl}\mathbf{x} + \mathbf{B}_{cl}\mathbf{u}_{cl}
\end{aligned}
$$

where $\mathbf{A}_{cl} = \mathbf{A} - \mathbf{B}\mathbf{K}$, $\mathbf{B}_{cl} = \mathbf{B}\mathbf{K}$, and $\mathbf{u}_{cl} = \mathbf{r}$. Since it matches the form of the open-loop system, all the same analysis tools will work with it.

## 4.4 Tables

### 4.4.1 Common derivatives and integrals

*Table 4.2: Common derivatives and integrals*

|    **$\int f(x) \,dx$**    | **$f(x)$** | **$f'(x)$**  |
|:--------------------------:|:----------:|:------------:|
|            $ax$            |    $a$     |     $0$      |
|     $\frac{1}{2}ax^2$      |    $ax$    |     $a$      |
| $\frac{1}{a + 1}x^{a + 1}$ |   $x^a$    | $ax^{a - 1}$ |
|    $\frac{1}{a}e^{ax}$     |  $e^{ax}$  |  $ae^{ax}$   |
|         $-\cos(x)$         | $\sin(x)$  |  $\cos(x)$   |
|         $\sin(x)$          | $\cos(x)$  |  $-\sin(x)$  |
|         $\cos(x)$          | $-\sin(x)$ |  $-\cos(x)$  |
|         $-\sin(x)$         | $-\cos(x)$ |  $\sin(x)$   |

## 4.5 Differential equations

A *differential equation* is an equation containing derivatives.

Let $y$ be an unknown function of $x$ and $f$ be a given function of $x$. An *ordinary differential equation* (ODE) contains an unknown function of one variable $x$, the unknown function’s derivatives, and given functions of $x$. For example,

$$
\frac{dy}{dx} = f(x)
$$

Let $y$ be an unknown function of $x_1$ and $x_2$. A *partial differential equation* (PDE) contains unknown multivariable functions and their partial derivatives. For example,

$$
y(x_1, x_2) = x_1 \frac{\partial y}{\partial x_1} +
    x_2 \frac{\partial y}{\partial x_2}
$$

Some ODEs and PDEs have closed form solutions (e.g., $y(x) = y(0)e^{ax}$ for the ODE $\frac{dy}{dx} = ay$), but most must be solved numerically with a computer instead.
