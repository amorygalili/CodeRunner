# Appendix D: Derivations

## D.1 Linear system zero-order hold

Starting with the continuous model

$$
\dot{\mathbf{x}}(t) = \mathbf{A}\mathbf{x}(t) + \mathbf{B}\mathbf{u}(t)
$$

by premultiplying the model by $e^{-\mathbf{A}t}$, we get

$$
\begin{aligned}
e^{-\mathbf{A}t}\dot{\mathbf{x}}(t) &= e^{-\mathbf{A}t}\mathbf{A}\mathbf{x}(t) +
    e^{-\mathbf{A}t}\mathbf{B}\mathbf{u}(t) \\
e^{-\mathbf{A}t}\dot{\mathbf{x}}(t) - e^{-\mathbf{A}t}\mathbf{A}\mathbf{x}(t) &=
    e^{-\mathbf{A}t}\mathbf{B}\mathbf{u}(t)
\end{aligned}
$$

The derivative of the matrix exponential is

$$
\frac{d}{dt}e^{\mathbf{A}t} = \mathbf{A}e^{\mathbf{A}t} = e^{\mathbf{A}t}\mathbf{A}
$$

so we recognize the previous equation as

$$
\frac{d}{dt}\left(e^{-\mathbf{A}t}\mathbf{x}(t)\right) =
    e^{-\mathbf{A}t}\mathbf{B}\mathbf{u}(t)
$$

By integrating this equation, we get

$$
\begin{aligned}
e^{-\mathbf{A}t}\mathbf{x}(t) - e^0\mathbf{x}(0) &=
    \int_0^t e^{-\mathbf{A}\tau}\mathbf{B}\mathbf{u}(\tau) \,d\tau \\
\mathbf{x}(t) &= e^{\mathbf{A}t}\mathbf{x}(0) +
    \int_0^t e^{\mathbf{A}(t - \tau)}\mathbf{B}\mathbf{u}(\tau) \,d\tau
\end{aligned}
$$

which is an analytical solution to the continuous model. Now we want to discretize it.

$$
\begin{aligned}
\mathbf{x}_k &\stackrel{def}{=} \mathbf{x}(kT) \\
\mathbf{x}_k &= e^{\mathbf{A}kT}\mathbf{x}(0) +
    \int_0^{kT} e^{\mathbf{A}(kT - \tau)}\mathbf{B}\mathbf{u}(\tau) \,d\tau \\
\mathbf{x}_{k+1} &= e^{\mathbf{A}(k + 1)T}\mathbf{x}(0) +
    \int_0^{(k + 1)T} e^{\mathbf{A}((k + 1)T - \tau)}\mathbf{B}\mathbf{u}(\tau) \,d\tau \\
\mathbf{x}_{k+1} &= e^{\mathbf{A}(k + 1)T}\mathbf{x}(0) +
    \int_0^{kT} e^{\mathbf{A}((k + 1)T - \tau)}\mathbf{B}\mathbf{u}(\tau) \,d\tau \\
&\quad +
    \int_{kT}^{(k + 1)T} e^{\mathbf{A}((k + 1)T - \tau)}\mathbf{B}\mathbf{u}(\tau)
    \,d\tau \\
\mathbf{x}_{k+1} &= e^{\mathbf{A}(k + 1)T}\mathbf{x}(0) +
    \int_0^{kT} e^{\mathbf{A}((k + 1)T - \tau)}\mathbf{B}\mathbf{u}(\tau) \,d\tau \\
&\quad +
    \int_{kT}^{(k + 1)T} e^{\mathbf{A}(kT + T - \tau)}\mathbf{B}\mathbf{u}(\tau) \,d\tau \\
\mathbf{x}_{k+1} &= e^{\mathbf{A}T} \underbrace{\left(e^{\mathbf{A}kT}\mathbf{x}(0) +
    \int_0^{kT} e^{\mathbf{A}(kT - \tau)}\mathbf{B}\mathbf{u}(\tau)
    \,d\tau\right)}_{\mathbf{x}_k} \\
&\quad +
    \int_{kT}^{(k + 1)T} e^{\mathbf{A}(kT + T - \tau)}\mathbf{B}\mathbf{u}(\tau) \,d\tau
\end{aligned}
$$

We assume that $\mathbf{u}$ is constant during each timestep, so it can be pulled out of the integral.

$$
\mathbf{x}_{k+1} = e^{\mathbf{A}T}\mathbf{x}_k +
    \left(\int_{kT}^{(k + 1)T} e^{\mathbf{A}(kT + T - \tau)} \,d\tau\right)
    \mathbf{B}\mathbf{u}_k
$$

The second term can be simplified by substituting it with the function $v(\tau) = kT + T - \tau$. Note that $d\tau = -dv$.

$$
\begin{aligned}
\mathbf{x}_{k+1} &= e^{\mathbf{A}T}\mathbf{x}_k -
    \left(\int_{v(kT)}^{v((k + 1)T)} e^{\mathbf{A}v} \,dv\right)
    \mathbf{B}\mathbf{u}_k \\
\mathbf{x}_{k+1} &= e^{\mathbf{A}T}\mathbf{x}_k -
    \left(\int_T^0 e^{\mathbf{A}v} \,dv\right) \mathbf{B}\mathbf{u}_k \\
\mathbf{x}_{k+1} &= e^{\mathbf{A}T}\mathbf{x}_k +
    \left(\int_0^T e^{\mathbf{A}v} \,dv\right) \mathbf{B}\mathbf{u}_k \\
\mathbf{x}_{k+1} &= e^{\mathbf{A}T}\mathbf{x}_k +
    \mathbf{A}^{-1}e^{\mathbf{A}v} \rvert_0^T \mathbf{B}\mathbf{u}_k \\
\mathbf{x}_{k+1} &= e^{\mathbf{A}T}\mathbf{x}_k +
    \mathbf{A}^{-1}(e^{\mathbf{A}T} - e^{\mathbf{A}0}) \mathbf{B}\mathbf{u}_k \\
\mathbf{x}_{k+1} &= e^{\mathbf{A}T}\mathbf{x}_k +
    \mathbf{A}^{-1}(e^{\mathbf{A}T} - \mathbf{I}) \mathbf{B}\mathbf{u}_k
\end{aligned}
$$

which is an exact solution to the discretization problem.

## D.2 Kalman filter as Luenberger observer

A Luenberger observer is defined as

$$
\begin{aligned}
\hat{\mathbf{x}}_{k+1}^+ &= \mathbf{A}\hat{\mathbf{x}}_k^- + \mathbf{B}\mathbf{u}_k + \mathbf{L}
    (\mathbf{y}_k - \hat{\mathbf{y}}_k) \\
\hat{\mathbf{y}}_k &= \mathbf{C} \hat{\mathbf{x}}_k^-
\end{aligned} \tag{D.1\text{--}D.2}
$$

where a superscript of minus denotes *a priori* and plus denotes *a posteriori* estimate. Combining equation (D.1) and equation (D.2) gives

$$
\hat{\mathbf{x}}_{k+1}^+ = \mathbf{A}\hat{\mathbf{x}}_k^- + \mathbf{B}\mathbf{u}_k + \mathbf{L}
    (\mathbf{y}_k - \mathbf{C}\hat{\mathbf{x}}_k^-) \tag{D.3}
$$

The following is a Kalman filter that considers the current update step and the next predict step together rather than the current predict step and current update step.

$$
\begin{aligned}
\text{Update step} \\
\mathbf{K}_k &= \mathbf{P}_k^- \mathbf{C}^{\mathsf{T}} (\mathbf{C}\mathbf{P}_k^- \mathbf{C}^{\mathsf{T}} +
    \mathbf{R})^{-1} \\
\hat{\mathbf{x}}_k^+ &= \hat{\mathbf{x}}_k^- + \mathbf{K}_k(\mathbf{y}_k -
    \mathbf{C}\hat{\mathbf{x}}_k^-) \\
\mathbf{P}_k^+ &= (\mathbf{I} - \mathbf{K}_k\mathbf{C})\mathbf{P}_k^- \\
\text{Predict step} \\
\hat{\mathbf{x}}_{k+1}^+ &= \mathbf{A}\hat{\mathbf{x}}_k^+ + \mathbf{B}\mathbf{u}_k \\
\mathbf{P}_{k+1}^- &= \mathbf{A} \mathbf{P}_k^+ \mathbf{A}^{\mathsf{T}} + \mathbf{Q}
\end{aligned} \tag{D.5\text{--}D.7}
$$

Substitute equation (D.5) into equation (D.7).

$$
\begin{aligned}
\hat{\mathbf{x}}_{k+1}^+ &= \mathbf{A}(\hat{\mathbf{x}}_k^- + \mathbf{K}_k(\mathbf{y}_k -
    \mathbf{C}\hat{\mathbf{x}}_k^-)) + \mathbf{B}\mathbf{u}_k \\
\hat{\mathbf{x}}_{k+1}^+ &= \mathbf{A}\mathbf{x}_k^- + \mathbf{A}\mathbf{K}_k(\mathbf{y}_k -
    \mathbf{C}\hat{\mathbf{x}}_k^-) + \mathbf{B}\mathbf{u}_k \\
\hat{\mathbf{x}}_{k+1}^+ &= \mathbf{A}\hat{\mathbf{x}}_k^- + \mathbf{B}\mathbf{u}_k +
    \mathbf{A}\mathbf{K}_k(\mathbf{y}_k - \mathbf{C}\hat{\mathbf{x}}_k^-)
\end{aligned}
$$

Let $\mathbf{L} = \mathbf{A} \mathbf{K}_k$.

$$
\begin{aligned}
\hat{\mathbf{x}}_{k+1}^+ &= \mathbf{A}\hat{\mathbf{x}}_k^- + \mathbf{B}\mathbf{u}_k + \mathbf{L}
    (\mathbf{y}_k - \mathbf{C}\hat{\mathbf{x}}_k^-)
\end{aligned}
$$

which matches equation (D.3). Therefore, the eigenvalues of the Kalman filter observer can be obtained by

$$
\begin{aligned}
&\operatorname{eig}(\mathbf{A} - \mathbf{L}\mathbf{C}) \\
&\operatorname{eig}(\mathbf{A} - (\mathbf{A}\mathbf{K}_k)(\mathbf{C})) \\
&\operatorname{eig}(\mathbf{A}(\mathbf{I} - \mathbf{K}_k\mathbf{C}))
\end{aligned}
$$

### D.2.1 Luenberger observer with separate prediction and update

To run a Luenberger observer with separate prediction and update steps, substitute the relationship between the Luenberger observer and Kalman filter matrices derived above into the Kalman filter equations.

Appendix [D.2](#d2-kalman-filter-as-luenberger-observer) shows that $\mathbf{L} = \mathbf{A}\mathbf{K}_k$. Since $\mathbf{L}$ and $\mathbf{A}$ are constant, one must assume $\mathbf{K}_k$ has reached steady-state. Then, $\mathbf{K} = \mathbf{A}^{-1}\mathbf{L}$. Substitute this into the Kalman filter update equation.

$$
\begin{aligned}
\hat{\mathbf{x}}_{k+1}^+ &= \hat{\mathbf{x}}_{k+1}^- + \mathbf{K}(\mathbf{y}_{k+1} -
    \mathbf{C}\hat{\mathbf{x}}_{k+1}^-) \\
\hat{\mathbf{x}}_{k+1}^+ &= \hat{\mathbf{x}}_{k+1}^- + \mathbf{A}^{-1}\mathbf{L}
    (\mathbf{y}_{k+1} - \mathbf{C}\hat{\mathbf{x}}_{k+1}^-)
\end{aligned}
$$

Substitute in equation (9.4).

$$
\begin{aligned}
\hat{\mathbf{x}}_{k+1}^+ &= \hat{\mathbf{x}}_{k+1}^- + \mathbf{A}^{-1}\mathbf{L}
    (\mathbf{y}_{k+1} - \hat{\mathbf{y}}_{k+1})
\end{aligned}
$$

The predict step is the same as the Kalman filter’s. Therefore, a Luenberger observer run with prediction and update steps is written as follows.

$$
\begin{aligned}
\text{Predict step} \\
\hat{\mathbf{x}}_{k+1}^- &= \mathbf{A}\hat{\mathbf{x}}_k^- + \mathbf{B}\mathbf{u}_k \\
\text{Update step} \\
\hat{\mathbf{x}}_{k+1}^+ &= \hat{\mathbf{x}}_{k+1}^- + \mathbf{A}^{-1}\mathbf{L}
    (\mathbf{y}_{k+1} - \hat{\mathbf{y}}_{k+1}) \\
\hat{\mathbf{y}}_{k+1} &= \mathbf{C} \hat{\mathbf{x}}_{k+1}^-
\end{aligned}
$$
