# Omnidirectional Robot Kinematics and Odometry

- [Kinematics](#kinematics)
  - [Inverse](#inverse)
  - [Forward](#forward)
- [Odometry](#odometry)
- [References](#references)

## Kinematics

The figure below describe the geometry paramaters used in the forward and inverse kinematics implemented on this controller.

<p align="center">
  <img src="https://github.com/mateusmenezes95/omnidirectional_controllers/blob/foxy/doc/images/axebot-kinematic-frames.jpg" />
  <br>
  <font size="2">Source: [1]</font>
</p>

### Inverse

From the geometry analysis in the above figure, we have that:

```math
\begin{align}
  v_{m_1} &= -v_n + \omega L\\
  v_{m_2} &=v\cos(\gamma) + v_n\sin(\gamma) + \omega L\\
  v_{m_3} &=-v\cos(\gamma) + v_n\sin(\gamma) + \omega L
\end{align}
```

Or in Matrix form:

```math
\begin{equation}
  \begin{bmatrix}
    v_{m_1}\\
    v_{m_2}\\
    v_{m_3}
  \end{bmatrix}=
  \begin{bmatrix}
    0            &           -1 & L\\
    \cos(\gamma) & \sin(\gamma) & L\\
    -\cos(\gamma) & \sin(\gamma) & L\\
  \end{bmatrix}
  \begin{bmatrix}
    v\\
    v_{n}\\
    \omega
  \end{bmatrix}
\end{equation}
```

For the wheels’ angular velocities, assuming that all wheels have the same radius $r$

```math
\begin{equation}
  \begin{bmatrix}
    \omega_{m_1}\\
    \omega_{m_2}\\
    \omega_{m_3}
  \end{bmatrix}
  =
  \frac{1}{r}
  \begin{bmatrix}
    0            &           -1 & L\\
    \cos(\gamma) & \sin(\gamma) & L\\
    -\cos(\gamma) & \sin(\gamma) & L\\
    \end{bmatrix}
  \begin{bmatrix}
    v\\
    v_{n}\\
    \omega
  \end{bmatrix}
\end{equation}
```

### Forward

Manipulating the inverse kinematics, we get in

```math
\begin{equation}
  \begin{bmatrix}
    v\\
    v_{n}\\
    \omega
  \end{bmatrix}=r
  \begin{bmatrix}
    0&\frac{1}{2\cos(\gamma)}&\frac{-1}{2\cos(\gamma)}\\
    \frac{-1}{\sin(\gamma)+1}&\frac{1}{2(\sin(\gamma)+1)}&\frac{1}{2(\sin(\gamma)+1)}\\
    \frac{\sin(\gamma)}{L(\sin(\gamma)+1)}&\frac{1}{2L(\sin(\gamma)+1)}&\frac{1}{2L(\sin(\gamma)+1)}
  \end{bmatrix}
  \begin{bmatrix}
    \omega_{m_1}\\
    \omega_{m_2}\\
    \omega_{m_3}
  \end{bmatrix}
\end{equation}
```

Or

```math
\begin{equation}
  \begin{bmatrix}
    v\\
    v_{n}\\
    \omega
  \end{bmatrix}
  =
  rG^{-1}
  \begin{bmatrix}
    \omega_{m_1}\\
    \omega_{m_2}\\
    \omega_{m_3}
  \end{bmatrix}
\end{equation}
```

Whereby

```math
\begin{equation}
  G^{-1} =
  \begin{bmatrix}
    0&\frac{1}{2\cos(\gamma)}&\frac{-1}{2\cos(\gamma)}\\
    \frac{-1}{\sin(\gamma)+1}&\frac{1}{2(\sin(\gamma)+1)}&\frac{1}{2(\sin(\gamma)+1)}\\
    \frac{\sin(\gamma)}{L(\sin(\gamma)+1)}&\frac{1}{2L(\sin(\gamma)+1)}&\frac{1}{2L(\sin(\gamma)+1)}
  \end{bmatrix}
\end{equation}
```

Given that $\beta = \frac{1}{2\cos(\gamma)}$ and $\alpha = \frac{1}{\sin(\gamma)+1}$, we have

```math
\begin{equation}
  G^{-1}=
  \begin{bmatrix}
    0 & \beta & -\beta\\
    -\alpha & 0.5\alpha & 0.5\alpha\\
    L^{-1}\sin(\gamma)\alpha & 0.5L^{-1}\alpha & 0.5L^{-1}\alpha
  \end{bmatrix}
\end{equation}
```

Therefore

```math
\begin{align}
  v &= \beta(\omega_{m_2} - \omega_{m_3})\\
  v_n &=\alpha(-w_{m_1} +0.5(w_{m_2}+w_{m_3}))\\
  \omega &= L^{-1}\alpha(\sin(\gamma)w_{m_1}+0.5(w_{m_2}+w_{m_3}))
\end{align}
```

## Odometry

The robot’s velocity with regard to the world (w.r.t) is given by

```math
\begin{equation}
  \begin{bmatrix}
    \dot{x}_r\\
    \dot{y}_r\\
    \dot{\theta}
  \end{bmatrix}
  =
  R(\theta)
  \begin{bmatrix}
    v\\
    v_{n}\\
    \omega
  \end{bmatrix}
\end{equation}
```

where $R(\theta)$ is the rotation matrix around the $z$ axis perpendicular to the horizontal plane $xy$, given by

```math
\begin{equation}
  R(\theta)
  =
  \begin{bmatrix}
    \cos(\theta)  & -\sin(\theta) & 0\\
    \sin(\theta) & \cos(\theta) & 0\\
    0             & 0            & 1
  \end{bmatrix}
\end{equation}
```

Assume that the robot configuration $q(t_k) = q_k = (x_k,y_k,\theta_k)$ at the sampling time $t_k$ is known, together with the velocity inputs $v_k$,  $v_{n_k}$, and  $w_k$ applied in the interval $[t_k,\:t_{k+1})$. The value of the configuration variable $q_{k+1}$ at the sampling time $t_{k+1}$ can then be reconstructed by forward integration of the kinematic model. Adapted from [2].

Considering that the sampling period $T_s = t_{k+1} - t_k$, the Euler method for the axebot configuration $q_{k+1}$ is given by

```math
\begin{align}
  x_{k+1} &= x_k + (v\cos(\theta_k)-v_n\sin(\theta_k))T_s\\
  y_{k+1} &= y_k + (v\sin(\theta_k)+v_n\cos(\theta_k))T_s\\
  \theta_{k+1} &= \theta_k + \omega_kT_s
\end{align}
```

And for the second-order Runge Kutta integration method

```math
\begin{align}
  \bar{\theta}_k &= \theta_k+\frac{w_kTs}{2}\\
  x_{k+1} &= x_k + (v\cos(\bar{\theta}_k)-v_n\sin(\bar{\theta}_k))T_s\\
  y_{k+1} &= y_k + (v\sin(\bar{\theta}_k)+v_n\cos(\bar{\theta}_k))T_s\\
  \theta_{k+1} &= \theta_k + \omega T_s
\end{align}
```

## References

[1] J. Santos, A. Conceição, T. Santos, and H. Ara ujo, “Remote control of an omnidirectional mobile robot with time-varying delay and noise attenuation,” *Mechatronics*, vol. 52, pp. 7–21, 2018, ISSN: 0957-4158. DOI: 10.1016/j.mechatronics.2018.04.003. [Online](https://www.sciencedirect.com/science/article/abs/pii/S0957415818300606).

[2] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Robotics: Modelling, Planning and Control*, 1st ed. Springer Publishing Company, Incorporated, 2008.
