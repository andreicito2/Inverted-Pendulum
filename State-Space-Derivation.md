The Lagrangian $(\mathcal{L})$ is the difference between the kinetic energy $( T )$ and potential energy $( V )$ of the system:
$$
\mathcal{L} = T - V
$$

**System Description:**
- $ x $: horizontal position of the cart
- $ \theta $: angle of the pendulum from the vertical (upwards is \( \theta = 0 \))
- $ M $: mass of the cart
- $ m $: mass of the pendulum
- $ l $: length to the pendulum's center of mass
- $ g $: acceleration due to gravity
- $ I $: moment of inertia of the pendulum about its center of mass

**Kinetic Energy $( T )$:**
$$
T = \frac{1}{2} M \dot{x}^2 + \frac{1}{2} m \left[ (\dot{x} + l \dot{\theta} \cos\theta)^2 + (l \dot{\theta} \sin\theta)^2 \right] + \frac{1}{2}I \dot{\theta}^2
$$
Expanding and simplifying:
$$
T = \frac{1}{2} (M + m) \dot{x}^2 + m l \dot{x} \dot{\theta} \cos\theta + \frac{1}{2} m l^2 \dot{\theta}^2 + \frac{1}{2} I \dot{\theta}^2
$$

**Potential Energy $( V )$:**
$$
V = m g l \cos\theta
$$

**Lagrangian $(\mathcal{L})$:**
$$
\mathcal{L} = T - V = \frac{1}{2} (M + m) \dot{x}^2 + m l \dot{x} \dot{\theta} \cos\theta + \frac{1}{2} m l^2 \dot{\theta}^2 + \frac{1}{2} I \dot{\theta}^2 - m g l \cos\theta
$$

$$
\mathcal{L} = \frac{1}{2} (M + m) \dot{x}^2 + \frac{1}{2} (m l^2 + I) \dot{\theta}^2 - m l \cos\theta (\dot{x} \dot{\theta} - g)
$$

**Euler-Lagrange Equations:**
The Euler-Lagrange equations for the system are given by:
$$
\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}} \right) - \frac{\partial \mathcal{L}}{\partial q} = Q_i
$$
where $ q $ represents the generalized coordinates $ x $ and $ \theta $, and $ Q_i $ are the generalized forces.

**For x:**
$$
\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{x}} \right) - \frac{\partial \mathcal{L}}{\partial x} = u
$$
Calculating the derivatives:
$$
\frac{\partial \mathcal{L}}{\partial \dot{x}} = (M + m) \dot{x} + m l \dot{\theta} \cos\theta
$$
$$
\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{x}} \right) = (M + m) \ddot{x} + m l \ddot{\theta} \cos\theta - m l \dot{\theta}^2 \sin\theta
$$
$$
\frac{\partial \mathcal{L}}{\partial x} = 0
$$
Thus, the equation becomes:
$$
(M + m) \ddot{x} + m l \ddot{\theta} \cos\theta - m l \dot{\theta}^2 \sin\theta = u
$$

**For $\theta$:**
$$
\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} \right) - \frac{\partial \mathcal{L}}{\partial \theta} = 0
$$
Calculating the derivatives:
$$
\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = (m l^2 + I) \dot{\theta} + m l \dot{x} \cos\theta
$$
$$
\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} \right) = (m l^2 + I) \ddot{\theta} + m l \ddot{x} \cos\theta - m l \dot{x} \dot{\theta} \sin\theta
$$
$$
\frac{\partial \mathcal{L}}{\partial \theta} = -m l \sin\theta (\dot{x} \dot{\theta} - g)
$$

Thus, the equation becomes:
$$
(m l^2 + I) \ddot{\theta} + m l \ddot{x} \cos\theta - m l g \sin\theta = 0
$$

**Linearize the Equations of Motion:**
To linearize the equations of motion, we assume small angles for the pendulum, which allows us to use the approximations $ \sin\theta \approx \theta $, $ \cos\theta \approx 1 $, and neglect higher-order terms.
The equations of motion become:
1. For x:
$$
(M + m) \ddot{x} + m l \ddot{\theta} = u
$$
2. For $\theta$:
$$
(m l^2 + I) \ddot{\theta} + m l \ddot{x} - m l g \theta = 0
$$

**Add Damping Terms:**
To include damping, we add terms proportional to the velocities:
1. For x:
$$
(M + m) \ddot{x} + c_x \dot{x} + m l \ddot{\theta} = u
$$
2. For $\theta$:
$$
(m l^2 + I) \ddot{\theta} + c_\theta \dot{\theta} + m l \ddot{x} - m l g \theta = 0
$$

**Solve for Accelerations in Matrix Form:**
First we rearrange the equations to isolate the accelerations $ \ddot{x} $ and $ \ddot{\theta} $:
1. For $ \ddot{x} $:
$$
(M+m) \ddot{x} + m l \ddot{\theta} = u - c_x \dot{x}
$$
2. For $ \ddot{\theta} $:
$$
(m l^2 + I) \ddot{\theta}  + m l \ddot{x} = - m l g \theta - c_\theta \dot{\theta}
$$
To express the equations in matrix form, we can write:
$$
\begin{bmatrix}
(M + m) & 0 \\
m l & (m l^2 + I)
\end{bmatrix}
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
u - c_x \dot{x} \\
- m l g \theta - c_\theta \dot{\theta}
\end{bmatrix}
$$

We can solve for the accelerations using the inverse of the coefficient matrix M:
$$
M = \begin{bmatrix}
(M + m) & 0 \\
-m l & (m l^2 + I)
\end{bmatrix}
$$
$$
M^{-1} = \frac{1}{\det(M)}
\begin{bmatrix}
(m l^2 + I) & -m l \\
-m l & (M + m)
\end{bmatrix}
$$
The determinant \( \det(M) \) can be calculated as:
$$
\det(M) = (M + m)(m l^2 + I) - m^2 l^2
$$

Finally, we can express the accelerations $ \ddot{x} $ and $ \ddot{\theta} $ as:
$$
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
=
M^{-1}
\begin{bmatrix}
u - c_x \dot{x} \\
- m l g \theta - c_\theta \dot{\theta}
\end{bmatrix}
$$
$$
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
=
\frac{1}{\det(M)}
\begin{bmatrix}
(m l^2 + I) & -m l \\
-m l & (M + m)
\end{bmatrix}
\begin{bmatrix}
u - c_x \dot{x} \\
- m l g \theta - c_\theta \dot{\theta}
\end{bmatrix}
$$

For simplicity's sake, we can denote the determinant as $ \Delta $:
$$
\Delta = (M + m)(m l^2 + I) - m^2 l^2
$$
Thus, the final expressions for the accelerations are:
$$
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
=
\frac{1}{\Delta}
\begin{bmatrix}
(m l^2 + I) & -m l \\
-m l & (M + m)
\end{bmatrix}
\begin{bmatrix}
u - c_x \dot{x} \\
- m l g \theta - c_\theta \dot{\theta}
\end{bmatrix}
$$

We can now evaluate the separate components of the accelerations:
1. For $ \ddot{x} $:
$$
\ddot{x} = \frac{1}{\Delta} \left[ (m l^2 + I)(u - c_x \dot{x}) - m l (- m l g \theta - c_\theta \dot{\theta}) \right]
$$
2. For $ \ddot{\theta} $:
$$
\ddot{\theta} = \frac{1}{\Delta} \left[ -m l (u - c_x \dot{x}) + (M + m)(- m l g \theta - c_\theta \dot{\theta}) \right]
$$

Finally, we can create the state-space matrices A and B for the system:
$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\frac{(m l^2 + I) c_x}{\Delta} & -\frac{m^2 l^2 g}{\Delta} & \frac{m l c_\theta}{\Delta} \\
0 & 0 & 0 & 1 \\
0 & -\frac{m l c_x}{\Delta} & \frac{(M+m)m l g}{\Delta} & \frac{(M+m) c_\theta}{\Delta}
\end{bmatrix}
$$
$$
B = \begin{bmatrix}
0 \\
\frac{m l^2 + I}{\Delta} \\ 
0 \\ 
\frac{m l}{\Delta}
\end{bmatrix}
$$
