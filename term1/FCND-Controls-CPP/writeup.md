# Control of a 3D Quadrotor - Writeup
____
## Implemented Controller
1.  Implement body rate control in C++.  
The body rate control was implemented in the function ```QuadControl::BodyRateControl```, in lines 94-122. This is a P controller, governed by
$$
pqr_c = I k_{p-pqr}(pqr_{t} - pqr_{a})
$$

2. Implement roll pitch control in C++.  
The roll-pitch controller was implemented in the function ```QuadControl::RollPitchControl```, in lines 125-170. Those are porportional controllers. Here, the collective thrust was divided by the mass in order to obtain the collective acceleration of the quadrotor. The commanded acceleration for both body rates is constraint into the maximum tilt angle range.  
Using the equation  
$$
\begin{pmatrix} p_c \\ q_c \end{pmatrix} = \frac{1}{R_{33}}\begin{pmatrix}R_{21} & -R_{11} \\ R_{22} & -R_{12} \end{pmatrix} \times \begin{pmatrix} \dot{b}^x_c \\ \dot{b}^y_c \end{pmatrix}
$$
it is possible to determine the desired roll and pitch rates.

3. Implement altitude controller in C++.  
The altitude controller was implemented in the function ```QuadControl::AltitudeControl```, in lines 172-213. A PID controller helps us to achieve the desired acceleration, i.e. the collective thrust, as well as to cope with possible quadrotor's weight instabilities.  
From the equation
$$
\begin{pmatrix}\ddot{x} \\ \ddot{y} \\ \ddot{z}\end{pmatrix} = \begin{pmatrix}0 \\ 0\\ g\end{pmatrix} + R \begin{pmatrix}0 \\ 0\\ c\end{pmatrix}
$$
and other values not included for reading easiness, it is possible to estimate the control value as follows
$$
\bar{u}_1 = k_{p-z}(z_t - z_a) + k_{d-z}(\dot{z}_t - \dot{z}_a) + \ddot{z}_t
$$

4. Implement lateral position control in C++.  
The lateral position controller was implemented in the function ```QuadControl::LateralPositionControl```, in lines 215-273. Similar to the pitch/roll controller, this is another P controller, for both position and velocity, including the feed-forward value of the acceleration, using
$$
\mathbf{\ddot{x}} = k_{p-\mathbf{x}}(\mathbf{x}_t - \mathbf{x}_a) + k_{p-\mathbf{\dot{x}}}(\mathbf{\dot{x}}_t - \mathbf{\dot{x}}_a) + \mathbf{\ddot{x}}_{ff}
$$

5. Implement yaw control in C++.  
The yaw controller was implemented in the function ```QuadControl::YawControl```, in lines 276-313. A P controller is good enough to handle the yaw rate commands. It is important to notice that the yaw values should be between _-pi_ and _pi_.

6. Implement calculating the rotor commands given the commanded thrust and moments in C++.  
The implementation was done in the function ```QuadControl::GenerateMotorCommands```, in lines 56-92. Here was crucial to resolve a linear system of 4 equations and 4 variables, in order to obtain the thrust for each rotor, taking into account the desired moments for the quadrotor to have.

## Flight Evaluation
Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.  

The outcome messages from the simulator for each scenario are included here:

```
Scenario 1
(../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
```
```
Scenario 2
(../config/2_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
```
```
Scenario 3
(../config/3_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
```
```
Scenario 4
(../config/4_Nonidealities.txt)
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
```
```
Scenario 5
(../config/5_TrajectoryFollow.txt)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
```