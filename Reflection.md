# The Model

Student describes their model in detail. This includes the state, actuators and update equations.
The state consists of the following:
* X position
* Y position
* Angle of the vehicle's rotation
* Current speed
* Cross-track error - How far off are we from the target position
* Orientation Error - How different is the current car angle from the desired one

Due to the used translation & rotation, the first three values are initially always 0

The actuators are:
* Steering angle - from -25 to +25 degrees in this case
* Acceleration/Braking - from -1.0 to 1.0

Update equations from the "mpc_to_line" project are used:
```cpp
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] =
    cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] =
    epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
* The x and y use the old position, car's angle and speed to determine the new x and y
* The new angle is determined by the old one, the steering actuator and how fast is the car moving
* The new speed is determined by the old one and how much are we accelerating or braking (taken from the actuators)
* The new CTE is determined by using the current CTE and the change caused by the vehicle's movement  `v0 * CppAD::sin(epsi0) * dt`
* The new orientation error is determined by using the current orientation error and the change caused by the vehicle's movement  `v0 * delta0 / Lf * dt`

# Timestep Length and Elapsed Duration (N & dt)

I use the N and dt from the "mpc_to_line" quiz and they seem to work fine

# Polynomial Fitting and MPC Preprocessing
Before fitting the polynomial, I translate and rotate all waypoints to the car's coordinate system.

This makes the position for the MPC always 0, 0 and makes a lot of the calculations much easier (for the epsi, CTE and etc.)


# Model Predictive Control with Latency

In order to compensate for latency, I predict the car position after 100 ms. This is done by the following code:
```cpp
double dist = v * 0.0445788;
px += cos(psi) * dist;
py += sin(psi) * dist;
```
0.0445788 is calculated by converting miles to meters and hours to miliseconds
