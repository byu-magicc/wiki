# Overview of Control Systems

## PID Control
The technique of PID control is used both in the lab, and widely in industrial control systems. It's probably the simplest and oldest form of control and is found in your car's cruise control, almost all of the autopilots we use in the lab, and tons of other places. PID controls is named because of the 3 parts of the algorithm:

P = proportional I = integral D = derivative

Not all systems require all three components, and you will learn about each one individually starting with P controls, but just so you can get an idea of the big picture before focusing in on each constituent part, the ensemble of PID controls is briefly presented now.

From Wikipedia, "A PID controller continuously calculates an 'error value' as the difference between a measured variable and a desired setpoint." In other words, it is how we get an autonomous vehicle to move from one point to another.

The equation looks like this

$$u(t) = K_p e(t) + K_i \int^t_0{e(\tau)} d\tau + K_d\frac{d_e}{d_t}$$

You can see the three coefficients, K, each multiplied by an error value, e(t)

P -> accounts for present values of error

I -> accounts for past values of error. Error accumulates over time through use of integration and is added to the proportional term to help reduce error value if proptional term isn't doing enough.

D -> predicts and accounts for future values of error, based on current rate of change of the error.
