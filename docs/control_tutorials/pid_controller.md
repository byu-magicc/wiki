# PID Controller

## Integration
We are now going to add wind to make our simulation a little more life-like. Go into the turtlesim_dynamics/src folder that you installed in order to complete the PD controller. Edit the turtle.cpp file to have wind by making the wind_x and wind_y parameters non zero values, or simply assign these values in a launch file. If you run the PD controller that you made in the last assignment you will notice that the turtle never reaches his destination because of the wind. You might see something similar to the situation below.

![pid_turtle.png](figures/p_turtle_2.png)

The turtle is trying to reach a destination at coordinates (7, 9). He has turned to face straight into the wind which is blowing from top right to bottom left. You can see his position has stalled at (6.7, 8.7). We can correct this problem by adding the third and final term to our equation, the long awaited integral of error. Our final equation is what you saw in the overview.

$$ u(t) = K_p e(t) + K_i \int{e(\tau)d\tau}{0}{t} + K_d \frac{d_e}{d_t} $$

If the turtle has stalled just shy of the destination, we can sum the error continuously (integrating) and add it to our P and D terms. This is the I term. As time passes and you still haven't reached the destination, the integral builds thus increasing the I term, and the turtle has the extra force needed in order to make the final push home.
