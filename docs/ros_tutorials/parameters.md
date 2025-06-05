## Continually Publishing Messages on Joy

Start by replicating the same scenario we had in the previous tutorial:

![rosgraph2.png](figures/rosgraph2.png)

Now try getting the turtle to go continually in any direction.  You'll notice that the turtle stops after a certain distance.
Now display the output of the joy_node:
``` bash
 rostopic echo joy
```

Now move the joystick. The output should be similar to the following:
```
 ---
 header:
   seq: 3
   stamp:
     secs: 1439228031
     nsecs: 139606633
   frame_id: ''
 axes: [0.13516968488693237, 0.0, 0.0, 0.0, 0.0, 0.0]
 buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
 ---
 header:
   seq: 4
   stamp:
     secs: 1439228031
     nsecs: 248627279
   frame_id: ''
 axes: [0.14702372252941132, 0.0, 0.0, 0.0, 0.0, 0.0]
 buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
 ---
```

Note that joy stops publishing when you stop moving the joystick.  The result is that holding the joystick fixed in any position will cause our turtle to stop moving.  Suppose we wanted to keep publishing at these moments.  Fortunately those who created the joy package built in that capability. Try:
``` bash
 rosrun joy joy_node _autorepeat_rate:=10
```

Joy should now be continually publishing messages at a rate of 10 Hz when no new messages are received.



## Joy Parameters

Take a look at the documentation for joy: [Joy Documentation](https://docs.ros.org/api/joy/html/)

Note that certain ROS Parameters are listed.  These parameters are values that often need to be changed depending on the specific situation.

* 'dev' allows you to change where joy looks in order to find your joystick.
* 'deadzone' allows you to choose the range in which inputs will register as zero.  Different joysticks will require different dead zones.
* 'autorepeat_rate' repeats the previous message received when no new message is received at a specified frequency.
* 'coalesce_interval' is a parameter whithatch can be adjusted in order to reduce the number of messages sent.

By building these parameters into the code, joy can be customized to fit many different situations without requiring a change in the code each time.

Parameters can be changed either by passing them in as we did with the parameter autorepeat_rate (note the underscore preceding the parameter).
``` bash
 rosrun joy joy_node _autorepeat_rate:=10
```

or by changing the parameters value on the ROS Parameter Server using 'rosparam'.



 ## Using rosparam

`rosparam` allows you to store and manipulate data on the ROS [Parameter Server](https://wiki.ros.org/Parameter%20Server). The Parameter Server can store integers, floats, boolean, dictionaries, and lists. rosparam uses the YAML markup language for syntax. In simple cases, YAML looks very natural: `1` is an integer, `1.0` is a float, `one` is a string, `true` is a boolean, `[1, 2, 3]` is a list of integers, and `{a: b, c: d}` is a dictionary. `rosparam` has many commands that can be used on parameters, as shown below:

Usage:
```
 rosparam set		set parameter
 rosparam get		get parameter
 rosparam load		load parameters from file
 rosparam dump		dump parameters to file
 rosparam delete		delete parameter
 rosparam list		list parameter names
```

Let's look at what parameters are currently on the param server:


### rosparam list

``` bash
 rosparam list
```

The turtlesim node has three parameters on the param server for background color.  You can also see that joy_node's autorepeat_rate parameter is on the parameter server:

```
 /background_b
 /background_g
 /background_r
 /joy_node/autorepeat_rate
 /rosdistro
 /roslaunch/uris/host_hulk__41325
 /rosversion
 /run_id
```


### rosparam set

Let's now use 'rosparam set':

Usage:
``` bash
 rosparam set [param_name]
```

Let's change autorepeat_rate's parameter to a lower value using rosparam set:
``` bash
 rosparam set /joy_node/autorepeat_rate 1
```

The command should return our recently set parameter value.
In order for the changes to take effect we need to restart joy_node:
``` bash
 rosrun joy joy_node
```

Or you may have to call 'clear' for new parameters to take effect:
``` bash
 rosservice call clear
```

Now take a look at joy_node's output.  The rate at which messages are published should have significantly decreased.


### rosparam get

We can retrieve the current value of parameters on the parameter server using 'rosparam get':

Usage:
``` bash
 rosparam get [param_name]
```

Let's try using this command with autorepeat_rate:
``` bash
 rosparam get /joy_node/autorepeat_rate
```
The value returned should be the value we set previously using 'rosparam set'
We can also use `rosparam get /` to show us the contents of the entire Parameter Server.
``` bash
 rosparam get /
```

``` bash
 background_b: 255
 background_g: 86
 background_r: 69
 joy_node: {autorepeat_rate: 0.1}
 rosdistro: 'indigo

   '
 roslaunch:
   uris: {host_hulk__41325: 'http://Hulk:41325/'}
 rosversion: '1.11.13

   '
 run_id: 4d2f60a2-3f8c-11e5-8ebb-2c27d731c70e
```


### rosparam dump

If you wish to store all current parameter values in a file you can use 'rosparam dump'.

Usage:
``` bash
 rosparam dump [file_name] [namespace]
```

Here we write all the parameters to the file params.yaml
``` bash
 rosparam dump params.yaml
```


### rosparam load

To load parameter values from a file simply use 'rosparam load'.

Usage:
``` bash
 rosparam load [file_name] [namespace]
```

You can even load these yaml files into new namespaces, e.g. `copy`:
``` bash
 rosparam load params.yaml copy
```

If you wished to get the value of autorepeat_rate within that namespace, you could simply use:
``` bash
 rosparam get copy/joy_node/autorepeat_rate
```
