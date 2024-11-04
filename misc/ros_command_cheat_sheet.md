# ROS Command Cheat Sheet #

## roslocate ##

Displays Package Information

```bash
roslocate info [PACKAGE]
```

## roscreate-pkg ##

Create a new package

```bash
roscreate-pkg [package_name] [depend1] [depend2] [depend3]
```

## rospack ##

Reindex packages

```bash
rospack profile
```

Find Packages

```bash
rospack find [PACKAGE]
```

List all nested dependencies

```bash
rospack depends(1) [PACKAGE]
```

## rosnode ##

List current ROS nodes

```bash
rosnode list
```

Detailed information about specific node

```bash
rosnode info [NODE]
```

## roscore ##

Daemon running msg handling/etc

## rosrun ##

Run a ros node

```bash
rosrun [PACKAGE] [NODE]
```

## rostopic ##

List the types of messages being published

```bash
rostopic list
```

for contents of message

```bash
rostopic echo [MESSAGE TOPIC]
```

Publish messages

```bash
rostopic pub [TOPIC] [MSG_TYPE] -- [ARGS]
```

## roslaunch ##

Launch from a launch file

```bash
roslaunch [PACKAGE] [launch-file]
```
