Headless vs Graphical Linux
===========================

When running your robot in hardware demonstrations and data gathering sessions, it is advantageous to run your companion computer in a **headless** (or text-based) mode with no Desktop Environment. This will save on resources like CPU and RAM, giving your algorithms slightly more computational power.

To do this, we will use `systemctl` commands. For a brief overview of `systemctl`, read the [Startup Scripts with systemd](systemd.md) article.

## Making Headless the Default Configuration ##

We will make the non-graphical boot the default configuration so that there is very little to do in the field. To change the default run the following commands

```bash
$ sudo systemctl set-default -f multi-user.target
```

Double-check by running

```bash
$ sudo systemctl get-default
```

You can change into the ''multi-user'' target by rebooting or running the following command

```bash
$ sudo systemctl enable multi-user.target
```

Note that on a reboot, if you have your computer connected to a monitor it will show the output of a form of `dmesg`. To get to a login screen you must change tty by using `Ctrl+Alt+F1`.

## Starting the Graphical Environment from Headless ##

To get back into a graphical desktop environment, run the following command

```bash
$ sudo systemctl enable graphical.target
```

You can always switch back to having graphical boot being the default with

```bash
$ sudo systemctl set-default -f graphical.target
```

## Resources ##

- [Non-graphical boot with systemd](https://unix.stackexchange.com/questions/164005/non-graphical-boot-with-systemd)
