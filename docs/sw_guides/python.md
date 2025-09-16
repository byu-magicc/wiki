Python Know-how
===============

Python is an excellent research and development tool because of its readability, intuitive syntax, and interactive ability. In addition, there is a plethora of popular 3rd party Python packages available for your procurement -- particularly for algorithm development and numerical computation.

## tl;dr ##

- Use Python 3 when you can, but note that doing this is not really an option when creating ROS packages
- ROS uses Python 2.7, ROS2 will use Python 3
- Always use `pip` to install Python packages -- **never use `sudo apt install ...`!** (can make for messy paths)
- It is preferable to use `pip install <package> --user` over `sudo -H pip install <package>`
- **Do not** install `anaconda`. It will probably make your life miserable (especially as a ROS user). Anaconda is nice for users who use Windows or who just want to start coding without learning about the Python ecosystem.
- Although you will often hear/read people refer to Python and `pip` without a version number, it is important to be aware of your system configuration and be specific when necessary. Always use the appropriate versions: `python3 main.py` or `pip3 install ... --user`.
- Virtual environments are good. But you probably won't see them used much in our ROS/research world.

## Python Versions ##

At the time of this writing, there are commonly two versions of Python in use: versions `2.7.14` and `3.6.4`. This has historically (and probably currently) caused much heartache for seasoned and beginning Python developers alike. Further, Python does not really help itself with its use of `PYTHONPATH` and its package management. Because of this, it is important to be aware of the tools and commands for installing packages into their proper place. The aim of this article is to document what we've learned about best practices with using different Python versions on the same machine.

## Getting Started with Python ##

On an Ubuntu 16.04 machine, system level `python` and `python3` will already be installed on your machine. Use `which` to see the path of the binary (i.e., `which python3`).

Install `pip` with

```bash
$ sudo apt install python-pip # or python3-pip
```

**Attention:** This is the only exception for installing a Python thing with `apt`

Install `ipython` (interactive Python):

```bash
$ pip install ipython --user
```

Now, instead of using the `python` interpreter, you may wish to use the `ipython` interpreter. It has colors, autocomplete, history, and you can exit with just `exit` instead of `exit()`. Additionally, you can load a file and stop at the end of it with `ipython -i main.py`. This allows you to interact with the program state (i.e., variables, functions, etc) after it has run.

## Python Virtual Environments ##

Because `pip` installs packages for all to see, it can become a hassle to deal with code that needs a specific version of a package -- especially if it is different than your system-wide install. Additionally, it can be unwieldy to have to communicate all your dependencies and their versions to run your package. To combat this, we use Python virtual environments, or `virtualenv`s for short. Using a `virtualenv` with `pip` allows one to create a `requirements.txt` file that can be easily installed into a new environment.

**Attention:** A lot of times, this could be overkill for your project. Virtual environments are the right way to do it, but they may not help you graduate any sooner. Virtual environments are critical in the development/release life cycle of software as it helps keep your development and production environments the same. This fits very well with webservers, but not as well with robots and ROS.

**Note:** that there are many different tools in the Python community (`virtualenv`, `pyenv`, `venv`, `pipenv`, etc). We limit this discussion to what seems to be the standard across the Python community: `virtualenv`. For a great executive summary, see [SO: What is the difference between venv, virtualenvwrapper, etc?](https://stackoverflow.com/a/41573588/2392520).

To setup a `virtualenv`, we will install `virtualenvwrapper`, which contains a nice set of CLI extensions for the `virtualenv` package. For Python 2.7, the installation would look like:

```bash
$ pip install virtualenvwrapper --user
```

Then, add the following to your `~/.bashrc` file and resource (`. ~/.bashrc`):

```bash
export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python # python2 binary
source /home/plusk01/.local/bin/virtualenvwrapper.sh # python2 install
```

You can ensure that the paths are correct with commands like:

```bash
$ which python
$ locate virtualenvwrapper # you may need to run sudo updatedb to refresh the file database
```

Now that everything is setup, run `pip list`. You should see a fair amount of packages listed. If the `virtualenv` is working correctly, those will be hidden once we create a new environment:

```bash
$ mkvirtualenv test # create an environment called 'test' && automatically activates it
(test) $ pip list # should show maybe 2 or 3 packages
```

Notice two things:

1. There should be only 2 or 3 packages from `pip list` (unless you have ROS on your system, see below)
1. The name of your `virtualenv` is prepended to your bash prompt (i.e., `(test)`).

To exit your `virtualenv`, use `deactivate`. To activate it again, run `workon test` (tab complete).

## ROS and Python ##

Because ROS pollutes the global Python namespace by adding its path to the `PYTHONPATH` (you know that line you add to your `~/.bashrc` when you install ROS? that is what does it), you will likely see a whole bunch of ROS specific Python packages when you `pip list` in your `virtualenv`. This is a shame, but most likely won't actually hurt anything.

However, if you would like to gain the satisfaction of having a clean `pip list`, we can add a hook to remove ROS from the `PYTHONPATH` (or just comment out the ROS line in your `~/.bashrc`):

1. Add the following to the `~/.virtualenvs/test/bin/postactivate` hook:

    ```bash
    export PYTHONPATH_TMP=`echo $PYTHONPATH`
    unset PYTHONPATH
    ```

1. Add the following to the `~/.virtualenvs/test/bin/postdeactivate` hook:

    ```bash
    export PYTHONPATH=`echo $PYTHONPATH_TMP`
    unset PYTHONPATH_TMP
    ```

## What does `sudo -H` do? ##

It changes the home directory to be the current user, as opposed to `/root`. From the manual:

```bash
     -H, --set-home
                 Request that the security policy set the HOME environment variable to the home directory specified by the target user's pass‐
                 word database entry.  Depending on the policy, this may be the default behavior.
```

So, for example:

```bash
user@hostname:~$ echo $HOME $USER
/home/user user
user@hostname:~$ sudo bash -c 'echo $HOME $USER'
/home/user root
user@hostname:~$ sudo -H bash -c 'echo $HOME $USER'
/root root
```
