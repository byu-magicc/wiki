# Environment Setup #

## Automated Environment Setup ##
---
We have set up a number of scripts to assist you in setting up your computer, after you have installed ROS. Here is a link to the scripts folder [Scripts]() and use the following credentials

```bash
username= "relative_nav"
password= "hex"
```
Then, bring up a terminal (Ctrl+Alt+T) and type the following commands. The file will always be a little different, so fill in the blank (or just press tab to auto-complete) when it says <rest of file name>

```bash
cd ~
sudo apt update
mkdir scripts
cd Downloads
ls
mv scripts.git<rest of file name>.zip ../scripts
cd ../scripts
unzip scripts.git<rest of file name>
cd installation
./toplevel
```
When running this script, you should answer with the following

  * **N**, you are not a cave computer
  * **Y**, you want to copy the bashrc file
  * your ROS_MASTER_URI is **local** (no quotes)
  * your magicc username is **firstname_lastname** (your actual first name and last name)

This script installs git, vim, a whole bunch of environment customizations, ROS, all the ROS packages needed to run the relative nav stack, Google Chrome, Sublime, QtCreator, Spotify, and the Numix Custom theme. It can take more than hour, depending mainly on your internet connection, because it downloads several gigabytes of data over the course of installation.

Then to apply the changes you just made to your environment,
```bash
cd ~
source ~/.bashrc
```
You'll know if it worked if your terminal prompt is colored green

## Environment Variables ##
---
One of the things that can often go wrong in getting programs to work properly is the "environment variables." These variables are sort of like global variables in scope of the entire operating system, that tell it where to find different libraries, how commands in the terminal get interpreted, the name of your system, and so on. Windows and Mac both actually have environment variables as well, but it's not often that you have to manage them.

Let's look at our environment variables, just for fun. Open up a terminal and type

```bash
printenv
```
A big long list of variables will show up. This list includes everything from where to find the python library, to what type of Linux you are running. It's sometimes unmanageable to go through the whole list, so let's use grep to filter through the list to see what we want. For example, let's use these commands to see what Linux thinks our active ROS workspace is.

```bash
printenv | grep ROS
```
So, this command prints the whole list of variables and pushes it straight into grep, which returns only lines that have the phrase "ROS" in them. What you end up with is a list of variables that have to do with ROS. Look at your ROSLISP_PACKAGE_DIRECTORIES variable. It should include your active workspace. If the ros shortcuts "roscd" and "roslaunch" aren't working, this is sometimes the culprit.

To change environment variables (which shouldn't be done on a regular basis, except for perhaps ROS_MASTER_URI and ROS_IP), type

```bash
export <VARIABLE>=<new_value>
```
So, an example would be as follows:
```bash
export ROS_MASTER_URI=http://localhost:11311
```
It should be noted that you can create environment variables this way as well. These variables are loaded when the terminal is started, so if you want the variable to be a part of every session, you would you have to put it in the .bashrc script, which brings us to the next section!

## The .bashrc file ##

Every time you open a terminal window, it runs a special script called your "bashrc" This script often calls other scripts such as the rosrc file we use in the lab

Let's look at our bashrc
```bash
vim ~/.bashrc
```

It's sort of confusing to read the file, but there are a few key words you should know.
```bash
alias <command>='<replacement command>'
```
This allows you to shorten commands. For example, I have the following line in my .bashrc
```bash
alias grep='grep --color=auto'
```
This means every time I use the grep command, I don't need to add the --color=auto argument. The terminal will automatically replace "grep" with "grep --color=auto" when it processes the command.

Another keyword you should probably know is
```bash
source <external script file.sh>
```
Source runs the content in some external script. For example, the line you added to the bashrc when you installed ROS was exactly this type of command. It's that particular script that runs and sets up your ROS environment variables.

It's important to note that the .bashrc file is loaded when a new terminal window is opened. That means that if you make changes to the bashrc, then you either need to open a new terminal to see the changes, or type
```bash
source ~/.bashrc
```
to see the changes in your current terminal.

The full version of the bashrc we start with in the MAGICC Lab can be found here: Link. Don't be afraid if you don't understand everything that is going on.

### Colors in the terminal ###

Also, if you want to change your command prompt to something other than the default, look into changing line 117 on the linked file. The PS1 variable defines the prompt color. The actual colors are defined by the code in brackets. For example

```bash
\[\033[01;32m\] -> Means that whatever follows is bold green
\[\033[01;34m\] -> is bold blue and
\[\033[00;33m\] -> is normal yellow
```

The "01;" defines bold or not, and the "3*m" defines the actual color. So, if we look at line 117

```bash
117. export PS1='\[\033[01;32m\]\u\[\033[01;34m\] \w\[\033[00;33m\]$(__git_ps1)\[\033[01;32m\] \$\[\033[00m\] '
```

Let's work through it.
```bash
export PS1=                  -> means we are setting the PS1 variable.
'\[\033[01;32m\]\u           -> sets the username in the bash prompt to be bold green
 \[\033[01;34m\]\w           -> sets the current path to be bold blue
 \[\033[00;33m\]$(__git_ps1) -> means that the git status variable (set when inside a git directory) is yellow
 [\033[01;32m\]\$            -> means that the dollar sign (or # when acting as root) is green
 \[\033[00m\]'               -> means the text you type will be the default text color.
 ```
 Here are the color definitions for reference.
 ```bash
 [00;30m] = normal black         [01;30m] = bold black
[00;31m] = normal red           [01;31m] = bold red
[00;32m] = normal green         [01;32m] = bold green
[00;33m] = normal yellow        [01;33m] = bold yellow
[00;34m] = normal blue          [01;34m] = bold blue
[00;35m] = normal purple        [01;35m] = bold purple
[00;36m] = normal cyan          [01;36m] = bold cyan
[00;37m] = normal white         [01;37m] = bold white
```

## Other Environment Files ##

There are other files we use in the MAGICC lab that function similarly to the .bashrc. These files all live in the home directory, right next to the bashrc and are also run every time the terminal window is opened.

### .rosrc ###

We noticed that we were often changing ROS variables, such as the workspace, ROS_IP and ROS_MASTER_URI. As a result, we created our own file, called .rosrc, and added a line to the .bashrc that just sources the .rosrc file. This helped to separate the ROS environment from the rest of the operating system environment.

The .rosrc file looks as follows:
```bash
1. source set_ros_master local
2. export ROSLAUNCH_SSH_UNKNOWN=1
3.
4. # ROS_WORKSPACE
5. source /opt/ros/indigo/setup.bash
6. #source ~/rel_nav_ws/devel/setup.bash
```
The first line runs the "set_ros_master" script contained in /usr/local/bin. This automatically sets the ROS_MASTER_URI and ROS_HOSTNAME variables. It will also show the state of these variables when it runs if told to do so in the startup script.

The second line is necessary for remote launching of ROS nodes. Not something you need to worry about now.

Lines 4-6 define the workspace. By default, the toplevel ROS workspace sourced. Whenever you make a new workspace, you need to come in and "source" it so ROS can find the packages you've put in it. For example, if I were to make a new workspace called "obstacle" I would have an additional line in my .rosrc that looked like this.

```bash
6. source ~/obstacle/devel/setup.bash
```
After making the change, I have to either

```bash
source ~/.bashrc
```
or if you're using the relative_nav bashrc
```bash
sbashrc
```
is an alias for the same thing.

### .rosalias ###

This is a file that I sometimes have on my systems where I have programmed several aliases specific to ROS. I usually reference it from my .rosrc file. Here is a copy of my ~/.rosalias file
```bash
1. alias fixCMake="rm CMakeLists.txt && cp /opt/ros/indigo/share/catkin/cmake/toplevel.cmake CMakeLists.txt"
2.
3. alias printROS="printenv | grep ROS"
```
The first line is a handy command to fix a common problem when first using qtcreator in a ROS workspace.

The second prints out all the ROS-related environment variables

### .vimrc ###

One last file is the .vimrc file. Whenever vim opens, it first goes through the .vimrc file, and loads a couple of defaults, such as syntax highlighting for files, tab and space defaults, etc...

Here is a copy of my ~/.vimrc file
```bash
"Convenient tab settings
set tabstop=4
set shiftwidth=2
set softtabstop=2
set expandtab

"Sets proper syntax highlighting for .launch, .rosrc, and .rosalias files
au BufRead,BufNewFile *.launch set filetype=xml
au BufRead,BufNewFile *.rosrc set filetype=sh
au BufRead,BufNewFile *.rosalias set filetype=sh
```
Vim also loads macros from the ~/.vim/plugin folder. David wrote a macro that automatically comments and uncomments code by pressing Ctrl+C and Ctrl+X respectively. If you're interested, you can look at that plugin.
