# Setting Up ssh #

## What is ssh? ##

`ssh` (secure shell) is a program we use to work on remote computers.
It allows you to log in to a remote computer, and open up a terminal window from that remote computer and display it on your screen.
We can use this to work on the companion computer of a multirotor because it is not connected to a keyboard or monitor.
With shell access, we can run commands (e.g. roslaunch), change code, copy a rosbag, move files, etc...

Terminology:
"ssh server": The (usually remote) computer that will be logged into; the machine whose ip address is `<ip.ad.dr.ess>`
"ssh client": The computer you are using; the one on which you type `ssh <username>@<ip.ad.dr.ess>`

In all examples below, a multirotor named "shredder" is connected to the lab network and its IP address is `192.168.1.237`.

Pre-requisites:
The server's IP address needs to be routable from your client machine.
E.g. If you are at home, and your IP address is `192.168.1.28`, and you want to `ssh` into shredder, the command `$ ssh shredder@192.168.1.237` will fail.
This is because your home 192.168.1/24 subnet is different from the lab's 192.168.1/24 subnet.
Your local machine needs to have the ssh-client, and the remote machine you will be accessing needs to have the ssh-server daemon running.
To install both the server and client (perfectly acceptable to have both on both machines), type:

```bash
sudo apt install ssh
```

Ubuntu automatically enables the ssh-server daemon upon installation.

For some history, and extended information beyond what is provided here, read the [Arch Wiki article], as well as lots of posts on lots of websites.
Just know that some sites are not peer reviewed by knowledgeable and experienced individuals, nor updated often (if ever).
The Arch Wiki generally has both of these traits.

<!-- Links -->
[Arch Wiki article]: https://wiki.archlinux.org/title/SSH_keys

## basic overview ##

`ssh` is actually really simple.
It essentially consists of logging into a computer with a known username and password.
For example, if I were connected to the lab network and typed

```bash
 ssh shredder@192.168.1.237
```

I would be connecting to the computer whose IP address is 192.168.1.237 and logging in as the user "shredder".
After providing shredder's password as requested on the command line, I would have a shredder terminal session on my computer.
I could run commands, and move, edit and delete files, just as if I were typing on a keyboard directly attached to shredder.

Try "ssh-ing" into your multirotor.
Fill in the correct user and IP address where indicated.
Type

```bash
ssh <username>@<your.multirotor.ip.address>
```

After you connect, it will ask you for your password--type it in, and see if you get in.
If you have problems, talk to someone else about getting access.
Otherwise, you are in your home folder on your vehicle!
I realize that's not terribly exciting, but it's a start.
To leave, simply hit `CTRL-D` on your keyboard, or type

```bash
$ exit
```

This places your terminal session back to controlling your local machine.

## a note on ports ##

ssh by default will use port 22, but the magicc server is set up on port 290, so we have to manually change it.
The "-p 290" command tells ssh to use port 290 on the server.

## Creating a Custom Hostname ##

What if I didn't want to type the full command,

```bash
$ ssh username@hostname
```

every time I wanted to log in?
Well, ssh allows you to make custom aliases (e.g. nicknames) for different hosts.
Here's how you could make a custom alias for shredder.
(This assumes that you already have access.)

First, navigate to your `~/.ssh/` directory.
If it doesn't exist, then make it.
Create a new "config file by typing

```bash
$ gedit ~/.ssh/config
```

and create config stanzas by adding the following lines

```bash
ServerAliveInterval 90
# IdentitiesOnly yes

Host shred
    Hostname 192.168.1.237
    User shredder
```

Now, if you type

```bash
$ ssh shred
```

`ssh` will replace "shred" with `shredder@192.168.1.237`.
It can save a lot of typing over time.

## Shared Keys ##

But what if I didn't want to type my password every time I log in?
For example, when you download a git repository from the magicc server, you would have to type your password every time you wanted to download an individual ros package.
(That would mean you'd have to type your password ~30 times if you downloaded the full Relative_Nav stack).
Luckily, `ssh` allows you to use a "shared key" between two computers by creating a private-public keypair.
This allows a client to access a host without typing the password every time, because the client is recognized as a trusted source.

Let's try this by creating a shared key between your computer and the magiccvs server.
Start by navigating to that same `~/.ssh/` folder on your local machine, and typing

```bash
$ ssh-keygen
```

It will first ask you where to save the generated key.
The default location is fine, so just press enter.

It will then ask you for a "passphrase" which you can leave empty and just press enter a few times.
Kudos if you assign a password to your private key.
(We believe in security but not that much. :P)
This will create a private key file, a public key file, and a fingerprint printed to screen as a box of random characters.

!!! Warning
    Safeguard your private key.
    If your machine, or your private key, get compromised, immediately decommision the public key that has been deployed to ssh servers, github, etc.
    Generate a new keypair after ensuring no persistent threat is present.
    Deploy your new public key to the services to which you need access.

You usually do not need to worry about the fingerprint.
If you care, there are some good superuser and stack exchange posts about it.

Now, we want to copy that key over to shredder.
The default settings that allow ssh login by username and password allow you to copy your public key to the remote machine using your password.
To do this, simply type

```bash
ssh-copy-id shred
```

That copies the public key you just generated over to the vehicle, using the mapping you created in the previous section.
You'll need to type your password for the magiccvs server to accept the new file.
But after that, you'll no longer need to type your password to log into the magicc server from your computer.
The `ssh` server will verify your identity using public-private key verification.
You should go ahead and test it out!

```bash
ssh shred
```

###

Certain lab resources do not use the default settings, opting to disable password authentication.
This is generally considered safer.
In this case, you need to copy the contents of your `~/.ssh/id_rsa.pub` public key that was created with the `ssh-keygen` command.
Paste that public key onto a new line in the `~/.ssh/authorized_keys` file on your `ssh` server.

In this case, you would connect "shredder" to a monitor and keyboard, and copy the public key via e.g. a usb drive.
Create the `authorized_keys` file if it does not already exist.

You will also need to alter the appropriate stanza in your `~/.ssh/config`.
First, uncomment `IdentitiesOnly yes` at the top of the file (as seen above in !#creating-a-custom-hostname).
Next, add `IdentityFile ~/.ssh/private_key` inside the appropriate stanza.

Your whole file should look something like this:

`~/.ssh/config`

```bash
ServerAliveInterval 90
IdentitiesOnly yes

Host shred
    Hostname 192.168.1.237
    User shredder
    IdentityFile ~/.ssh/your_private_key
```

## More on creating a key ##

There are several different types of keys that you can use.
The default is an `rsa` key.
You can look up its history if you are interested.
What you need to know is that a 2048-bit rsa key used to be default.
Now, modern computers can brute-force a 2048-bit rsa key in "reasonable time".
It is recommended to use **_at least_** a 3072-bit rsa key (this is the current default).
A 4096-bit key is currently considered more than sufficient.
The larger the key, the longer it will take to authenticate, but on modern computers, it is generally negligible.
An `ed25519` key type is considered about as strong as a 3072-bit rsa key.
I personally prefer these because the public key is much shorter than one generated with `rsa`, and hence is feasibly typed should the necessity arise.

You can create an ed25519 key with:

```bash
$ cd ~/.ssh
$ ssh-keygen -t ed25519 -C "remoteHost_usernameOnRemoteHost_localClientHostName_localUsername" -f remoteHost_usernameOnRemoteHost_localClientHostName_localUsername
```

You'll notice this creates two files in your `~/.ssh/`

```bash
$ cd ~/.ssh
$ ls

remoteHost_usernameOnRemoteHost_localClientHostName remoteHost_usernameOnRemoteHost_localClientHostName_localUsername.pub
```

Inside the pubkey, at the end, you will see the contents of the string you typed after the `-C`.
`-C` is for "comment".
You are free to type anything in this string.
It is simply a method to help humans keep track of the contents/uses of that keypair.
`-f` is the filename.
I generally try to make the comment and the filename match, though this is not a requirement.
If you paste the public key into your github `ssh` keys settings section, the comment is conveniently set to the title of that key.
This is an example of how a comment can help you later.
If you have multiple keys in your github account, and you need to decommission one of them, it will be easy to know, by its title, which needs to be removed.

I recommend creating a new key for each `ssh` server you wish to connect to.
Key management in this manner facilitates decommissioning a key because if a single key gets accidentally published somewhere, you can decommission on that server only (or service, e.g. github) and not have to worry about the other servers or services that you connect to with `ssh`.
If you only use one key for everything, if that private key becomes compromised, you have to decommission and re-deploy to every. single. host/site/service.
Thus, it is helpful to have meaningful comments and file names.

### Caveats ###

Note that you will have to use the nickname you set in the `Host` field if three conditions are met:

1. You use `IdentitiesOnly` and `IdentityFile` in your stanza,
1. You use a non-default filename (e.g. you set `-f some_non_default_key_name`) when generating the key, and
1. That custom filename is in the config stanza

`ssh` will not know it needs to use that specific, custom keyname when attempting to authorize a connection.
By using the nickname on the `Host` line that opens the stanza, you are informing `ssh` of the non-standard key name to use.

#### git ####

This complicates certain things, e.g. cloning submodules with `git submodule update --init --recursive`.
When you first clone a repo, you can use your nickname, if your stanza is configured for e.g. GitHub.
Let's say for example you have ssh-ed into shredder, and have created a keypair and pasted the public key into GitHub to pull down changes you make on your main dev machine.
Further, your GitHub username is `coolcoder`.
A stanza in `/home/shredder/.ssh/config` could look like this:

```bash
Host ghub.coolcoder
    HostName github.com
    User git
    IdentityFile ~/.ssh/github_coolcoder_shredder
```

Create the keypair with:

```bash
$ cd ~/.ssh
$ ssh-keygen -t ed25519 -C "github_coolcoder_shredder" -f github_coolcoder_shredder
```

Copy the contents of `github_coolcoder_shredder.pub` into your GitHub account.

To clone a repo and retrieve the submodule content, you would e.g.:

```bash
$ cd ~/repos
$ git clone ghub.coolcoder:coolcoder/superawesome_research.git
$ cd superawesome_research
$ git submodule update --init --recursive
```

However, the last command would fail because your `.gitmodules` file should look something like this:

```bash
[submodule "superawesome_utils"]
    path = superawesome_utils
    url = git@github.com:coolcoder/superawesome_utils.git
```

`ssh` would fail to authenticate you properly with GitHub because it did not use your GitHub keypair.

To resolve this, you should alter the `.git/config` file in your local repo directory.
There should be a line that looks like this:

```bash
[submodule "superawesome_utils"]
        active = true
        url = git@github.com:coolcoder/superawesome_utils.git
```

!!! Note
    This section will only be visible after calling `submodule update`.
    So the workflow is to run it once, expecting it to fail, so that it writes this section to the local `.git/config`.
    Then, you can open that file in your favorite editor and make this following change:

Change the url line to e.g. the following:

```bash
[submodule "superawesome_utils"]
        active = true
        url = ghub.coolcoder:coolcoder/superawesome_utils.git
```

and run a second time:

```bash
$ git submodule update --init --recursive
```

Now, ssh will know that when attempting to fetch the submodule content, it needs to verify identity with GitHub using your GitHub keypair, just as it does for your main repo.
You probably do not want to change the `.gitmodule` file to use your ssh nickname because:

1. If you clone it on a different machine you own, you may be using a different nickname, or keyfile name, or both
1. If someone else is working on your code with you, they will have their own authentication method, e.g. perhaps they (inferiorly) prefer username/password authentication.

So, leave the traditional clone url in the `.gitmodule` file in your repo.
