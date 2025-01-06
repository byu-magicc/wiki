# Syncing Time #

## motivation ##

If you have more than one computer collecting data for your thesis or dissertation, you likely want that data correlated in some manner.
If the time stamps for the data from machine a do not match those from machine b, you will likely not be able to combine the data.
Don't rely on software up your stack to sync time.
The hardware clock is the base.
Use a proper mechanism to sync the hardware clocks of multiple machines.
Use `chrony`.

Let's define some terms:

```sh
# e.g. a base station, or, *the time server*
amy="machine a"
# e.g. a rover, or, *the time client*
bob="machine b"

amy.ip="192.168.0.2"
bob.ip="192.168.0.3"
```

Note!<br>
  - use a public NTP server that is geographically close to you---you will get better results
  - If you have a gps antenna and module connected to your computer, you can use chrony to sync to GPS stratum 0
  - you can check stratum of a particular server at https://servertest.online/ntp

## Prerequisites ##

First install chrony on both `$amy` and `$bob`, then open `/etc/chrony/chrony.conf` for editing:

```console
$ sudo apt install chrony
```

Configure your router(s) to always give the same IP address to each of `$amy` and `$bob`.
This is usually in the "advanced" settings of your router.
E.g. it will pair the MAC address of `$amy` to an IP address you specify, usually in a table of some sort.

Note!<br>
  `$amy`'s wireless card and ethernet port have unique MAC addresses.
  If you plan to use one, or the other, or switch back and forth, make sure you configure a static IP for both.
  You will need to extrapolate the `chrony.conf` server section on the client to allow it to search for both server IPs dynamically.<br>
  Similarly, if `$bob` has multiple methods of connecting, e.g. wireless and wired,
  you will need to configure `$amy`'s `chrony.conf` to allow both client IPs to sync.

### Terminology ###

- stratum: how many "hops" your server is away from an atomic clock
  - examples of "stratum 0": the time source, e.g. GPS time (attoseconds), atomic clock (attoseconds), LTE time (nanoseconds)
  - examples of "stratum 1": the server that connects to the stratum 0 hardware --- time.google.com, some of time.apple.com
- leap smearing: leap seconds are smeared, rather than discontinuous jump
  - It is not recommended to mix ntp upstream servers in your config that use smearing
    - that means the members of the following set should be used exclusive of the others:
      - {the google pool | the facebook pool | a pool of servers that do not smear time }

## Settings for both server and client ##

Both machines need to be configured to attempt to connect to an internet source when available.
The `server` directive near the top of `chrony.conf` specifies "upstream" hosts that you want to request time from.
A reasonable base is Google's public NTP pool, which are advertised as "stratum 0."

```sh
server time1.google.com iburst trust prefer
server time2.google.com iburst trust prefer
server time3.google.com iburst trust prefer
server time4.google.com iburst trust prefer
```

You may want to modify/enable a few other directives on both server(s) and client(s).
Read the [man page] and understand any and all directives before changing the defaults.

```conf
maxupdateskew 50

makestep 1.0 3

hwtimestamp *

rtconutc

rtcsync
```

## Setting up the Server ##

Configure the ip addresses that are allowed to talk to the server.
Edit `/etc/chrony/chrony.conf`.
About half-way down the file, replace lines that start with "allow":

```bash
allow 169.254/16
allow 192.168.1
allow ${subnet.of.bobs.ip}
allow ${bob.ip}

manual

local stratum 3 orphan
```

In this example, the first two lines are the subnets---CIDR notation is allowed.
This allows any computer on those subnets to talk to the server.
You can also set it to allow a specific IP address.

The `manual` directive tells chrony that we want to be able to manually update the time.
This is usually only necessary if the time servers in your network do not have a button-cell battery keeping their clocks alive when disconnected from power, e.g. odroid, raspberry pi, etc.
That means when you power on the machines, and none of them can access the internet, they will all have a system clock starting at linux epoch time, or January 1, 1970.
Chrony has a mechanism to store the last known sync time to disk, but it will be incorrect by as many days and hours lapsed un-powered, and powered with no internet connection.
The take-away is if you will ever use `$amy` and `$bob` away from an internet connection, and if they both do not have a button-cell battery, you may decide that you want this directive.
If `$bob` and `$amy` both sync to your laptop `$chad` in the field, the laptop will retain the correct data and hour away from the internet for quite a long time.
And in the field, the only thing that matters is that your data is time-synced to each other.
It won't matter that 3:03pm in the data was actually 3:02pm according to time.nist.gov.

I do not include the `manual` directive, as I only use laptops as servers, and it simplifies a lot of setup when you get to the field for a flight test.

Finally, the `orphan` directive is required if you have multiple machines, e.g. `$alice` and `$chad` serving time away from the internet.
Pretty much, you may as well have it.
You can (and should) read more specifics about it in the [man page].

Then restart chrony

```console
$ sudo systemctl restart chrony.service
```

## Setting up the Client ##

Then edit `/etc/chrony/chrony.conf` and replace the 4 server lines with the following (or add to them):

```bash
server ${amy.ip} iburst trust xleave minpoll 1 maxpoll 2
server ${chad.ip} iburst trust xleave minpoll 1 maxpoll 2
```

Finally, restart chrony

```console
$ sudo systemctl restart chrony.service
```

## Checking if Time is Synchronized ##

To check if time is synced up, just use the following command on the client, e.g. `${bob}`
(all of these commands will work on `${amy}` and `${chad}`, if you want to verify what they are registering)

```console
$ sudo chronyc
```
this puts you on the `chronyc` (chrony client) command line

Note! <br>
Chrony "client" is the user-facing chrony program used to send commands to `chronyd`, the daemon.
It doesn't have anything to do with the "ntp client", e.g. `${amy}`.
`chronyd`, the chrony daemon, is what is running in the background, and is managed by `systemd`.

```console
chronyc> tracking
```

The "reference ID" field should be the IP of the server.
The calculated time offset is also calculated and shown with this command.

Chrony works by speeding up or slowing down the clock to sync it with the server, so it might not be all the way up to date when you check it.
To make it jump all at once, type

```console
chronyc> makestep
```

which should automatically sync the clock.

Some other commands you may want:

```console
chronyc> help
chronyc> sources -v
chronyc> sourcestats -v
chronyc> activity
chronyc> ntpdata
chronyc> refresh
chronyc> clients
chronyc> serverstats
chronyc> rtcdata
chronyc> trimrtc
```

<!-- links -->
[man page]: https://chrony.tuxfamily.org/doc/devel/chrony.conf.html
