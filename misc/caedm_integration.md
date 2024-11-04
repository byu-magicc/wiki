# CAEDM Integration #

**Mount your CAEDM J:\ drive in Linux**

```bash
mkdir ~/jdrive
sudo apt install cifs-utils
sudo mount -t cifs //fs-caedm.et.byu.edu/<caedm_username> ~/jdrive -o user=<caedm_username>,rw,uid=$USER,gid=$USER
```

**Mount CAEDM group directory in Linux**

```bash
mkdir ~/my_group
# Replace caedm_username below
sudo mount -t cifs //fs-caedm.et.byu.edu/<caedm_username>/groups/my_group ~/my_group -o user=<caedm_username>,rw,uid=$USER,gid=$USER
 ```
 **Connect to CAEDM VPN from off campus**

 Use the guide provided by CAEDM at this [link](https://caedm.et.byu.edu/wiki/index.php/VPN_Instructions_for_Linux).
