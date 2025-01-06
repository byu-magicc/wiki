Installing CUDA
===============

## Background ##

The CUDA toolkit is Nvidia's GPGPU API and parallel computing platform. If you have an Nvidia graphics card, and you want to create and/or use a deep neural network ("ai"), or you want to quickly process imagery (OpenCV), you'll want to use libraries that can incorporate the CUDA API.

## Requirements ##

Depending on which version of CUDA you want to use and which architecture your GPU was built on, you need to first have a compatible Nvidia driver installed. As of the time of this writing, CUDA 10.1 requires the >=418.39. The table on the [docs.nvidia.com](https://docs.nvidia.com/deploy/cuda-compatibility/) website shows the Pascal architecture (on which the GTX 1050 Ti is built) can be paired with various driver versions. Make sure you verify your system architecture and use the correct table at the above link to ensure a compatible and functional setup.

## Installation ##

We assume you are using the Ubuntu OS on an X86 processor for this guide. JetPack will install the CUDA toolkit on the TX2. If you have a different machine architecture with a CUDA-enabled GPU, well, good luck. Google is your friend. Just do me a favor, and don't install CUDA 9.x at this point if you can avoid it. Do not use Python 2.x, either. It has been long enough; convert all your scripts to Python 3.x. Ok, I'm off my soapbox now.

### Driver ###

- Add the [`graphics-drivers-ppa`](https://launchpad.net/~graphics-drivers/+archive/ubuntu/ppa) to your system:
```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
```

### CUDA ###

- Download the CUDA `*.deb` file from the [nvidia.com website](https://developer.nvidia.com/cuda-downloads)
    - Select the Operating System, Architecture, Distribution, and Version
- On Ubuntu 18.04 and above, you can `cd` to the location of the `*.deb` and install with:
```bash
sudo apt install ./<file_name>.deb
```
or by using `dpkg`
```bash
sudo dpkg -i <file_name>.deb
sudo apt install -f
```
- Your terminal output will give you the appropriate command to add the public CUDA GPG key with a command similar to the following:
```bash
sudo apt-key add /var/cuda-repo-10-1-local-10.1.168-418.67/7fa2af80.pub
```
You need to copy the specific one from your terminal output and run it.

- Update your local repos: 
```bash
sudo apt update
```
- Finish installing CUDA: 
```bash
sudo apt install cuda
```
- Reboot your machine

The installation will create two directories in `/usr/local`, one is `cuda` and another is `cuda-<version>`. The `cuda` directory is a symbolic link to the `cuda-<version>` directory.

- Add `/usr/local/cuda/bin` to your PATH environment variable by adding the following line to your `.bashrc` or equivalent:
```bash
export PATH=/usr/local/cuda/bin:$PATH
```
- Run `source ~/.bashrc`, open a new terminal window, or reboot
- Congratulations, you now have CUDA installed

For the happy couple.

![for the happy couple](assets/mazal_tov.gif)

## Up-Keep

When future driver versions/updates are released by the ppa, just double check that the driver being used is correct after a reboot. According to [this Nvidia page](https://docs.nvidia.com/deploy/cuda-compatibility/index.html), you should be able to select a newer driver number without having to update CUDA.

To update CUDA, download the newest `*.deb` file, install, and double check the `/usr/local/cuda` directory correctly sym-links to the new `/usr/local/cuda-<version>` directory. If it does not, update the sym-link. From there, the line you added to your `~/.bashrc` should not need editing. Just perform a reboot. You may also need to run `sudo ldconfig`, but this is untested at this time.

#
