CUDA and GPGPUs
===============

CUDA (initially released in 2007) is an API built by NVIDIA for parallel computing using Graphics Processor Units (GPUs). Because it allows general-purpose use of the GPU, it is commonly referred to as GPGPU programming. CUDA can be used to increase the speed and efficiency of routines in OpenCV and TensorFlow (and other deep learning libraries), as well as custom CUDA-specific code. To use CUDA you must have an CUDA-enabled NVIDIA graphics card (e.g., the [GeForce GTX 1050 Ti](https://www.nvidia.com/en-us/geforce/products/10series/geforce-gtx-1050/#specs)). An alternative is to use [OpenCL](https://wiki.tiker.net/CudaVsOpenCL), a less-supported open source unified API for a variety of different hardware vendors. Basically, it's not as good as NVIDIA hardware/CUDA and in a research setting we should probably just focus on CUDA and creating cool things with it.

## Software ##

There are a number of software packages that can be found in the [Tools & Ecosystem](https://developer.nvidia.com/tools-ecosystem) section of the NVIDIA Developer CUDA ZONE. It is important that you choose the correct CUDA Toolkit version for the specific NVIDIA graphics driver of your GPU. More information can be found from [this SO answer](https://stackoverflow.com/questions/30820513/what-is-version-of-cuda-for-nvidia-304-125/30820690#30820690). At the time of this writing, CUDA Toolkit 8.0 was the most current which supports minimum of driver version 367.4x. My desktop machine is running driver version 375.39 for a GeForce GTX 1050 Ti.

#### CUDA Toolkit ####

The CUDA Toolkit provides a comprehensive environment (API/SDK) for C/C++ developers building GPU-accelerated applications. This includes CUDA-specific compilers needed by GPU-accelerated libraries (cuDNN, OpenCV, etc) or your own CUDA-enabled code.

#### CUDA-Enabled Libraries ####

###### **cuDNN** ######

This is a required CUDA-enabled library for GPU-accelerated TensorFlow and other deep neural network libraries.

###### **OpenCV** ######

The go-to open source computer vision library does have built-in CUDA-enabled code. For CUDA-enabled installation instructions, see [here](#!sw_guides/opencv.md).

###### **FFmpeg** ######

If you do anything that uses ffmpeg for video editing / splicing / management, [this](https://developer.nvidia.com/ffmpeg) would be useful for you.


## Installation Instructions ##

Download the appropriate runfile installer from [developer.nvidia.com](https://developer.nvidia.com/cuda-downloads).

During the setup of the installation, you will be asked about installing a graphics driver that may be outdated (i.e., your machine already has a newer version). The [correct response](https://devtalk.nvidia.com/default/topic/967017/cuda-toolkit-installing-outdated-graphics-driver/) is to decline this driver installation:

```bash
Install NVIDIA Accelerated Graphics Driver for Linux-x86_64 375.26? n
```

If you do this, you can ignore the `WARNING` about CUDA driver missing at the end of the installation summary.

Put the following in your `~/.bashrc` for post-installation and environment setup:

```bash
# CUDA Toolkit
export PATH="/usr/local/cuda-8.0/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-8.0/lib64:$LD_LIBRARY_PATH"
```

#### CUDA-accelerated samples ####

To see how cool CUDA has made your machine, install build and play with some of the NVIDIA CUDA samples found in either `/usr/local/cuda/samples` or `~/NVIDIA_CUDA-8.0_Samples`:

```bash
$ cuda-install-samples-8.0.sh
$ cd ~/NVIDIA_CUDA-8.0_Samples
$ make
```

Building the sample files can take about 15-20 minutes.
So go get the mormon equivalent of a coffee.

If you get an error about `-lnvcuvid` not being found, it is because your graphics driver is not exactly what the samples are looking for. A [lazy fix](https://askubuntu.com/a/890601) would be to

```bash
$ cd ~/NVIDIA_CUDA-8.0_Samples
$ find . -type f -execdir sed -i 's/UBUNTU_PKG_NAME = "nvidia-367"/UBUNTU_PKG_NAME = "nvidia-375"/g' '{}' \;
```

Some interesting samples are:

```bash
./smokeParticles #(60 fps)
./particles #(60 fps)
```

Note that this works if your driver is *375.xx* (i.e., we are replacing *367* with *375*).


#### Uninstall ####

To uninstall the CUDA Toolkit, run the uninstall script in `/usr/local/cuda-8.0/bin`


## Resources ##

* [CUDA Toolkit Documentation](https://docs.nvidia.com/cuda/index.html)

* CUDA_Installation_Guide_Linux.pdf found in `/usr/local/cuda/doc/pdf`: Good for post-installation information / environment setup.

* Udacity [Intro to Parallel Programming](https://www.udacity.com/course/intro-to-parallel-programming--cs344)
