OpenCV with CUDA
================

Warning: Consider this guide as a list of suggestions and lessons learned rather than an exact pathway to get OpenCV installed. Your computer may be set up differently. You may have installed something that conflicts with one of the packages we reccommend installing. You may have a different use case. As such, consider these two tips:
1. **Read through this entire guide before performing any actions.**
2. **Think before you type.**

In order to use OpenCV with CUDA-acceleration, you must compile OpenCV from source and tell CMake to include it when building.

## Building OpenCV from Source ##

1. Install prerequisite packages required for cloning the latest OpenCV git repositories:

    ```bash
    $ sudo apt install git curl
    ```

1. Download the OpenCV version you would like to install from the **Releases** tab of the [OpenCV repo](https://github.com/opencv/opencv). The following installs the latest release of OpenCV from the github repo. To install an earlier version, replace
"$(curl --silent "https://api.github.com/repos/opencv/opencv/releases/latest" | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/')"
with the desired version tag (e.g. 3.4.6)

    ```bash
    $ cd ~/Downloads
    $ git clone https://github.com/opencv/opencv.git
    $ git clone https://github.com/opencv/opencv_contrib.git
    $ git clone https://github.com/opencv/opencv_extra.git
    $ cd opencv
    $ git checkout $(curl --silent "https://api.github.com/repos/opencv/opencv/releases/latest" | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/')
    $ cd ../opencv_contrib
    $ git checkout $(curl --silent "https://api.github.com/repos/opencv/opencv/releases/latest" | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/')
    $ cd ../opencv_extra
    $ git checkout $(curl --silent "https://api.github.com/repos/opencv/opencv/releases/latest" | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/')
    $ cd ..
    ```

1. Install the following packages as prerequisites for building OpenCV with CUDA support. Note that we recommend installing `libvtk6-*` because ROS Melodic has a dependency on VTK 6. If you do not use ROS (i.e. if you do not have it installed), you can replace `libvtk6-*` with `libvtk7-*` in the following:

    ```bash
    $ sudo apt install ccache cmake libopenblas-dev liblapacke-dev libjpeg-dev libtiff-dev libpng-dev libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libatlas-base-dev gfortran libhdf5-serial-dev libeigen3-dev libeigen-stl-containers-dev libavresample-dev zlib1g libatlas3-base libatlas-base-dev liblapacke-dev libvtk6-jni libvtk6.3 libvtk6-dev libvtk6-java libopenjp2-tools libprotobuf-dev libgtkglext1-dev libdc1394-22-dev libtbb-dev libceres-dev libcaffe-cuda-dev libleptonica-dev coinor-libclp-dev libtesseract-dev tesseract-ocr libogre-1.9-dev ogre-1.9-tools ocl-icd-opencl-dev ocl-icd-libopencl1 opencl-headers clinfo ocl-icd-dev hdf5-tools pybind11-dev python-dev python3-dev python-pip python3-pip
	```
You may have an error installing one or more of these packages. If `apt` returns an error, `apt` will not install any of the packages after the package that errored, so make sure that all packages in this list get installed.

	```
    $ pip2 install --upgrade --user pip setuptools wheel
    $ pip3 install --upgrade --user pip setuptools wheel
    $ reboot
    ```
You must reboot.

    ```bash
    $ pip3 install --user ipython numpy scipy pybind11 pygame vtk matplotlib pyqt5 pyside2 pytesseract tesserocr jupyter gnupg
    $ pip3 install --user pyopencl
    ```
If you need specific OpenCL features that are not provided by these packages, the following link may be informative (https://github.com/opencv/opencv/wiki/OpenCL-optimizations)

1. Due to header file sourcing location issues with OpenBLAS, create a symlink for the BLAS header file in the appropriate path:

    ```bash
    $ sudo ln -s /usr/include/x86_64-linux-gnu/cblas.h /usr/include/cblas.h
    ```
1. Similarly, create a symlink for the vtk binary

	```bash
	$ sudo update-alternatives --install /usr/bin/vtk vtk /usr/bin/vtk6 10
	```

Note: If you want to use the RealSense libraries with OpenCV, you need to first get the RealSense SDK from [Intel's RealSense GitHub](https://github.com/IntelRealSense/librealsense). For use with ROS, use [our wiki page](#!hw_guides/intel_rs_d400.md#Intel_RealSense_D400_Camera). Once the SDK is installed, add the following cmake flags between the `cmake` and the `..` at the end of the `cmake` command on the next step:
`-DWITH_LIBREALSENSE=On \`
`-DLIBREALSENSE_INCLUDE_DIR=<fill this in> \`
`-DLIBREALSENSE_LIBRARIES=<fill this in> \`
`-Drealsense2_DIR=<fill this in> \`

Note: If you are using a Python Virtual Environment (recommended), make sure you have activated the virtual environment before running `cmake`. Take a look at the [PyImageSearch Tutorial](https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/). This does not provide instructions for building with CUDA, but it is a good, additional resource. As mentioned in step 3 of the tutorial, first install the NumPy package into your virtual environment before running `cmake`, as it is a requirement for working with Python and OpenCV.

1. Configure CMake (see below for some additional options you may want, you may need to check the paths for some of the components):

    ```bash
    $ cd ~/Downloads/opencv
    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_INSTALL_PREFIX=/usr/local/opencv \
                -DCMAKE_BUILD_TYPE=RELEASE \
                -DWITH_CUDA=ON \
                -DENABLE_FAST_MATH=1 \
                -DCUDA_FAST_MATH=1 \
                -DWITH_CUBLAS=1 \
                -DINSTALL_PYTHON_EXAMPLES=OFF \
                -DENABLE_PRECOMPILED_HEADERS=OFF \
                -DWITH_OPENMP=ON \
                -DWITH_NVCUVID=ON \
                -DOPENCV_EXTRA_MODULES_PATH=~/Downloads/opencv_contrib/modules \
                -DBUILD_opencv_cudacodec=OFF \
                -DPYTHON_DEFAULT_EXECUTABLE=$(which python3) \
                -DBUILD_USE_SYMLINKS=ON \
                -DBUILD_PERF_TESTS=OFF \
                -DBUILD_TESTS=OFF \
                -DBUILD_JAVA=OFF \
                -DBUILD_PROTOBUF=ON \
                -DBUILD_opencv_java_bindings_gen=OFF \
                -DBUILD_opencv_cnn_3dobj=OFF \
                -DWITH_GDAL=ON \
                -DWITH_CLP=ON \
                -DTesseract_INCLUDE_DIR=/usr/include/tesseract \
                -DTesseract_LIBRARY=/usr/lib/x86_64-linux-gnu/libtesseract.so \
                -DOpenBLAS_LIB=/usr/lib/x86_64-linux-gnu/openblas/libblas.so \
                -DWITH_OPENGL=ON \
                -DWITH_VULKAN=ON \
                -DPYTHON3_INCLUDE_DIR2=~/.local/include/python3.6m \
                ..
    $ cmake -DOPENCV_PYTHON3_VERSION=ON ..
    ```

	In most cases you will also want to include the non-free OpenCV modules. These include algorithms that are free for personal and academic use, but require a license for commercial use, e.g. feature descriptors such as SURF and SIFT. Unless you have a specific reason not to (check the licenses, know your usage scenario), add them:
	```bash
	-DOPENCV_ENABLE_NONFREE=ON \
	```
    Additional cmake flags to include for development computers (i.e. NOT Jetson TX* Devices):
    ```bash
    -DWITH_TBB=ON \
    ```
    **Note:** the OPENCV_PYTHON3_VERSION flag will cause the build to fail if included in the initial cmake option list. It must be added during a second cmake configuration.

    **Note:** if you are on the **TX1** it is likely that you will run out of storage space unless you have `-DENABLED_PRECOMPILED_HEADERS=OFF` set. See [this](https://docs.opencv.org/3.2.0/d6/d15/tutorial_building_tegra_cuda.html) *OpenCV Building from Source for Tegra/CUDA* tutorial for more flags/help.

    Make sure to check the output of that command and make sure the NVIDIA CUDA modules are all set. This includes the **NVIDIA GPU arch**, AKA, *compute capability* (see below). Also, you can use `ccmake` to visually check all the available CMake flags:

    ```bash
    $ sudo apt install cmake-curses-gui
    $ ccmake .
    ```

You will probably have errors of the following form as you run `cmake`:
```
-- The imported target "____" references the file
   "_____"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "_____"
but not all the files it references.
```
You can reduce these errors by installing more packages or if the packages are installed, by tracking down the files and symlinking them to where `cmake` is looking for them. I was never able to get rid of all of them, specifically one concerning `vtkRendingPythonTkWidgets` and one concering `libopenjp` (OpenJPEG). I do not think either of these will actually cause issues. The core OpenCV libraries will still build.

**Note:** The "`vtkRendingPythonTkWidgets`" warning _may_ have something to do with installation of "`tk`", or "`python-tk`" or something like that from `apt`, or "`vtk`" or something similar from `pip`. If you find out the cause, please upstream the fix for the broader community's benefit and learning. Thanks!

1. Once everything is configured, go ahead and build and install:

    ```bash
    $ make -j8
    $ sudo make install # alternatively, consider using checkinstall
    $ sudo ldconfig # update your shared library cache so ld knows about your new lib
    ```

    The `-j8` flag tells CMake to use 8 threads. If you have an i7 processor, you probably have threading, so you can multiply the number of cores by 2. Otherwise, use `-j4` for a quadcore i5, for example. Also, if you are simultaneously running other cpu-intensive programs, you will likely want to lower the number to explicitly command how many threads to dedicate to CMake vs your other program(s).

    **Note:** The `make` process can take from 30-45 minutes, so go get a mug of meat. The build may spend a significant amount of time at "[ 98%] Built target opencv_stitching" (usually around 15-20 minutes). This is completely normal, just be patient.

1. Finally add the INSTALL_DIR to the system PATH in your `.bashrc` (or equivalent):
```bash
export PATH=/usr/local/opencv/bin:$PATH
```

1. and source it on the command line: `source .bashrc`

![BOOM BABY!](assets/mission_accomplished.gif)


## Compute Capability ##

See [this visual_mtt repo](https://gitlab.magiccvs.byu.edu/robust_tracking/visual_mtt2/issues/29) for a discussion on choosing the right compute capability for your platform. Failure to do so when building OpenCV can result in an error such as:

```bash
GPU API call (invalid device function)
```

The TX1 has a [compute capability](https://en.wikipedia.org/wiki/CUDA) of 5.3 and TX2 has 6.2, as shown on Wikipedia.

## Managing Multiple OpenCV Versions (outside of ROS) ##

When you have multiple versions of a given library, it is important to tell the linker which library to link to. Often, we use CMake to take care of finding libraries and building C/C++ code. In this case, we tell CMake where to find our new OpenCV lib. See [this GitHub repo](https://github.com/plusk01/tests/tree/master/opencv_multiple_versions) for a good (if I may say so myself) example. You can force CMake to know about your specific library by adding an environment variable before running cmake commands for your own code.

```bash
$ export OpenCV_DIR=/usr/local/share/OpenCV/
```

## More on OpenCV with Python virtual environments
Following the [PyImageSearch Tutorial](https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/), you need to provide your Python virtual environment with the correct `cv2.so` binary. Refer to step 5 of the linked tutorial for more information, but for my install, the `cv2.so` file did not end up in the same place as in the tutorial.

Here's the process I followed to create a symbolic link to the binary.

Find the `cv2.cpython-36m-x86_64-linux-gnu.so` binary. (This binary name may be different depending on the python version you have).
```bash
$ find /usr/ -name cv2
```
This will (hopefully) return two different paths. Mine were:

```
/usr/local/opencv/lib/python3.6/site-packages/cv2
/usr/local/opencv/lib/python2.7/dist-packages/cv2
```

Others have had `dist-packages` instead of `site-packages` or more than two paths here. You might need to poke around for a minute to figure out the path to `cv2.cpython-36m-x86_64-linux-gnu.so`.
Assuming your paths were the same and you have python 3.6 installed, the binary is then in

```
/usr/local/opencv/lib/python3.6/site-packages/cv2/python-3.6/
```

Run the following, making sure to modify it if your paths were different. Make sure to replace `<<<YOUR_VIRTUAL_ENV>>>` with the name of your virtual environment.

```bash
sudo ln -s /usr/local/opencv/lib/python3.6/site-packages/cv2/python-3.6/cv2.cpython-36m-x86_64-linux-gnu.so ~/.virtualenvs/<<<YOUR_VIRTUAL_ENV>>>/lib/python3.6/site-packages/cv2.so
```

## Using Your Newly Minted OpenCV with ROS ##

ROS Kinetic (currently) packages OpenCV 3.2. In order to use your CUDA-enabled OpenCV version, you have to tell CMake, in no uncertain terms, from where to link OpenCV. Similarly to above, you can either point the environment variable `OpenCV_DIR` to your OpenCV cmake config location (likely `/usr/local/share/OpenCV`) or you can simply pass it to `catkin_make` as follows:

```bash
catkin_make -DOpenCV_DIR=/usr/local/share/OpenCV
```

Note that once you point `catkin_make` to your OpenCV install once, you can simply run `catkin_make` without passing the `-DOpenCV_DIR` option again. This is because it is saved in the CMake cache. You will need to add the option again if you delete the `devel` and `build` directories, of course. Remember that you can always check if that variable is set by running (inside of the `catkin_ws` directory):

```bash
ccmake build
```

Note that if any other packages require OpenCV (which is most of the time), catkin will also pull in the wrong dependencies. The solution to this problem is to add the [vision_opencv](https://wiki.ros.org/vision_opencv) package that provides `cv_bridge` into your `catkin_ws/src` directory. Then, when you build your packages with `catkin_make -DOpenCV_DIR=/usr/local/share/OpenCV`, the `vision_opencv` packages will also point to your CUDA-enabled OpenCV. And now your ROS code (inside your current `catkin_ws`) is using CUDA-enabled OpenCV!

It can be helpful to be certain at runtime if you are linking to the correct OpenCV. This can be done with:

```cpp
// Grab the install path of OpenCV
  int s = cv::getBuildInformation().find("Install path:");
  int e = cv::getBuildInformation().find('\n', s);
  ROS_INFO("OpenCV %s", cv::getBuildInformation().substr(s, e-s).c_str());
#if OPENCV_CUDA
  ROS_INFO("Visual MTT CUDA enabled with %i device(s).", cv::cuda::getCudaEnabledDeviceCount());
#endif
```
Note that in this snippet we are also able to know at compile-time whether to use CUDA with OpenCV. For more details, see the [visual_mtt](https://gitlab.magiccvs.byu.edu/robust_tracking/visual_mtt2) repo.



## Understanding OpenCV / CUDA Development ##

## Common Errors / FAQs ##

1. Errors with `ccache -- invalid option -E`. The CUDA compiler (nvcc) and ccache do not play well together, add the `-DCUDA_HOST_COMPILER=/usr/bin/g++` flag to the cmake command, as discussed [here](https://stackoverflow.com/a/38532883/2392520).

## Future Work ##

Further work may be done in the future to add additional feature capabilities to the OpenCV build. Below are notes regarding attempted implementation and troubleshooting of various features.

While attempting to implement Python 3 functionality into the build, the manually-set option flags (found to be functional) and automatically-detected flags were recorded as follows:

```bash
"manual"
-DPYTHON3_EXECUTABLE=/usr/bin/python3m
-DPYTHON3_INCLUDE_DIR=/usr/include/python3.6m
-DPYTHON3_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
-DPYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages
```
```bash
"automatic"
-DPYTHON3_EXECUTABLE=/usr/bin/python3
-DPYTHON3_INCLUDE_DIR=/usr/include/python3.6m
-DPYTHON3_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
-DPYTHON3_PACKAGES_PATH=lib/python3.6/dist-packages
```

Typically the cmake compiler is able to successfully find the following directories. However, if cmake reports that it cannot be found, add the following option:
```bash
-DPYTHON3_NUMPY_INCLUDE_DIRS=~/.local/lib/python3.6/site-packages/numpy/core/include
```

While trying to allow compatibility with OGRE, it was found that standard installation and linking to the following library did not work:

```bash
"did not work"
-DOGRE_DIR=/usr/share/OGRE/cmake/modules
```

Other features that passed configuration errors during make include JASPER_LIBRARIES and JASPER_INCLUDE_DIR. The build also passed import errors for vtk and pvtk.

## Resources ##

* [Compiling OpenCV with CUDA Support](https://www.pyimagesearch.com/2016/07/11/compiling-opencv-with-cuda-support/)

* [ROS Kinetic and New OpenCV](https://answers.ros.org/question/242497/kinetic-enforces-opencv3-throughout-the-system/)

* [ROS Kinetic and OpenCV-CUDA](https://answers.ros.org/question/242376/having-trouble-using-cuda-enabled-opencv-with-kinetic/#)
