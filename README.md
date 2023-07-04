# TIGERs Mannheim On-Robot Vision
This repository contains code running on a Raspberry Pi 3A+ with a camera module v1 detecting mainly balls. Additionaly functionality was implemented for the RoboCup 2021 and 2022 technical challenges.

## How it works
The ball detection is based on color segmentation similar to the algorithm running on the shared ssl-vision software. It is configured for orange, white, and black. The orange blobs are estimated in 3D space based on their diameter or an intersection of the camera ray with the ground plane. Afterwards a tracking filter is used to estimate motion of each detected ball. The largest ball is reported via a UART to the microcontroller controlling the robot. Depending on its active bot skill the robot can autonomously drive to the ball.

A simple point distance sensor is implemented which estimates the distance to the next obstacle by checking for white and black vertical lines (goals and walls) in front of it. A larger vertical line corresponds to a closer obstacle.

# Development

## How to build

Build is based on cmake. The applications are cross-compiled on a standard x64 computer.

You need:
 - The sysroot of your target. Available as artifact of the tigers-buildroot project. It contains all the system libraries and files.
   - If you have a local tigers-buildroot setup the required subfolder is `tigers-buildroot/buildroot/output/host/arm-buildroot-linux-gnueabihf/sysroot`
   - Alternatively, you may download the sysroot from the tigers-buildroot CI [here](https://gitlab.tigers-mannheim.de/main/tigers-buildroot/-/pipelines). Download the rpi3-tigers-sysroot artifact.
 - A toolchain:
   - For Linux: [here](https://developer.arm.com/-/media/Files/downloads/gnu-a/10.3-2021.07/binrel/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabihf.tar.xz)
   - For Windows: [here](https://developer.arm.com/-/media/Files/downloads/gnu-a/10.3-2021.07/binrel/gcc-arm-10.3-2021.07-mingw-w64-i686-arm-none-linux-gnueabihf.tar.xz)
 - cmake

:point_up: The required compiler is GCC 10.3 for 32-Bit ARM with hardware floating point support. It is the same version tigers-buildroot is configured to use. If tigers-buildroot is updated, this one must be changed as well.

Then go into the repository root folder and run:

1. Linux build
   ```
   cmake -B build -DCMAKE_SYSROOT=/your/path/to/sysroot -DCMAKE_PREFIX_PATH=/your/path/to/toolchain .
   cmake --build build -j
   ```
1. Windows build (assuming MinGW environment)
   ```
   cmake -B build -G "MinGW Makefiles" -DCMAKE_SYSROOT=/your/path/to/sysroot -DCMAKE_PREFIX_PATH=/your/path/to/toolchain .
   cmake --build build -j
   ```

You may need to adjust your sysroot and toolchain path.


## Eclipse IDE Setup

1. Download Eclipse Embedded for your platform: [2022-06 Embedded](https://www.eclipse.org/downloads/packages/release/2022-06/r/eclipse-ide-embedded-cc-developers)
1. Unpack the file and start Eclipse
    1. Select a workspace. It should not contain any whitespaces!
1. Go to Help => Eclipse Marketplace. Install the following plugins:
    1. cmake4eclipse
    1. CMake Editor (optional, syntax highlighting for cmake files)
    1. DevStyle (optional, for a nice dark theme)
1. Restart Eclipse after plugin installation
1. In Eclipse open: Window => Preferences
    1. Go to: Run/Debug => String Substitution
    1. Click: New..., Name: TIGERS_ROBOTPI_TOOLCHAIN, Value: path to your cross compilation toolchain
    1. Click: New..., Name: TIGERS_ROBOTPI_ROOTFS, Value: path to your target's sysroot
    1. (Windows only) Go to: C/C++ => Cmake4eclipse => General tab
    1. Change default build system to: MinGW Makefiles

### Project Import

1. In Eclipse select: File => Import...
1. Under "GIT" select "Projects from Git", hit Next
1. Select "Clone URI", hit Next
1. Enter the RobotPi repository URL
1. Select at least the master branch, others are optional, click Next
1. Select a local directoy for the project, click Next
1. Wait for the download to finish and then select "Import existing projects", click Next
1. Select the RobotPi project and click "Finish"

### Compiling

1. Right click on the RobotPi project and go to: Build Configurations => Set Active
1. There are different configurations. One for a debug build and one for a release with optimizations enabled.
1. Choose the configuration you wish to build
1. Right click on the project and select Build Project
1. You can also select the build configuration and the build command in the toolbar. It is the small hammer symbol and the symbol right of it. Make sure you select the RobotPi project before using the buttons.

### Remote Development Setup

In order to run and debug RobotPi on the actual target hardware you need to setup an SSH connection. Go to Window => Preferences.
 - Navigate to: Remote Development => Remote Connections
 - At the top select Remote Services: SSH
 - Click Add
   - Connection name: RPi Wifi
   - Host: 192.168.42.1
   - User: root
   - Select "Password based authentication"
   - Password: root
 - Click finish and close preferences

### How to run

In Eclipse:
 - Go to Run => Run Configurations...
 - Under C/C++ Remote Application
 - Select "RobotPi Release"
 - Check that the Connection is set to "RPi Wifi"
 - Hit Run

### How to debug

You may need to install the following packages if gdb fails to start: `sudo apt install libpython2.7 libncursesw5`
Or on Windows (use 32bit installer!): `https://www.python.org/downloads/release/python-2716/`

In Eclipse:
 - Go to Run => Debug Configurations...
 - Under C/C++ Remote Application
 - Select "RobotPi Debug"
 - Check that the Connection is set to "RPi Wifi"
 - Hit Debug
 - If everything works will ask you if you want to switch to the Debug View, say yes
 - Eclipse will stop the program at main(), happy debugging!
