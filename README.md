# TIGERs Mannheim On-Robot Vision
This repository contains the program running on a Raspberry Pi 3A+ with a camera module v1 detecting balls and goals. This code was used in the RoboCup 2021 technical challenge.

## How to build
Build is based on cmake. The final software runs on a standard Raspberry Pi OS.

The application is cross-compiled on a standard x64 computer, not on the Pi itself.

You need:
 - The rootfs of your Raspberry Pi OS, containing all the system libraries and files.
 - A toolchain:
   - For Linux: https://github.com/Pro/raspi-toolchain
   - For Windows: https://gnutoolchains.com/raspberry/
 - cmake

To build go into the repository root folder and run:
```
cmake -DROOTFS=/opt/rootfs -DTOOLCHAIN=/opt/cross-pi-gcc -DCMAKE_TOOLCHAIN_FILE=CrossCompile.cmake .
make -j
```

You may need to adjust your rootfs and toolchain path.
Afterwards, copy the compiled binary to your Pi.

## How to run
Connect a camera module v1 to your Pi and run RobotPi. You will see a debug output on the HDMI output with detected balls and goals.

## How it works
The ball detection is based on color segmentation similar to the algorithm running on the shared ssl-vision software. It is configured only for orange. The orange blobs are estimated in 3D space based on their diameter or an intersection of the camera ray with the ground plane. Afterwards a tracking filter is used to estimate motion of each detected ball. The largest ball is reported via a UART to the microcontroller controlling the robot. Depending on its active bot skill the robot can autonomously drive to the ball.

The goal detection is just searching for continuous white areas in front of the robot, no 3D estimation is done. The angle to the goal and its size is reported to the microcontroller. Together with the ball detection the robot can approach a ball, dribble with it, and shoot it on the goal.
