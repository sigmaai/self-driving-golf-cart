Running the Simulation
======================

===============================
Dependencies & Pre-installation
===============================

First of all, you need to install the Carla simulator. Carla runs on Unreal Engine, a cross-platform high quality game engine.


Install the build tools and dependencies::

$ sudo apt-get install build-essential clang-3.9 git cmake ninja-build python3-requests python-dev tzdata sed curl wget unzip autoconf libtool

To avoid compatibility issues between Unreal Engine and the CARLA dependencies, the best configuration is to compile everything with the same compiler version and C++ runtime library. We use clang 3.9 and LLVM's libc++. You may need to change your default clang version to compile Unreal::

$ sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-3.9/bin/clang++ 100
$ sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-3.9/bin/clang 100

===================
Build Unreal Engine
===================

Please note that Unreal Engine repositories are private. In order to gain access you need to **add your GitHub username when you sign up at www.unrealengine.com.**

Download and compile Unreal Engine 4.18. Here we will assume you install it at "~/UnrealEngine_4.18", but you can install it anywhere, just replace the path where necessary::

$ git clone --depth=1 -b 4.18 https://github.com/EpicGames/UnrealEngine.git ~/UnrealEngine_4.18
$ cd ~/UnrealEngine_4.18
$ ./Setup.sh && ./GenerateProjectFiles.sh && make


Check Unreal's documentation "Building On Linux" if any of the steps above fail.

================
Installing Carla
================

