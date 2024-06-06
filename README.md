mjbots hoverbot
===============

Source and design files for the mjbots hoverbot, its controlling
interfaces, and utilities for developing and operating it.

* GitHub https://github.com/mjbots/hoverbot
* Most files are free hardware and software: Apache 2.0 License
* F360 assembly: https://a360.co/4bEdIkS
* [BOM](BOM.md)

Directory structure
-------------------

* **base/** - C++ source files common to many applications.
* **mech/** - C++ source files specific to walking robots.
* **utils/** - Utilities for development and data analysis.
* **configs/** - Configuration files for different robots and applications.
* **hw/** - Hardware design files
* **docs/** - Documentation.


First Time Setup
----------------

The following should work on Ubuntu 22.04

```
./install-packages
```

Building for host
-----------------

```
tools/bazel test //...
```

Building for the target
-----------------------

```
tools/bazel test --config=pi //mech:hoverbot_deploy.tar
```
