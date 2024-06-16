# GTSAM Use Cases

This repo is a collection of GTSAM tutorials. Checkout GTSAM tutorial slides for more explanations and infomation.

Prerequisites
------

- CMake >= 3.0 (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/) >= 1.50 (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [GTSAM](https://bitbucket.org/gtborg/gtsam) >= 4.0 alpha.

Compilation & Installation
------

In the parent folder excute:

```
$ mkdir build
$ cd build
$ cmake ..
```
Build all examples, execute:

```
$ make
```

Build specific example, execute:
```
$ make <example_name>
```
