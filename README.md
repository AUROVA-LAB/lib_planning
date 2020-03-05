# lib_planning


## Dependencies

This package requires of the following system libraries and packages

* [cmake](https://www.cmake.org "CMake's Homepage"), a cross-platform build system.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), C++ template library for linear algebra

## Compilation and installation

Installation:

``` git clone https://github.com/AUROVA-LAB/lib_planning ``` 

``` cd lib_planning/ ```

``` mkdir build```

``` cd build```

``` cmake .. ```

```make```

```sudo make install```

## How to use it

To use this library in an other library or application, it is necessary add in the CMakeLists.txt file:

``` find_package(planning REQUIRED) ```

And Eigen's dependence is necessary:

``` find_package (Eigen3 3.3 REQUIRED NO_MODULE) include_directories(${Eigen3_INCLUDE_DIRS}) ```

And link the libraries to the program

``` TARGET_LINK_LIBRARIES(<executable name> Eigen3::Eigen planning) ```

