# Example on how to use DepthAI V3 in C++ using vcpkg
This example shows how to use DepthAI V3 in C++ using vcpkg as a package manager.


## How to compile
```
cd {path to this repository}
git submodule update --recursive --init
cmake --preset=default
cmake --build build
```

## How to run
```
./build/example
```


> **Note:** We will support non-vcpkg usage soon and we'll be opening a PR to get DepthAI included as an official vcpkg package.

