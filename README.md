# Anytime Multi-Heuristic A* (A-MHA\*)

An implementation of the Anytime Multi-Heuristic A* algorithm introduced in the [paper](https://drive.google.com/file/d/1Ddgmdt5whgEDyybShNgNUQeBTq13ZrOc/view?usp=sharing).

## Building

This is a fully templated C++ `INTERFACE` implementation of the A-MHA* search and does not build standalone. It has to be built by linking against a dependent library or executable. The current implementation has a test module to check the build:

```
mkdir build
cmake -DCMAKE_BUILD_TYPE=Release ../
make
```

## Testing 

A test utility is included to check/evaluate the working of A-MHA\*. It is tested on a 15-sliding puzzle. To check:

```
mkdir build
./sliding_puzzle_test
```


