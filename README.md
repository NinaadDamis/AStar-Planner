## A Star Planner Implementation

This repository implements a A star planner for the Circle World Environment. The planner takes in a Circular Robot and a Rectangular Map with circular obstacles as input. It represents the map as a 8-connected grid and outputs the shortest path as a series of (x,y) coordinates. 

This code was developed and tested on Ubuntu 22.04 using C++11.

To compile and run the code : 

```
g++ -std=c++11 planner.cpp -o planner
```
```
./planner 
```