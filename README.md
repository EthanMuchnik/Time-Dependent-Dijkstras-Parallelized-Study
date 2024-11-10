# Time Dependent Dijikstra Parallelized Study

## Goal 

To model traffic using time dependent graphs/ dynamic networks and solve the time-dependent short dependent
problem for multiple vehicles in parallel.

## Approach 1

Simple annealing with standard dijkstra's adaptation for time dependent graphs:
    - for each car in swarm in parallel:
        compute shortest time dependent graph with current graph
        after each iteration:
            aggregate graph and repeat


## Approach 2

dedicate a core entirely for managing the graph and keeping it as updayed as possible, while
distribiting it to other worker cores. This reduces the overhead for synchronization , but if not carefully
executed can lead to conflicts in the graph (i.e. we'll probably need to enforce a lock on the graph it or design
it as a lock-free data structure)

## Approach 3


## The challenge
    
While parallel algorithms are well studied for static graphs, there is far less literature concerning dynamic graphs.
This is in part due to the inherent challenge of implementing a parallel solution in a problem where dependencies span
not only space/ branches of execution but also time. As such, we expect that our main challenges with achieving good
parallel performance will be with synchronization. It is relatively simple to achieve good speedups but with the complexity
of the problem at hand it is unlikely that trivial solutions would perform well.


## Background

## Goals and Deliverables

## Platform choice

  GHC for OpenMP, CUDA? MPI? 
## Schedule

    - Sequential implementation of time dependent dijkstras:
    - OpenMP implementation of parallel time dependend dijkstras for one vehicle
    - OpenMP implementation of parallel TDD for multiple vehicles
