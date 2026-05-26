# Adaptive HGA for VRP/TSP-D

This repository contains the source code for a C++17 implementation of an Adaptive Hybrid Genetic Algorithm (AHGA) for the Traveling Salesman Problem with Drone (TSP-D)  under the **min-time objective**. The algorithm builds truck-drone delivery plans from benchmark instances, improves them with adaptive local search operators, and can generate experiment results.

## Reference Paper

This implementation is inspired by the following Adaptive Hybrid Genetic Algorithm (AHGA) framework for the Traveling Salesman Problem with Drone (TSP-D):

> [A Hybrid Genetic Algorithm for the Traveling Salesman Problem with Drone]
> Authors: [Q. M. Ha, Y. Deville, Q. D. Pham, M. H. H`a]
> Year: [2018]

## Repository Structure

```text
.
├── benchmarks/              # TSP-D benchmark instances
├── src/                     # AHGA source code
├── experiments/             # Experiment logging and summary source files
├── README.md
└── .gitignore
```

## Benchmark Instances

Benchmark instance files are NOT included in this repository.nThe benchmark dataset used in this project is the Ha et al. (2018) TSP-D dataset referenced in the repository linked below.

Repo link: https://github.com/kaist-comet/TSPDroneLIB/tree/main

To reproduce experiments, download the benchmark `.txt` files and place them inside:

```text
benchmarks/
```

## Requirements

- A C++17 compatible compiler, such as `g++` or `clang++`
- macOS, Linux, or another Unix-like shell environment

No external C++ libraries are required.

## Build Commands

The project should be compiled with the `-O3` optimization level.

Build the main AHGA executable from the project root:

```bash
g++ -std=c++17 -O3 src/*.cpp -o ahga_tspd
```

Build the experiment summary executable:

```bash
g++ -std=c++17 -O3 experiments/benchmark_statistics.cpp -o benchmark_statistics
```

## Run

Run the algorithm with the default benchmark:

```bash
./ahga_tspd
```

Run a specific benchmark:

```bash
./ahga_tspd benchmarks/mbB102.txt
```

Run with a fixed random seed:

```bash
./ahga_tspd benchmarks/mbB102.txt 1
```

Each run prints the best feasible solution, route information, runtime, feasibility status, and penalty statistics. It also creates or appends to this local result file:

```text
experiments/results/results.csv
```

## Experiment Summary

After collecting local runs, create a summary file:

```bash
./benchmark_statistics
```

The summary is written locally to:

```text
experiments/results/summary_result.csv
```

## Benchmark Format

Benchmark files are stored in `benchmarks/` and generally include:

- customer count
- truck and drone speeds
- drone endurance
- launch and retrieve times
- edge weight type
- node coordinates
- optional drone eligibility flags

The depot is represented as node `0`.

## Notes

- Main algorithm parameters such as `MU`, `LAMBDA`, `ITER_NI`, `ITER_DIV`, and `OMEGA` are defined in `src/ahga.hpp`.
- The current objective focuses on minimizing total completion time.
- Results are stochastic unless a seed is provided.
