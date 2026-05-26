# Source Code Guide

The `src` directory contains the AHGA implementation used to solve TSP-D benchmark instances. These files are the main source code that should be uploaded to GitHub.

## Main Files

```text
main.cpp                 # Program entry point and console reporting
ahga.hpp                 # Shared data structures, constants, and function declarations
runner.cpp               # Main AHGA execution loop
benchmark_loader.cpp     # Benchmark loading and path resolution
distance.cpp             # Distance matrix and edge metric utilities
drone.cpp                # Truck-drone solution construction and drone feasibility helpers
fitness.cpp              # Objective, feasibility, penalty, and completion-time evaluation
population.cpp           # Population initialization and survivor selection helpers
genetic_algorithm.cpp    # Individual creation, crossover, education, and selection logic
local_search.cpp         # Neighborhood operators for truck and drone moves
adaptive_operator.cpp    # Adaptive local-search operator scoring and selection
utils.cpp                # Random, string, and small route utility functions
```

## Build

Compile the executable from the project root with `-O3` optimization:

```bash
g++ -std=c++17 -O3 src/*.cpp -o ahga_tspd
```

The generated `ahga_tspd` executable is a local build artifact and should not be committed.

## Execution Flow

1. `main.cpp` reads the benchmark path and optional seed from command-line arguments.
2. `runner.cpp` loads the benchmark and creates the distance matrix.
3. Initial routes are generated with k-cheapest insertion.
4. Individuals are split into feasible and infeasible populations.
5. New offspring are generated with crossover and improved through adaptive local search.
6. Penalty coefficients are adjusted during the search.
7. The best feasible solution and run statistics are returned to `main.cpp`.
8. The run result is written locally through `experiments/benchmark_logger.cpp`.

## Algorithm Components

- Giant-tour representation for chromosomes.
- Split procedure that converts a route into a truck-drone solution.
- Feasible and infeasible subpopulations.
- Biased fitness using solution cost and diversity.
- Adaptive operator scores for local search neighborhoods.
- Repair logic for infeasible drone deliveries.
- Diversification when the search stagnates.

## Important Parameters

Most tunable parameters are defined in `ahga.hpp`:

```cpp
constexpr int MU = 15;
constexpr int LAMBDA = 25;
constexpr int NB_ELITE = 6;
constexpr double OMEGA = 1.0;
constexpr int ITER_NI = 2500;
constexpr int K_CHEAPEST = 3;
constexpr int MAX_EDUCATION_ROUNDS = 20;
```

Change these constants before recompiling if a different search configuration is needed.
