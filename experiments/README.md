# Experiments Guide

The `experiments` directory contains source files for recording and summarizing AHGA runs.

## Files

```text
benchmark_logger.cpp          # CSV logger included by src/main.cpp
benchmark_statistics.cpp      # Standalone summary generator source
operator_analysis.cpp         # Standalone adaptive operator analysis runner
results/                      # Local output directory, generated when runs are executed
```

## Result Logging

`benchmark_logger.cpp` is included by `src/main.cpp`. After each successful AHGA run, it creates or appends one row to:

```text
experiments/results/results.csv
```

The logged columns are:

```text
instance,series,seed,best_min_time_minutes,runtime_seconds,iterations,feasible
```

`results.csv` is generated output.

## Running Experiments

Compile the main executable from the project root with `-O3` optimization:

```bash
g++ -std=c++17 -O3 src/*.cpp -o ahga_tspd
```

Run one benchmark with a fixed seed:

```bash
./ahga_tspd benchmarks/mbB102.txt 1
```

Example loop for multiple seeds:

```bash
for seed in 1 2 3 4 5 6 7 8 9 10; do
  ./ahga_tspd benchmarks/mbB102.txt "$seed"
done
```

## Summary Generation

Compile the summary utility with `-O3` optimization:

```bash
g++ -std=c++17 -O3 experiments/benchmark_statistics.cpp -o benchmark_statistics
```

Create the local summary:

```bash
./benchmark_statistics
```

The summary utility reads:

```text
experiments/results/results.csv
```

and writes:

```text
experiments/results/summary_result.csv
```

The summary columns are:

```text
instance,min_AHGA,avg_AHGA,avg_TAHGA,runs
```

These benchmark result CSV files are generated outputs and are usually kept local.

## Operator Selection Analysis

`operator_analysis.cpp` is a separate experiment mode for inspecting adaptive
local search operator behavior without rerunning the full benchmark suite. It
keeps the normal AHGA flow intact and writes to a separate CSV instead of
`results.csv`.

Compile it from the project root without `src/main.cpp`:

```bash
g++ -std=c++17 -O3 src/adaptive_operator.cpp src/benchmark_loader.cpp src/distance.cpp src/drone.cpp src/fitness.cpp src/genetic_algorithm.cpp src/local_search.cpp src/population.cpp src/runner.cpp src/utils.cpp experiments/operator_analysis.cpp -o operator_analysis
```

Run:

```bash
./operator_analysis
```

The runner uses the representative instances `mbB101`, `mbC106`, `mbD110`,
`mbE101`, `mbF105`, and `mbG110` with seeds `1`, `2`, and `3`. For each local
search operator it records selection count, improvement count, non-improvement
count, success rate, final adaptive score, and final selection probability.

The output is appended to:

```text
experiments/results/operator_analysis.csv
```

This operator analysis CSV can be committed when it is used as report data.

## Notes

- `avg_TAHGA` is the average runtime converted to minutes.
- The summary utility keeps feasible runs and groups them by instance.
- The summary utility prints a warning when an instance has a number of feasible runs different from 10.
