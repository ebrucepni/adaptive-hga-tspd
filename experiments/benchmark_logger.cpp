#include "../src/ahga.hpp"

inline string benchmark_instance_name(const RunResult& result, const string& benchmark_file) {
    filesystem::path path(result.benchmark.file_path.empty() ? benchmark_file : result.benchmark.file_path);
    string instance = path.stem().string();
    if (instance.empty()) instance = filesystem::path(benchmark_file).stem().string();
    return instance;
}

inline string benchmark_series_from_instance(const string& instance) {
    for (size_t i = 0; i + 1 < instance.size(); ++i) {
        if (isalpha(static_cast<unsigned char>(instance[i])) &&
            isdigit(static_cast<unsigned char>(instance[i + 1]))) {
            return string(1, static_cast<char>(toupper(static_cast<unsigned char>(instance[i]))));
        }
    }
    return "";
}

inline void append_benchmark_result_csv(const RunResult& result,
                                        const string& benchmark_file,
                                        const optional<unsigned int>& seed) {
    const string instance = benchmark_instance_name(result, benchmark_file);
    const string series = benchmark_series_from_instance(instance);
    const filesystem::path results_dir = filesystem::path("experiments") / "results";
    const filesystem::path results_file = results_dir / "results.csv";

    filesystem::create_directories(results_dir);
    const bool write_header = !filesystem::exists(results_file);

    ofstream output(results_file, ios::app);
    if (!output) throw runtime_error("Could not open experiments/results/results.csv for writing.");

    if (write_header) {
        output << "instance,series,seed,best_min_time_minutes,runtime_seconds,iterations,feasible\n";
    }

    const bool feasible = result.best_individual && result.best_individual->solution.is_feasible;
    output << instance << ','
           << series << ','
           << (seed ? to_string(*seed) : string("random")) << ','
           << fixed << setprecision(4) << result.best_min_time_minutes.value_or(0.0) << ','
           << fixed << setprecision(4) << result.runtime_seconds << ','
           << result.iterations << ','
           << (feasible ? "True" : "False") << '\n';
}
