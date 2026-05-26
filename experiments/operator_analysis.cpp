#include "../src/ahga.hpp"

#include <cctype>

using namespace std;

namespace {

string instance_name_from_result(const RunResult& result, const string& benchmark_file) {
    filesystem::path path(result.benchmark.file_path.empty() ? benchmark_file : result.benchmark.file_path);
    string instance = path.stem().string();
    if (instance.empty()) instance = filesystem::path(benchmark_file).stem().string();
    return instance;
}

string short_operator_name(const string& operator_name) {
    const size_t separator = operator_name.find('_');
    return separator == string::npos ? operator_name : operator_name.substr(0, separator);
}

double success_rate(const OperatorStats& stats) {
    if (stats.selected_count == 0) return 0.0;
    return static_cast<double>(stats.improved_count) / static_cast<double>(stats.selected_count);
}

string top_operator_by_success_rate(const map<string, OperatorStats>& stats) {
    if (stats.empty()) return "n/a";
    auto best = max_element(stats.begin(), stats.end(), [](const auto& a, const auto& b) {
        const double a_rate = success_rate(a.second);
        const double b_rate = success_rate(b.second);
        if (abs(a_rate - b_rate) > EPS) return a_rate < b_rate;
        return a.second.selected_count < b.second.selected_count;
    });
    return short_operator_name(best->first);
}

string top_operator_by_final_score(const map<string, OperatorStats>& stats) {
    if (stats.empty()) return "n/a";
    auto best = max_element(stats.begin(), stats.end(), [](const auto& a, const auto& b) {
        if (abs(a.second.final_score - b.second.final_score) > EPS) {
            return a.second.final_score < b.second.final_score;
        }
        return a.first > b.first;
    });
    return short_operator_name(best->first);
}

void append_operator_analysis_csv(const RunResult& result, const string& benchmark_file, unsigned int seed) {
    const filesystem::path results_dir = filesystem::path("experiments") / "results";
    const filesystem::path results_file = results_dir / "operator_analysis.csv";

    filesystem::create_directories(results_dir);
    const bool write_header = !filesystem::exists(results_file);

    ofstream output(results_file, ios::app);
    if (!output) throw runtime_error("Could not open experiments/results/operator_analysis.csv for writing.");

    if (write_header) {
        output << "instance,seed,operator,selected_count,improved_count,not_improved_count,"
               << "success_rate,final_score,final_probability,best_min_time_minutes,"
               << "runtime_seconds,iterations\n";
    }

    const string instance = instance_name_from_result(result, benchmark_file);
    const double best_minutes = result.best_min_time_minutes.value_or(0.0);

    output << fixed << setprecision(4);
    for (const auto& [operator_name, stats] : result.operator_stats) {
        output << instance << ','
               << seed << ','
               << short_operator_name(operator_name) << ','
               << stats.selected_count << ','
               << stats.improved_count << ','
               << stats.not_improved_count << ','
               << success_rate(stats) << ','
               << stats.final_score << ','
               << stats.final_probability << ','
               << best_minutes << ','
               << result.runtime_seconds << ','
               << result.iterations << '\n';
    }
}

void print_run_summary(const RunResult& result, const string& benchmark_file, unsigned int seed) {
    const string instance = instance_name_from_result(result, benchmark_file);
    const double best_minutes = result.best_min_time_minutes.value_or(result.best_penalized_cost_minutes);

    cout << fixed << setprecision(2);
    cout << "Instance: " << instance
         << " Seed: " << seed
         << " Best: " << best_minutes
         << " Runtime: " << result.runtime_seconds << "s\n";
    cout << "Top operator by success rate: " << top_operator_by_success_rate(result.operator_stats) << '\n';
    cout << "Top operator by final score: " << top_operator_by_final_score(result.operator_stats) << "\n\n";
}

}

int main() {
    const vector<string> instances = {
        "mbB101.txt",
        "mbC106.txt",
        "mbD110.txt",
        "mbE101.txt",
        "mbF105.txt",
        "mbG110.txt",
    };
    const vector<unsigned int> seeds = {1, 2, 3};

    try {
        for (const auto& instance : instances) {
            for (unsigned int seed : seeds) {
                RunResult result = run_ahga_with_operator_stats(instance, seed, false);
                append_operator_analysis_csv(result, instance, seed);
                print_run_summary(result, instance, seed);
            }
        }
    } catch (const exception& e) {
        cerr << "Operator analysis failed: " << e.what() << endl;
        return 1;
    }

    return 0;
}
