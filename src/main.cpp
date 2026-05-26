#include "ahga.hpp"
#include "../experiments/benchmark_logger.cpp"

void print_section(const string& title) {
    cout << "\n" << string(70, '=') << "\n" << title << "\n" << string(70, '=') << endl;
}

void print_vector(const vector<int>& values) {
    cout << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) cout << ", ";
        cout << values[i];
    }
    cout << "]" << endl;
}

void print_deliveries(const vector<Delivery>& deliveries) {
    cout << "[";
    for (size_t i = 0; i < deliveries.size(); ++i) {
        if (i) cout << ", ";
        cout << "(" << deliveries[i].launch << ", " << deliveries[i].customer << ", " << deliveries[i].rendezvous << ")";
    }
    cout << "]" << endl;
}

void print_best_solution(const optional<Individual>& best, const optional<double>& best_min_time_minutes, const string& time_unit) {
    print_section("FINAL BEST FEASIBLE SOLUTION");
    if (!best) {
        cout << "No feasible solution was found." << endl;
        return;
    }
    cout << "Giant route:" << endl;
    print_vector(best->route);
    cout << "\nTruck route:" << endl;
    print_vector(best->solution.truck_route);
    cout << "\nDrone deliveries:" << endl;
    print_deliveries(best->solution.drone_deliveries);
    cout << "\nFeasible:" << endl;
    cout << (best->solution.is_feasible ? "True" : "False") << endl;
    cout << "\nReal completion time (minutes):" << endl;
    cout << fixed << setprecision(4) << best_min_time_minutes.value_or(0.0) << endl;
    cout << "\nTotal violation:" << endl;
    cout << fixed << setprecision(6) << best->solution.total_violation << endl;
    cout << "\nPenalized cost (minutes):" << endl;
    cout << fixed << setprecision(4) << to_minutes(best->solution.penalized_cost, time_unit) << endl;
}

int main(int argc, char** argv) {
    try {
        string benchmark_file = argc > 1 ? argv[1] : BENCHMARK_FILE;
        optional<unsigned int> seed;
        if (argc > 2) {
            string seed_text = argv[2];
            if (seed_text.empty() || seed_text.find_first_not_of("0123456789") != string::npos) {
                throw invalid_argument("Invalid seed value. Seed must be a non-negative integer.");
            }
            try {
                unsigned long parsed_seed = stoul(seed_text);
                if (parsed_seed > numeric_limits<unsigned int>::max()) {
                    throw out_of_range("seed");
                }
                seed = static_cast<unsigned int>(parsed_seed);
            } catch (const exception&) {
                throw invalid_argument("Invalid seed value. Seed must be a non-negative integer.");
            }
            set_random_seed(*seed);
        }
        print_section("AHGA for TSP-D");
        RunResult result = run_ahga(benchmark_file, nullopt, false);
        const auto& benchmark = result.benchmark;
        print_section("Run Summary");
        cout << "Benchmark file: " << benchmark.file_path << endl;
        cout << "Seed: " << (seed ? to_string(*seed) : string("random")) << endl;
        cout << "Edge weight type: " << (benchmark.edge_weight_type.empty() ? "EUC_2D" : benchmark.edge_weight_type) << endl;
        cout << "Number of customers: " << benchmark.customers.size() << endl;
        cout << "Drone eligible customers: " << benchmark.drone_eligible_customers.size() << endl;
        cout << "mu: " << MU << endl;
        cout << "lambda: " << LAMBDA << endl;
        cout << "IterNI: " << ITER_NI << endl;
        cout << "IterDIV: " << ITER_DIV << endl;
        cout << "Iteration count: " << result.iterations << endl;
        cout << "Feasible population size: " << result.feasible_population_size << endl;
        cout << "Infeasible population size: " << result.infeasible_population_size << endl;
        cout << "Final omega: " << fixed << setprecision(8) << result.final_penalty << endl;
        cout << "Best feasible completion_time (minutes): ";
        if (result.best_min_time_minutes) cout << fixed << setprecision(4) << *result.best_min_time_minutes;
        else cout << "n/a";
        cout << endl;
        cout << "Best feasible total_violation: ";
        if (result.best_individual) cout << fixed << setprecision(6) << result.best_individual->solution.total_violation;
        else cout << "n/a";
        cout << endl;
        cout << "Best feasible penalized_cost (minutes): ";
        if (result.best_individual) cout << fixed << setprecision(4) << to_minutes(result.best_individual->solution.penalized_cost, benchmark.time_unit);
        else cout << "n/a";
        cout << endl;
        cout << "Best feasible flag: ";
        if (result.best_individual) cout << (result.best_individual->solution.is_feasible ? "True" : "False");
        else cout << "n/a";
        cout << endl;
        cout << "Best penalized cost (minutes): " << fixed << setprecision(4) << result.best_penalized_cost_minutes << endl;
        cout << "Best penalized real completion time (minutes): "
             << fixed << setprecision(4)
             << to_minutes(result.best_penalized_individual.solution.completion_time, benchmark.time_unit) << endl;
        cout << "Best penalized solution penalized_cost (minutes): "
             << fixed << setprecision(4)
             << to_minutes(result.best_penalized_individual.solution.penalized_cost, benchmark.time_unit) << endl;
        cout << "Best penalized solution total_violation: "
             << fixed << setprecision(6) << result.best_penalized_individual.solution.total_violation << endl;
        cout << "Best penalized solution feasible: "
             << (result.best_penalized_individual.solution.is_feasible ? "True" : "False") << endl;
        cout << "Repaired solutions: " << result.repaired_solutions << endl;
        cout << "Infeasible solutions generated: " << result.infeasible_solutions_generated << endl;
        cout << "Runtime seconds: " << fixed << setprecision(4) << result.runtime_seconds << endl;
        print_best_solution(result.best_individual, result.best_min_time_minutes, benchmark.time_unit);
        append_benchmark_result_csv(result, benchmark_file, seed);
    } catch (const exception& ex) {
        cerr << ex.what() << endl;
        return 1;
    }
    return 0;
}
