#include "ahga.hpp"

vector<Individual> build_individuals(const vector<vector<int>>& routes, const Benchmark& instance,
                                     const map<int, map<int, double>>& dist, double penalty) {
    vector<Individual> out;
    for (const auto& route : routes) out.push_back(create_individual(route, dist, instance, penalty));
    return out;
}

pair<vector<Individual>, vector<Individual>> split_by_feasibility(const vector<Individual>& individuals) {
    vector<Individual> feasible, infeasible;
    for (const auto& ind : individuals) {
        (ind.solution.is_feasible ? feasible : infeasible).push_back(ind);
    }
    return {feasible, infeasible};
}

pair<vector<Individual>, vector<Individual>> trim_population(vector<Individual> feasible, vector<Individual> infeasible) {
    if (static_cast<int>(feasible.size()) > MU + LAMBDA) feasible = select_survivors(feasible, MU);
    if (static_cast<int>(infeasible.size()) > MU + LAMBDA) infeasible = select_survivors(infeasible, MU);
    return {feasible, infeasible};
}

void reevaluate_population(vector<Individual>& population, const map<int, map<int, double>>& dist,
                           double drone_endurance, double penalty) {
    for (auto& ind : population) {
        ind.cost = evaluate_solution(ind.solution, dist, drone_endurance, penalty);
        ind.route = ind.solution.giant_tour;
    }
}

vector<Individual> initialize_individuals(const vector<int>& customers, const map<int, map<int, double>>& dist,
                                          const Benchmark& instance, double penalty) {
    return build_individuals(initialize_population(customers, dist, 4 * MU, K_CHEAPEST), instance, dist, penalty);
}

pair<vector<Individual>, vector<Individual>> diversify_population(vector<Individual> complete, const vector<int>& customers,
                                                                  const map<int, map<int, double>>& dist,
                                                                  const Benchmark& instance, double penalty,
                                                                  map<string, double>& operator_scores) {
    int keep_count = max(1, MU / 3);
    update_biased_fitness(complete);
    sort(complete.begin(), complete.end(), [](const auto& a, const auto& b) { return a.biased_fitness < b.biased_fitness; });
    vector<Individual> combined(complete.begin(), complete.begin() + min(keep_count, static_cast<int>(complete.size())));
    vector<vector<int>> fresh_routes = initialize_population(customers, dist, 4 * MU, K_CHEAPEST);
    for (const auto& route : fresh_routes) {
        combined.push_back(create_educated_individual(route, dist, instance, penalty, operator_scores));
    }
    auto [feasible, infeasible] = split_by_feasibility(combined);
    return {select_survivors(feasible, MU), select_survivors(infeasible, MU)};
}

void add_child_to_population(const Individual& child, const optional<Individual>& repaired,
                             vector<Individual>& feasible, vector<Individual>& infeasible) {
    if (child.solution.is_feasible) feasible.push_back(child);
    else {
        infeasible.push_back(child);
        if (repaired && repaired->solution.is_feasible) feasible.push_back(*repaired);
    }
}

double to_minutes(double time_value, const string& unit) {
    return unit == "hours" ? time_value * 60.0 : time_value;
}

double individual_min_time(const Individual& ind) {
    return ind.solution.completion_time;
}

optional<Individual> best_feasible_individual(const vector<Individual>& feasible) {
    if (feasible.empty()) return nullopt;
    vector<Individual> valid;
    for (const auto& ind : feasible) {
        if (ind.solution.is_feasible) valid.push_back(ind);
    }
    if (valid.empty()) return nullopt;
    return *min_element(valid.begin(), valid.end(), [](const auto& a, const auto& b) {
        return individual_min_time(a) < individual_min_time(b);
    });
}

RunResult run_ahga(const string& benchmark_file, optional<unsigned int> seed, bool verbose) {
    if (seed) rng.seed(*seed);
    auto start = chrono::steady_clock::now();
    Benchmark instance = load_benchmark(benchmark_file.empty() ? BENCHMARK_FILE : benchmark_file);
    auto dist = create_distance_matrix(instance.nodes, instance.edge_weight_type);

    double penalty = OMEGA;
    vector<Individual> initial = initialize_individuals(instance.customers, dist, instance, penalty);
    auto [feasible, infeasible] = split_by_feasibility(initial);
    feasible = select_survivors(feasible, MU);
    infeasible = select_survivors(infeasible, MU);
    vector<Individual> complete = feasible;
    complete.insert(complete.end(), infeasible.begin(), infeasible.end());
    update_biased_fitness(complete);
    Individual global_best_penalized = *min_element(complete.begin(), complete.end(), [](const auto& a, const auto& b) { return a.cost < b.cost; });
    optional<Individual> global_best_feasible = best_feasible_individual(feasible);
    auto operator_scores = initialize_operator_scores();

    int iteration = 0;
    int without_improvement = 0;
    int no_improvement_for_diversification = 0;
    int repaired_solutions = 0;
    int infeasible_solutions_generated = static_cast<int>(infeasible.size());
    while (without_improvement < ITER_NI) {
        ++iteration;
        complete = feasible;
        complete.insert(complete.end(), infeasible.begin(), infeasible.end());
        if (complete.size() < 2) break;
        update_biased_fitness(complete);
        auto [child, repaired] = create_offspring_ahga(complete, dist, operator_scores, instance, penalty);
        if (!child.solution.is_feasible) ++infeasible_solutions_generated;
        if (repaired && repaired->solution.is_feasible) ++repaired_solutions;
        add_child_to_population(child, repaired, feasible, infeasible);
        tie(feasible, infeasible) = trim_population(feasible, infeasible);
        if (iteration % 100 == 0) {
            penalty = update_penalty_coefficient(feasible, infeasible, penalty);
            reevaluate_population(feasible, dist, instance.drone_endurance, penalty);
            reevaluate_population(infeasible, dist, instance.drone_endurance, penalty);
            vector<Individual> refreshed = feasible;
            refreshed.insert(refreshed.end(), infeasible.begin(), infeasible.end());
            tie(feasible, infeasible) = split_by_feasibility(refreshed);
        }
        complete = feasible;
        complete.insert(complete.end(), infeasible.begin(), infeasible.end());
        evaluate_solution(global_best_penalized.solution, dist, instance.drone_endurance, penalty);
        global_best_penalized.cost = global_best_penalized.solution.cost;
        global_best_penalized.route = global_best_penalized.solution.giant_tour;
        Individual generation_best = *min_element(complete.begin(), complete.end(), [](const auto& a, const auto& b) { return a.cost < b.cost; });
        if (generation_best.cost < global_best_penalized.cost) global_best_penalized = generation_best;
        optional<Individual> gen_best_feasible = best_feasible_individual(feasible);
        if (gen_best_feasible && (!global_best_feasible || individual_min_time(*gen_best_feasible) < individual_min_time(*global_best_feasible))) {
            global_best_feasible = *gen_best_feasible;
            without_improvement = 0;
            no_improvement_for_diversification = 0;
        } else {
            ++without_improvement;
            ++no_improvement_for_diversification;
        }
        if (no_improvement_for_diversification >= ITER_DIV) {
            tie(feasible, infeasible) = diversify_population(complete, instance.customers, dist, instance, penalty, operator_scores);
            no_improvement_for_diversification = 0;
        }
        if (verbose) {
            cout << "Iter " << iteration << " | best penalized(min): "
                 << fixed << setprecision(4) << to_minutes(generation_best.cost, instance.time_unit)
                 << " | best min time: ";
            if (global_best_feasible) cout << fixed << setprecision(4) << to_minutes(individual_min_time(*global_best_feasible), instance.time_unit);
            else cout << "n/a";
            cout << endl;
        }
    }
    double runtime = chrono::duration<double>(chrono::steady_clock::now() - start).count();
    RunResult result;
    result.benchmark = instance;
    result.best_individual = global_best_feasible;
    if (global_best_feasible) result.best_min_time_minutes = to_minutes(individual_min_time(*global_best_feasible), instance.time_unit);
    result.best_penalized_individual = global_best_penalized;
    result.best_penalized_cost_minutes = to_minutes(global_best_penalized.cost, instance.time_unit);
    result.runtime_seconds = runtime;
    result.iterations = iteration;
    result.feasible_population_size = static_cast<int>(feasible.size());
    result.infeasible_population_size = static_cast<int>(infeasible.size());
    result.repaired_solutions = repaired_solutions;
    result.infeasible_solutions_generated = infeasible_solutions_generated;
    result.final_penalty = penalty;
    result.operator_scores = operator_scores;
    return result;
}
