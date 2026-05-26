#include "ahga.hpp"

Individual create_individual(const vector<int>& route, const map<int, map<int, double>>& dist, const Benchmark& instance,
                             double penalty_coefficient) {
    Solution solution = split_route_to_tspd_solution(route, dist, instance.drone_endurance, instance.truck_speed,
                                                     instance.drone_speed, instance.launch_time, instance.retrieve_time,
                                                     instance.drone_eligible_customers, penalty_coefficient);
    solution = attach_solution_parameters(solution, instance.drone_endurance, instance.truck_speed, instance.drone_speed,
                                          instance.launch_time, instance.retrieve_time, OBJECTIVE,
                                          TRUCK_COST_COEFF, DRONE_COST_COEFF, WAIT_TRUCK_COEFF, WAIT_DRONE_COEFF,
                                          instance.drone_eligible_customers);
    double cost = evaluate_solution(solution, dist, instance.drone_endurance, penalty_coefficient);
    return {solution.giant_tour, solution, cost};
}

vector<int> remove_depots(const vector<int>& route);

double normalized_hamming_distance(const vector<int>& a, const vector<int>& b) {
    if (a.empty() || b.empty()) return 1.0;
    vector<int> ca = remove_depots(a);
    vector<int> cb = remove_depots(b);
    int n = min(ca.size(), cb.size());
    if (n == 0) return 1.0;
    int diff = 0;
    for (int i = 0; i < n; ++i) if (ca[i] != cb[i]) ++diff;
    diff += abs(static_cast<int>(ca.size()) - static_cast<int>(cb.size()));
    return static_cast<double>(diff) / max(ca.size(), cb.size());
}

void update_biased_fitness(vector<Individual>& population, int nb_elite, double nclose_ratio) {
    if (population.empty()) return;
    vector<int> indices(population.size());
    iota(indices.begin(), indices.end(), 0);
    sort(indices.begin(), indices.end(), [&](int a, int b) { return population[a].cost < population[b].cost; });
    for (int rank = 1; rank <= static_cast<int>(indices.size()); ++rank) population[indices[rank - 1]].cost_rank = rank;
    int nclose = max(1, static_cast<int>(floor(nclose_ratio * population.size())));
    for (auto& ind : population) {
        vector<double> distances;
        for (const auto& other : population) {
            if (&other == &ind) continue;
            distances.push_back(normalized_hamming_distance(ind.route, other.route));
        }
        sort(distances.begin(), distances.end());
        ind.diversity = distances.empty() ? 0.0
            : accumulate(distances.begin(), distances.begin() + min(nclose, static_cast<int>(distances.size())), 0.0)
                / min(nclose, static_cast<int>(distances.size()));
    }
    iota(indices.begin(), indices.end(), 0);
    sort(indices.begin(), indices.end(), [&](int a, int b) { return population[a].diversity > population[b].diversity; });
    for (int rank = 1; rank <= static_cast<int>(indices.size()); ++rank) population[indices[rank - 1]].diversity_rank = rank;
    double scale = max(0.0, 1.0 - (static_cast<double>(nb_elite) / max<size_t>(1, population.size())));
    for (auto& ind : population) ind.biased_fitness = ind.cost_rank + scale * ind.diversity_rank;
}

Solution educate_solution_ahga(Solution current,
                               const map<int, map<int, double>>& dist,
                               map<string, double>& operator_scores,
                               const Benchmark& instance,
                               double penalty_coefficient) {
    bool improvement_found = true;
    int round = 0;

    while (improvement_found && round < MAX_EDUCATION_ROUNDS) {
        improvement_found = false;

        string selected_operator = select_operator_adaptively(operator_scores);
        double old_cost = evaluate_solution(current, dist, instance.drone_endurance, penalty_coefficient);

        Solution candidate = apply_local_search_operator(
            current,
            selected_operator,
            dist,
            instance.drone_endurance,
            instance.drone_eligible_customers
        );
        candidate = restore_giant_tour(candidate);
        candidate = attach_solution_parameters(candidate, instance.drone_endurance, instance.truck_speed,
                                               instance.drone_speed, instance.launch_time, instance.retrieve_time,
                                               OBJECTIVE, TRUCK_COST_COEFF, DRONE_COST_COEFF,
                                               WAIT_TRUCK_COEFF, WAIT_DRONE_COEFF,
                                               instance.drone_eligible_customers);
        double candidate_cost = evaluate_solution(candidate, dist, instance.drone_endurance, penalty_coefficient);

        if (candidate_cost < old_cost) {
            current = candidate;
            operator_scores = update_operator_score(operator_scores, selected_operator, true);
            improvement_found = true;
        } else {
            operator_scores = update_operator_score(operator_scores, selected_operator, false);
        }

        ++round;
    }

    current = restore_giant_tour(current);
    current = attach_solution_parameters(current, instance.drone_endurance, instance.truck_speed,
                                         instance.drone_speed, instance.launch_time, instance.retrieve_time,
                                         OBJECTIVE, TRUCK_COST_COEFF, DRONE_COST_COEFF,
                                         WAIT_TRUCK_COEFF, WAIT_DRONE_COEFF, instance.drone_eligible_customers);
    return current;
}

Individual create_educated_individual(const vector<int>& route, const map<int, map<int, double>>& dist,
                                      const Benchmark& instance, double penalty_coefficient,
                                      map<string, double>& operator_scores) {
    Solution solution = split_route_to_tspd_solution(route, dist, instance.drone_endurance, instance.truck_speed,
                                                     instance.drone_speed, instance.launch_time, instance.retrieve_time,
                                                     instance.drone_eligible_customers, penalty_coefficient);
    solution = attach_solution_parameters(solution, instance.drone_endurance, instance.truck_speed, instance.drone_speed,
                                          instance.launch_time, instance.retrieve_time, OBJECTIVE,
                                          TRUCK_COST_COEFF, DRONE_COST_COEFF, WAIT_TRUCK_COEFF, WAIT_DRONE_COEFF,
                                          instance.drone_eligible_customers);
    solution = educate_solution_ahga(solution, dist, operator_scores, instance, penalty_coefficient);
    double cost = evaluate_solution(solution, dist, instance.drone_endurance, penalty_coefficient);
    return {solution.giant_tour, solution, cost};
}

Individual tournament_selection(const vector<Individual>& population, int tournament_size) {
    vector<int> indices(population.size());
    iota(indices.begin(), indices.end(), 0);
    auto sample = random_sample(indices, min(tournament_size, static_cast<int>(population.size())));
    auto best_index = min_element(sample.begin(), sample.end(), [&](int a, int b) {
        double af = population[a].biased_fitness != 0.0 ? population[a].biased_fitness : population[a].cost;
        double bf = population[b].biased_fitness != 0.0 ? population[b].biased_fitness : population[b].cost;
        return af < bf;
    });
    return population[*best_index];
}

vector<int> remove_depots(const vector<int>& route) {
    vector<int> customers;
    for (int node : route) {
        if (node != 0 && !contains(customers, node)) customers.push_back(node);
    }
    return customers;
}

vector<int> normalize_route_with_depots(const vector<int>& route) {
    vector<int> normalized = {0};
    vector<int> customers = remove_depots(route);
    normalized.insert(normalized.end(), customers.begin(), customers.end());
    normalized.push_back(0);
    return normalized;
}

int find_position(const vector<int>& route, int node) {
    return index_of(route, node);
}

vector<int> choose_segment_from_chromosome(const vector<int>& chromosome) {
    if (chromosome.empty()) return {};
    if (chromosome.size() == 1) return chromosome;

    int last = static_cast<int>(chromosome.size()) - 1;
    int a = randint(0, last);
    int b = randint(0, last);
    if (a > b) swap(a, b);
    if (a == b) {
        if (b < last) ++b;
        else if (a > 0) --a;
    }
    a = max(0, min(a, last));
    b = max(a, min(b, last));
    if (a > b || a < 0 || b >= static_cast<int>(chromosome.size())) return {};

    return vector<int>(chromosome.begin() + a, chromosome.begin() + b + 1);
}

void fill_missing_by_parent2_order(vector<int>& child, const vector<int>& tsp2) {
    size_t write_pos = 1;
    for (int node : tsp2) {
        if (node == 0 || contains(child, node)) continue;
        while (write_pos + 1 < child.size() && child[write_pos] != -1) ++write_pos;
        if (write_pos + 1 < child.size()) child[write_pos] = node;
    }
}

vector<int> dx_crossover(const Individual& p1, const Individual& p2) {
    vector<int> tsp1 = normalize_route_with_depots(p1.route);
    vector<int> tsp2 = normalize_route_with_depots(p2.route);
    vector<int> child(tsp1.size(), -1);
    child.front() = 0;
    child.back() = 0;

    vector<int> td1 = remove_depots(p1.solution.truck_route);
    vector<int> dd1;
    for (const auto& delivery : p1.solution.drone_deliveries) {
        if (delivery.customer != 0 && !contains(dd1, delivery.customer)) dd1.push_back(delivery.customer);
    }

    vector<int> source_chromosome;
    if (rand01() <= 0.5) {
        source_chromosome = td1;
    } else {
        source_chromosome = dd1.size() >= 2 ? dd1 : td1;
    }

    vector<int> selected = choose_segment_from_chromosome(source_chromosome);
    if (selected.empty()) {
        selected = choose_segment_from_chromosome(remove_depots(tsp1));
    }

    for (int node : selected) {
        int pos = find_position(tsp1, node);
        if (pos > 0 && pos + 1 < static_cast<int>(tsp1.size()) && child[pos] == -1) {
            child[pos] = node;
        }
    }

    fill_missing_by_parent2_order(child, tsp2);

    for (size_t pos = 1; pos + 1 < child.size(); ++pos) {
        if (child[pos] != -1) continue;
        for (int node : tsp1) {
            if (node != 0 && !contains(child, node)) {
                child[pos] = node;
                break;
            }
        }
    }

    return child;
}

pair<Individual, optional<Individual>> create_offspring_ahga(const vector<Individual>& complete_population,
                                                             const map<int, map<int, double>>& dist,
                                                             map<string, double>& operator_scores,
                                                             const Benchmark& instance,
                                                             double penalty_coefficient) {
    Individual parent1 = tournament_selection(complete_population);
    Individual parent2 = tournament_selection(complete_population);
    vector<int> child_route = dx_crossover(parent1, parent2);
    Solution child_solution = split_route_to_tspd_solution(child_route, dist, instance.drone_endurance,
                                                           instance.truck_speed, instance.drone_speed,
                                                           instance.launch_time, instance.retrieve_time,
                                                           instance.drone_eligible_customers, penalty_coefficient);
    child_solution = attach_solution_parameters(child_solution, instance.drone_endurance, instance.truck_speed,
                                                instance.drone_speed, instance.launch_time, instance.retrieve_time,
                                                OBJECTIVE, TRUCK_COST_COEFF, DRONE_COST_COEFF,
                                                WAIT_TRUCK_COEFF, WAIT_DRONE_COEFF,
                                                instance.drone_eligible_customers);
    Solution current = educate_solution_ahga(child_solution, dist, operator_scores, instance, penalty_coefficient);
    double new_cost = evaluate_solution(current, dist, instance.drone_endurance, penalty_coefficient);
    Individual child{current.giant_tour, current, new_cost};

    optional<Individual> repaired_individual;
    if (!child.solution.is_feasible && rand01() < PREP) {
        Solution repaired = repair_infeasible_solution(child.solution, dist, instance.drone_endurance);
        repaired = restore_giant_tour(repaired);
        repaired = attach_solution_parameters(repaired, instance.drone_endurance, instance.truck_speed,
                                              instance.drone_speed, instance.launch_time, instance.retrieve_time,
                                              OBJECTIVE, TRUCK_COST_COEFF, DRONE_COST_COEFF,
                                              WAIT_TRUCK_COEFF, WAIT_DRONE_COEFF,
                                              instance.drone_eligible_customers);
        double repaired_cost = evaluate_solution(repaired, dist, instance.drone_endurance, penalty_coefficient);
        if (repaired.is_feasible) {
            repaired_individual = Individual{repaired.giant_tour, repaired, repaired_cost};
        }
    }

    return {child, repaired_individual};
}

vector<Individual> remove_clones(vector<Individual> population) {
    sort(population.begin(), population.end(), [](const auto& a, const auto& b) { return a.cost < b.cost; });
    map<vector<int>, Individual> unique;
    for (const auto& ind : population) {
        vector<int> route = ind.route;
        vector<int> rev = route;
        reverse(rev.begin(), rev.end());
        vector<int> sig = min(route, rev);
        if (!unique.count(sig)) unique[sig] = ind;
    }
    vector<Individual> out;
    for (auto& [_, ind] : unique) out.push_back(ind);
    return out;
}

vector<Individual> select_survivors(vector<Individual> population, int target_size, int nb_elite) {
    if (static_cast<int>(population.size()) <= target_size) return population;
    population = remove_clones(population);
    update_biased_fitness(population, nb_elite);
    if (static_cast<int>(population.size()) <= target_size) return population;
    sort(population.begin(), population.end(), [](const auto& a, const auto& b) { return a.biased_fitness < b.biased_fitness; });
    population.resize(target_size);
    return population;
}

double update_penalty_coefficient(const vector<Individual>& feasible, const vector<Individual>& infeasible, double penalty) {
    int total = feasible.size() + infeasible.size();
    if (total == 0) return penalty;
    double ratio = static_cast<double>(feasible.size()) / total;
    if (ratio < EREF - 0.05) penalty *= 1.2;
    else if (ratio > EREF + 0.05) penalty *= 0.85;
    return max(penalty, 1e-4);
}
