#include "ahga.hpp"

double calculate_drone_delivery_time(int launch, int drone_customer, int rendezvous,
                                     const map<int, map<int, double>>& dist,
                                     double drone_speed,
                                     [[maybe_unused]] double launch_time,
                                     double retrieve_time) {
    return dist.at(launch).at(drone_customer) / drone_speed
        + dist.at(drone_customer).at(rendezvous) / drone_speed + retrieve_time;
}

bool has_interval_interference(const vector<int>& truck_route, const vector<Delivery>& drone_deliveries,
                               int new_launch, int new_rendezvous) {
    int new_start = index_of(truck_route, new_launch);
    int new_end = index_of(truck_route, new_rendezvous);
    if (new_start < 0 || new_end < 0 || new_start >= new_end) return true;
    for (const auto& d : drone_deliveries) {
        int existing_start = index_of(truck_route, d.launch);
        int existing_end = index_of(truck_route, d.rendezvous);
        if (existing_start < 0 || existing_end < 0 || existing_start >= existing_end) return true;
        if (!(new_end <= existing_start || new_start >= existing_end)) return true;
    }
    return false;
}

Solution attach_solution_parameters(Solution solution, double drone_endurance, double truck_speed, double drone_speed,
                                    double launch_time, double retrieve_time, const string& objective,
                                    double truck_cost_coeff, double drone_cost_coeff,
                                    double wait_truck_coeff, double wait_drone_coeff,
                                    const vector<int>& drone_eligible_customers) {
    solution.drone_endurance = drone_endurance;
    solution.truck_speed = truck_speed;
    solution.drone_speed = drone_speed;
    solution.launch_time = launch_time;
    solution.retrieve_time = retrieve_time;
    solution.objective = objective;
    solution.truck_cost_coeff = truck_cost_coeff;
    solution.drone_cost_coeff = drone_cost_coeff;
    solution.wait_truck_coeff = wait_truck_coeff;
    solution.wait_drone_coeff = wait_drone_coeff;
    solution.drone_eligible_customers = drone_eligible_customers;
    return solution;
}

Solution split_route_to_tspd_solution(const vector<int>& route, const map<int, map<int, double>>& dist,
                                      double drone_endurance, double truck_speed, double drone_speed,
                                      double launch_time, double retrieve_time,
                                      const vector<int>& drone_eligible_customers,
                                      double penalty_coefficient) {
    vector<int> tsp_route = {0};
    for (int node : route) {
        if (node != 0) tsp_route.push_back(node);
    }
    tsp_route.push_back(0);

    int n = static_cast<int>(tsp_route.size());
    set<int> eligible(drone_eligible_customers.begin(), drone_eligible_customers.end());
    if (eligible.empty()) {
        for (int node : tsp_route) if (node != 0) eligible.insert(node);
    }
    vector<double> prefix(n, 0.0);
    for (int i = 1; i < n; ++i) prefix[i] = prefix[i - 1] + dist.at(tsp_route[i - 1]).at(tsp_route[i]);

    auto truck_segment_time = [&](int i, int k) {
        return (prefix[k] - prefix[i]) / truck_speed;
    };
    auto truck_segment_time_skip_one = [&](int i, int k, int j) {
        double original = prefix[k] - prefix[i];
        int prev_node = tsp_route[j - 1], node_j = tsp_route[j], next_node = tsp_route[j + 1];
        double adjusted = original - dist.at(prev_node).at(node_j) - dist.at(node_j).at(next_node)
            + dist.at(prev_node).at(next_node);
        return adjusted / truck_speed;
    };

    vector<double> best_penalized_cost(n, numeric_limits<double>::infinity());
    vector<int> predecessor(n, -1);
    vector<int> chosen_drone_index(n, -1);
    best_penalized_cost[0] = 0.0;

    for (int i = 0; i < n - 1; ++i) {
        if (isinf(best_penalized_cost[i])) continue;
        for (int k = i + 1; k < n; ++k) {
            int launch = tsp_route[i], rendezvous = tsp_route[k];
            double previous_penalized_cost = best_penalized_cost[i];
            double segment_completion_contribution = truck_segment_time(i, k);
            double candidate_penalized = previous_penalized_cost + segment_completion_contribution;
            if (candidate_penalized < best_penalized_cost[k]) {
                best_penalized_cost[k] = candidate_penalized;
                predecessor[k] = i;
                chosen_drone_index[k] = -1;
            }
            for (int j = i + 1; j < k; ++j) {
                int drone_customer = tsp_route[j];
                if (drone_customer == 0 || !eligible.count(drone_customer)) continue;
                double truck_time_with_skip = truck_segment_time_skip_one(i, k, j);
                double drone_flight_time = dist.at(launch).at(drone_customer) / drone_speed
                    + dist.at(drone_customer).at(rendezvous) / drone_speed;
                double tuple_violation = calculate_tuple_violation(launch, drone_customer, rendezvous,
                                                                    truck_time_with_skip, {}, dist,
                                                                    drone_endurance, drone_speed,
                                                                    launch_time, retrieve_time);
                segment_completion_contribution = launch_time + max(truck_time_with_skip, drone_flight_time) + retrieve_time;
                candidate_penalized = previous_penalized_cost
                    + segment_completion_contribution
                    + penalty_coefficient * tuple_violation;
                if (candidate_penalized < best_penalized_cost[k]) {
                    best_penalized_cost[k] = candidate_penalized;
                    predecessor[k] = i;
                    chosen_drone_index[k] = j;
                }
            }
        }
    }

    Solution solution;
    solution.giant_tour = tsp_route;
    if (isinf(best_penalized_cost[n - 1])) {
        solution.truck_route = tsp_route;
    } else {
        vector<tuple<int, int, int>> segments;
        int idx = n - 1;
        while (idx != 0) {
            int start = predecessor[idx];
            segments.emplace_back(start, idx, chosen_drone_index[idx]);
            idx = start;
        }
        reverse(segments.begin(), segments.end());
        solution.truck_route = {tsp_route[0]};
        for (const auto& [start, end, drone_idx] : segments) {
            for (int node_idx = start + 1; node_idx <= end; ++node_idx) {
                if (drone_idx >= 0 && node_idx == drone_idx) continue;
                solution.truck_route.push_back(tsp_route[node_idx]);
            }
            if (drone_idx >= 0) solution.drone_deliveries.push_back({tsp_route[start], tsp_route[drone_idx], tsp_route[end]});
        }
    }
    if (solution.truck_route.empty()) {
        solution.truck_route = tsp_route;
        solution.drone_deliveries.clear();
    }
    solution.truck_distance = route_distance(solution.truck_route, dist);
    solution.number_of_drone_deliveries = static_cast<int>(solution.drone_deliveries.size());
    solution.drone_endurance = drone_endurance;
    solution.truck_speed = truck_speed;
    solution.drone_speed = drone_speed;
    solution.launch_time = launch_time;
    solution.retrieve_time = retrieve_time;
    solution.drone_eligible_customers.assign(eligible.begin(), eligible.end());
    solution.completion_time = calculate_completion_time(solution, dist, truck_speed, drone_speed, launch_time, retrieve_time);
    evaluate_feasibility(solution, dist, drone_endurance);
    solution.penalty = penalty_coefficient * solution.total_violation;
    solution.objective_value = solution.completion_time;
    solution.penalized_cost = solution.completion_time + solution.penalty;
    solution.cost = solution.penalized_cost;
    return solution;
}

Solution restore_giant_tour(const Solution& solution) {
    vector<int> reference_customers;
    for (int node : solution.giant_tour) if (node != 0) reference_customers.push_back(node);
    if (reference_customers.empty()) {
        for (int node : solution.truck_route) if (node != 0) reference_customers.push_back(node);
        for (const auto& d : solution.drone_deliveries) reference_customers.push_back(d.customer);
    }
    vector<int> candidate_order;
    for (int node : solution.truck_route) if (node != 0) candidate_order.push_back(node);
    for (const auto& d : solution.drone_deliveries) {
        int launch_idx = index_of(candidate_order, d.launch);
        int rendezvous_idx = index_of(candidate_order, d.rendezvous);
        if (launch_idx >= rendezvous_idx) continue;
        if (contains(candidate_order, d.customer)) remove_first(candidate_order, d.customer);
        launch_idx = index_of(candidate_order, d.launch);
        rendezvous_idx = index_of(candidate_order, d.rendezvous);
        if (launch_idx >= 0 && rendezvous_idx > launch_idx) {
            int insert_idx = randint(launch_idx + 1, rendezvous_idx);
            candidate_order.insert(candidate_order.begin() + insert_idx, d.customer);
        }
    }
    set<int> seen;
    vector<int> ordered;
    set<int> reference_set(reference_customers.begin(), reference_customers.end());
    for (int node : candidate_order) {
        if (node != 0 && reference_set.count(node) && !seen.count(node)) {
            ordered.push_back(node);
            seen.insert(node);
        }
    }
    for (int node : reference_customers) {
        if (!seen.count(node)) {
            ordered.push_back(node);
            seen.insert(node);
        }
    }
    Solution restored = solution;
    restored.giant_tour = ordered;
    return restored;
}

Solution repair_infeasible_solution(Solution solution, const map<int, map<int, double>>& dist, double drone_endurance) {
    vector<Delivery> repaired;
    set<int> eligible(solution.drone_eligible_customers.begin(), solution.drone_eligible_customers.end());
    for (const auto& d : solution.drone_deliveries) {
        double truck_time = d.launch == 0 ? 0.0
            : truck_time_between_nodes(solution.truck_route, d.launch, d.rendezvous, dist, solution.truck_speed);
        double tuple_violation = calculate_tuple_violation(d.launch, d.customer, d.rendezvous, truck_time,
                                                           solution.drone_deliveries, dist, drone_endurance,
                                                           solution.drone_speed, solution.launch_time,
                                                           solution.retrieve_time, d.customer);
        bool valid = tuple_violation <= EPS
            && (eligible.empty() || eligible.count(d.customer));
        if (valid) {
            repaired.push_back(d);
        } else if (!contains(solution.truck_route, d.customer)) {
            int pos = index_of(solution.truck_route, d.launch);
            if (pos >= 0) solution.truck_route.insert(solution.truck_route.begin() + pos + 1, d.customer);
            else solution.truck_route.insert(solution.truck_route.end() - 1, d.customer);
        }
    }
    solution.drone_deliveries = repaired;
    solution.truck_distance = route_distance(solution.truck_route, dist);
    solution.number_of_drone_deliveries = static_cast<int>(repaired.size());
    evaluate_feasibility(solution, dist, drone_endurance);
    return solution;
}

Solution repair_drone_deliveries_after_truck_change(Solution solution, const map<int, map<int, double>>& dist, double drone_endurance) {
    vector<Delivery> repaired;
    set<int> eligible(solution.drone_eligible_customers.begin(), solution.drone_eligible_customers.end());
    for (const auto& d : solution.drone_deliveries) {
        bool valid = true;
        int launch_idx = index_of(solution.truck_route, d.launch);
        int rendezvous_idx = index_of(solution.truck_route, d.rendezvous);
        if (launch_idx < 0 || rendezvous_idx < 0 || launch_idx >= rendezvous_idx) valid = false;
        if (valid) {
            double truck_time = d.launch == 0 ? 0.0
                : truck_time_between_nodes(solution.truck_route, d.launch, d.rendezvous, dist, solution.truck_speed);
            double tuple_violation = calculate_tuple_violation(d.launch, d.customer, d.rendezvous, truck_time,
                                                               solution.drone_deliveries, dist, drone_endurance,
                                                               solution.drone_speed, solution.launch_time,
                                                               solution.retrieve_time, d.customer);
            if (tuple_violation > EPS) valid = false;
        }
        if (valid && !eligible.empty() && !eligible.count(d.customer)) valid = false;
        if (valid) {
            repaired.push_back(d);
        } else if (!contains(solution.truck_route, d.customer)) {
            int pos = index_of(solution.truck_route, d.launch);
            if (pos >= 0) solution.truck_route.insert(solution.truck_route.begin() + pos + 1, d.customer);
            else solution.truck_route.insert(solution.truck_route.end() - 1, d.customer);
        }
    }
    solution.drone_deliveries = repaired;
    solution.truck_distance = route_distance(solution.truck_route, dist);
    solution.number_of_drone_deliveries = static_cast<int>(repaired.size());
    evaluate_feasibility(solution, dist, drone_endurance);
    return solution;
}
