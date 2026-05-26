#include "ahga.hpp"

double route_distance(const vector<int>& route, const map<int, map<int, double>>& dist) {
    double total = 0.0;
    for (size_t i = 0; i + 1 < route.size(); ++i) total += dist.at(route[i]).at(route[i + 1]);
    return total;
}

double truck_time_between_nodes(const vector<int>& truck_route, int start, int end,
                                const map<int, map<int, double>>& dist, double truck_speed) {
    int start_idx = index_of(truck_route, start);
    int end_idx = index_of(truck_route, end);
    if (start_idx < 0 || end_idx < 0 || start_idx >= end_idx) return numeric_limits<double>::infinity();
    double total = 0.0;
    for (int i = start_idx; i < end_idx; ++i) total += dist.at(truck_route[i]).at(truck_route[i + 1]) / truck_speed;
    return total;
}

double truck_recovery_time(const vector<Delivery>& deliveries, int rendezvous, double launch_time, double retrieve_time,
                           optional<int> excluded_customer) {
    double recovery = retrieve_time;
    for (const auto& next_delivery : deliveries) {
        if (excluded_customer && next_delivery.customer == *excluded_customer) continue;
        if (next_delivery.launch == rendezvous) {
            recovery += launch_time;
            break;
        }
    }
    return recovery;
}

double calculate_tuple_violation(int launch, int customer, int rendezvous, double truck_leg_time,
                                 const vector<Delivery>& deliveries,
                                 const map<int, map<int, double>>& dist,
                                 double endurance, double drone_speed,
                                 double launch_time, double retrieve_time,
                                 optional<int> excluded_customer) {
    double recover_truck = launch == 0 ? 0.0
        : truck_recovery_time(deliveries, rendezvous, launch_time, retrieve_time, excluded_customer);
    double truck_violation = launch == 0 ? 0.0
        : max(0.0, truck_leg_time + recover_truck - endurance);
    double drone_flight_time = dist.at(launch).at(customer) / drone_speed
        + dist.at(customer).at(rendezvous) / drone_speed;
    double drone_violation = max(0.0, drone_flight_time + retrieve_time - endurance);
    return max(truck_violation, drone_violation);
}

double calculate_endurance_violation(const Solution& solution, const map<int, map<int, double>>& dist,
                                      double drone_endurance, double truck_speed, double drone_speed,
                                      double launch_time, double retrieve_time) {
    double violation = 0.0;
    for (const auto& d : solution.drone_deliveries) {
        double truck_leg_time = d.launch == 0 ? 0.0
            : truck_time_between_nodes(solution.truck_route, d.launch, d.rendezvous, dist, truck_speed);
        violation += calculate_tuple_violation(d.launch, d.customer, d.rendezvous, truck_leg_time,
                                               solution.drone_deliveries, dist, drone_endurance,
                                               drone_speed, launch_time, retrieve_time, d.customer);
    }
    return violation;
}

double simulate_min_time_solution(const Solution& solution, const map<int, map<int, double>>& dist,
                                  double truck_speed, double drone_speed, double launch_time, double retrieve_time) {
    if (solution.truck_route.size() < 2) return 0.0;
    map<int, vector<Delivery>> launches_by_node;
    for (const auto& d : solution.drone_deliveries) launches_by_node[d.launch].push_back(d);

    double truck_time = 0.0;
    double latest_drone_return_time = 0.0;
    set<int> launched_customers;
    optional<Delivery> active_delivery;
    optional<double> active_drone_arrival;

    for (size_t idx = 0; idx + 1 < solution.truck_route.size(); ++idx) {
        int current = solution.truck_route[idx], next = solution.truck_route[idx + 1];

        if (!active_delivery && launches_by_node.count(current)) {
            for (const auto& d : launches_by_node[current]) {
                if (launched_customers.count(d.customer)) continue;
                truck_time += launch_time;
                double drone_flight = dist.at(d.launch).at(d.customer) / drone_speed
                    + dist.at(d.customer).at(d.rendezvous) / drone_speed;
                active_drone_arrival = truck_time + drone_flight;
                latest_drone_return_time = max(latest_drone_return_time, *active_drone_arrival);
                active_delivery = d;
                launched_customers.insert(d.customer);
                break;
            }
        }

        truck_time += dist.at(current).at(next) / truck_speed;

        if (active_delivery && next == active_delivery->rendezvous) {
            truck_time = max(truck_time, *active_drone_arrival) + retrieve_time;
            latest_drone_return_time = max(latest_drone_return_time, truck_time);
            active_delivery.reset();
            active_drone_arrival.reset();
        }
    }

    if (active_drone_arrival) latest_drone_return_time = max(latest_drone_return_time, *active_drone_arrival);
    return max(truck_time, latest_drone_return_time);
}

double calculate_completion_time(const Solution& solution, const map<int, map<int, double>>& dist,
                                 double truck_speed, double drone_speed, double launch_time, double retrieve_time) {
    return simulate_min_time_solution(solution, dist, truck_speed, drone_speed, launch_time, retrieve_time);
}

double calculate_operational_cost(const Solution& solution, const map<int, map<int, double>>& dist,
                                  double truck_speed, double drone_speed, double launch_time, double retrieve_time) {
    double total = solution.truck_cost_coeff * route_distance(solution.truck_route, dist);
    double drone_distance = 0.0;
    for (const auto& d : solution.drone_deliveries) {
        drone_distance += dist.at(d.launch).at(d.customer) + dist.at(d.customer).at(d.rendezvous);
    }
    total += solution.drone_cost_coeff * drone_distance;
    map<int, int> route_index;
    for (int i = 0; i < static_cast<int>(solution.truck_route.size()); ++i) route_index[solution.truck_route[i]] = i;
    for (const auto& d : solution.drone_deliveries) {
        if (!route_index.count(d.launch) || !route_index.count(d.rendezvous)) continue;
        int i = route_index[d.launch], k = route_index[d.rendezvous];
        if (i >= k) continue;
        double truck_leg_time = 0.0;
        for (int idx = i; idx < k; ++idx) truck_leg_time += dist.at(solution.truck_route[idx]).at(solution.truck_route[idx + 1]) / truck_speed;
        double drone_leg_time = launch_time + dist.at(d.launch).at(d.customer) / drone_speed + dist.at(d.customer).at(d.rendezvous) / drone_speed;
        total += solution.wait_truck_coeff * max(0.0, drone_leg_time - truck_leg_time);
        total += solution.wait_drone_coeff * max(0.0, truck_leg_time - drone_leg_time);
    }
    return total;
}

double calculate_structural_violation(const Solution& solution) {
    double violation = 0.0;
    if (solution.truck_route.empty() || solution.truck_route.front() != 0 || solution.truck_route.back() != 0) violation += 100.0;
    map<int, int> launch_indices, rendezvous_indices;
    for (int i = 0; i < static_cast<int>(solution.truck_route.size()); ++i) {
        if (!launch_indices.count(solution.truck_route[i])) launch_indices[solution.truck_route[i]] = i;
        rendezvous_indices[solution.truck_route[i]] = i;
    }
    vector<int> truck_customers;
    for (int node : solution.truck_route) if (node != 0) truck_customers.push_back(node);
    vector<int> drone_customers;
    for (const auto& d : solution.drone_deliveries) drone_customers.push_back(d.customer);
    vector<int> served = truck_customers;
    served.insert(served.end(), drone_customers.begin(), drone_customers.end());
    set<int> required_set, served_set(served.begin(), served.end());
    for (int node : solution.giant_tour) if (node != 0) required_set.insert(node);
    if (!required_set.empty()) {
        for (int node : required_set) if (!served_set.count(node)) violation += 10.0;
        for (int node : served_set) if (!required_set.count(node)) violation += 10.0;
    }
    violation += static_cast<double>(served.size() - served_set.size()) * 10.0;
    set<int> truck_set(truck_customers.begin(), truck_customers.end());
    set<int> eligible(solution.drone_eligible_customers.begin(), solution.drone_eligible_customers.end());
    for (const auto& d : solution.drone_deliveries) {
        if (!launch_indices.count(d.launch) || !rendezvous_indices.count(d.rendezvous)
            || launch_indices[d.launch] >= rendezvous_indices[d.rendezvous]) violation += 10.0;
        if (truck_set.count(d.customer)) violation += 10.0;
        if (!eligible.empty() && !eligible.count(d.customer)) violation += 10.0;
    }
    return violation;
}

double evaluate_feasibility(Solution& solution, const map<int, map<int, double>>& dist,
                            optional<double> drone_endurance) {
    double endurance = drone_endurance.value_or(solution.drone_endurance == 0.0 ? 50.0 : solution.drone_endurance);
    double endurance_violation = calculate_endurance_violation(solution, dist, endurance, solution.truck_speed,
                                                               solution.drone_speed, solution.launch_time,
                                                               solution.retrieve_time);
    double structural = calculate_structural_violation(solution);
    double total_violation = endurance_violation + structural;
    solution.endurance_violation = endurance_violation;
    solution.structural_violation = structural;
    solution.total_violation = total_violation;
    solution.is_feasible = total_violation <= EPS;
    return total_violation;
}

double evaluate_solution(Solution& solution, const map<int, map<int, double>>& dist,
                         optional<double> drone_endurance, double penalty_coefficient) {
    double completion = calculate_completion_time(solution, dist, solution.truck_speed, solution.drone_speed,
                                                  solution.launch_time, solution.retrieve_time);
    double total_violation = evaluate_feasibility(solution, dist, drone_endurance);
    double objective_value = completion;
    solution.penalty = penalty_coefficient * total_violation;
    solution.completion_time = completion;
    solution.objective_value = objective_value;
    solution.penalized_cost = objective_value + solution.penalty;
    solution.cost = solution.penalized_cost;
    return solution.cost;
}
