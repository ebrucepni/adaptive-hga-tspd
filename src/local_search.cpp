#include "ahga.hpp"

const map<int, vector<int>>& build_closest_neighbors(const map<int, map<int, double>>& dist) {
    static map<int, vector<int>> closest_neighbors;
    static vector<int> cached_nodes;

    vector<int> current_nodes;
    for (const auto& [node, _] : dist) current_nodes.push_back(node);
    if (!closest_neighbors.empty() && cached_nodes == current_nodes) {
        return closest_neighbors;
    }

    closest_neighbors.clear();
    cached_nodes = current_nodes;
    int nclose = max(1, static_cast<int>(ceil(GRANULAR_H * dist.size())));

    for (const auto& [base_node, row] : dist) {
        int base = base_node;
        vector<int> candidates;
        for (const auto& [node, _] : row) {
            if (node != base) candidates.push_back(node);
        }
        sort(candidates.begin(), candidates.end(), [&](int a, int b) {
            return dist.at(base).at(a) < dist.at(base).at(b);
        });
        candidates.resize(min(nclose, static_cast<int>(candidates.size())));
        closest_neighbors[base] = candidates;
    }

    return closest_neighbors;
}

bool is_close_neighbor(int base_node, int candidate, const map<int, vector<int>>& closest_neighbors) {
    auto it = closest_neighbors.find(base_node);
    if (it == closest_neighbors.end()) return false;
    return contains(it->second, candidate);
}

vector<int> get_granular_candidates(int base_node, vector<int> candidates, const map<int, map<int, double>>& dist) {
    if (candidates.empty()) return {};
    const map<int, vector<int>>& closest_neighbors = build_closest_neighbors(dist);
    vector<int> granular;
    for (int candidate : candidates) {
        if (is_close_neighbor(base_node, candidate, closest_neighbors)) granular.push_back(candidate);
    }
    return granular;
}

vector<int> get_granular_route_nodes(int base_node, const vector<int>& route, const map<int, vector<int>>& closest_neighbors) {
    vector<int> nodes;
    auto it = closest_neighbors.find(base_node);
    if (it == closest_neighbors.end()) return nodes;

    for (int node : it->second) {
        if (contains(route, node) && !contains(nodes, node)) nodes.push_back(node);
    }

    sort(nodes.begin(), nodes.end(), [&](int a, int b) {
        return index_of(route, a) < index_of(route, b);
    });
    return nodes;
}

vector<pair<int, int>> build_ordered_pairs_from_nodes(const vector<int>& route, const vector<int>& nodes) {
    vector<pair<int, int>> pairs;
    for (int launch : nodes) {
        for (int rendezvous : nodes) {
            int launch_idx = index_of(route, launch);
            int rendezvous_idx = index_of(route, rendezvous);
            if (launch_idx >= 0 && rendezvous_idx >= 0 && launch_idx < rendezvous_idx) {
                pairs.emplace_back(launch, rendezvous);
            }
        }
    }
    return pairs;
}

set<int> get_drone_related_nodes(const Solution& s) {
    set<int> nodes;
    for (const auto& d : s.drone_deliveries) {
        nodes.insert(d.launch);
        nodes.insert(d.customer);
        nodes.insert(d.rendezvous);
    }
    return nodes;
}

vector<int> get_truck_only_nodes(const Solution& s) {
    set<int> related = get_drone_related_nodes(s);
    vector<int> out;
    for (int node : s.truck_route) if (node != 0 && !related.count(node)) out.push_back(node);
    return out;
}

vector<pair<int, int>> get_consecutive_truck_only_pairs(const Solution& s) {
    vector<int> truck_only_values = get_truck_only_nodes(s);
    set<int> truck_only(truck_only_values.begin(), truck_only_values.end());
    vector<pair<int, int>> pairs;
    for (int i = 1; i < static_cast<int>(s.truck_route.size()) - 2; ++i) {
        int u1 = s.truck_route[i], u2 = s.truck_route[i + 1];
        if (truck_only.count(u1) && truck_only.count(u2)) pairs.emplace_back(u1, u2);
    }
    return pairs;
}

set<int> get_drone_launch_and_rendezvous_nodes(const Solution& s) {
    set<int> nodes;
    for (const auto& d : s.drone_deliveries) {
        nodes.insert(d.launch);
        nodes.insert(d.rendezvous);
    }
    return nodes;
}

bool has_drone_interference_between(const Solution& s, int i, int k) {
    int i_idx = index_of(s.truck_route, i), k_idx = index_of(s.truck_route, k);
    if (i_idx < 0 || k_idx < 0 || i_idx >= k_idx) return true;
    set<int> lr = get_drone_launch_and_rendezvous_nodes(s);
    for (int idx = i_idx + 1; idx < k_idx; ++idx) if (lr.count(s.truck_route[idx])) return true;
    return false;
}

bool is_drone_customer_eligible(const Solution& s, int customer) {
    set<int> eligible(s.drone_eligible_customers.begin(), s.drone_eligible_customers.end());
    return eligible.empty() || eligible.count(customer);
}

bool is_drone_delivery_time_feasible(const Solution& s, int launch, int customer, int rendezvous,
                                     const map<int, map<int, double>>& dist, double drone_endurance) {
    if (!is_drone_customer_eligible(s, customer)) return false;
    double drone_time = calculate_drone_delivery_time(launch, customer, rendezvous, dist,
                                                      s.drone_speed, s.launch_time, s.retrieve_time);
    if (drone_time > drone_endurance) return false;
    if (launch == 0) return true;
    double truck_time = truck_time_between_nodes(s.truck_route, launch, rendezvous, dist, s.truck_speed);
    double recover_truck = truck_recovery_time(s.drone_deliveries, rendezvous, s.launch_time, s.retrieve_time, customer);
    return truck_time + recover_truck <= drone_endurance;
}

Solution enforce_drone_eligibility(Solution solution, const map<int, map<int, double>>& dist) {
    set<int> eligible(solution.drone_eligible_customers.begin(), solution.drone_eligible_customers.end());
    if (eligible.empty()) return solution;
    vector<Delivery> filtered;
    for (const auto& d : solution.drone_deliveries) {
        if (eligible.count(d.customer)) filtered.push_back(d);
        else if (!contains(solution.truck_route, d.customer)) {
            int pos = index_of(solution.truck_route, d.launch);
            if (pos >= 0) solution.truck_route.insert(solution.truck_route.begin() + pos + 1, d.customer);
            else solution.truck_route.insert(solution.truck_route.end() - 1, d.customer);
        }
    }
    solution.drone_deliveries = filtered;
    solution.truck_distance = route_distance(solution.truck_route, dist);
    solution.number_of_drone_deliveries = static_cast<int>(filtered.size());
    return solution;
}

Solution n1(Solution s, const map<int, map<int, double>>& dist) {
    vector<int> truck_only = get_truck_only_nodes(s);
    if (truck_only.empty()) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    int u = random_choice(truck_only);
    remove_first(ns.truck_route, u);
    vector<int> possible(ns.truck_route.begin(), ns.truck_route.end() - 1);
    if (possible.empty()) return s;
    vector<int> granular = get_granular_candidates(u, possible, dist);
    int v = random_choice(granular.empty() ? possible : granular);
    ns.truck_route.insert(ns.truck_route.begin() + index_of(ns.truck_route, v) + 1, u);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n2(Solution s, const map<int, map<int, double>>& dist, bool reversed_pair) {
    auto pairs = get_consecutive_truck_only_pairs(s);
    if (pairs.empty()) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    auto [u1, u2] = random_choice(pairs);
    remove_first(ns.truck_route, u1);
    remove_first(ns.truck_route, u2);
    vector<int> possible;
    for (size_t i = 0; i + 1 < ns.truck_route.size(); ++i) if (ns.truck_route[i] != u1 && ns.truck_route[i] != u2) possible.push_back(ns.truck_route[i]);
    if (possible.empty()) return s;
    vector<int> granular = get_granular_candidates(u1, possible, dist);
    int v = random_choice(granular.empty() ? possible : granular);
    int vi = index_of(ns.truck_route, v);
    if (reversed_pair) {
        ns.truck_route.insert(ns.truck_route.begin() + vi + 1, u2);
        ns.truck_route.insert(ns.truck_route.begin() + vi + 2, u1);
    } else {
        ns.truck_route.insert(ns.truck_route.begin() + vi + 1, u1);
        ns.truck_route.insert(ns.truck_route.begin() + vi + 2, u2);
    }
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n4(Solution s, const map<int, map<int, double>>& dist, double endurance) {
    if (s.truck_route.size() <= 4) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    int i = randint(1, static_cast<int>(ns.truck_route.size()) - 2);
    vector<int> candidates;
    for (int idx = 1; idx < static_cast<int>(ns.truck_route.size()) - 1; ++idx) if (idx != i) candidates.push_back(idx);
    vector<int> candidate_nodes;
    for (int idx : candidates) candidate_nodes.push_back(ns.truck_route[idx]);
    vector<int> granular = get_granular_candidates(ns.truck_route[i], candidate_nodes, dist);
    int j;
    if (!granular.empty()) {
        vector<int> granular_indices;
        for (int idx : candidates) if (contains(granular, ns.truck_route[idx])) granular_indices.push_back(idx);
        j = random_choice(granular_indices.empty() ? candidates : granular_indices);
    } else {
        j = random_choice(candidates);
    }
    swap(ns.truck_route[i], ns.truck_route[j]);
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n5(Solution s, const map<int, map<int, double>>& dist, double endurance) {
    if (s.truck_route.size() <= 5) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    set<int> lr = get_drone_launch_and_rendezvous_nodes(ns);
    vector<tuple<int, int, int>> pairs;
    for (int i = 1; i < static_cast<int>(ns.truck_route.size()) - 2; ++i) {
        int u1 = ns.truck_route[i], u2 = ns.truck_route[i + 1];
        if (u2 != 0 && !lr.count(u2)) pairs.emplace_back(i, u1, u2);
    }
    if (pairs.empty()) return s;
    auto [pi, u1, u2] = random_choice(pairs);
    vector<int> possible;
    for (int idx = 1; idx < static_cast<int>(ns.truck_route.size()) - 1; ++idx) if (idx != pi && idx != pi + 1) possible.push_back(idx);
    if (possible.empty()) return s;
    vector<int> possible_nodes;
    for (int idx : possible) possible_nodes.push_back(ns.truck_route[idx]);
    vector<int> granular_nodes = get_granular_candidates(u1, possible_nodes, dist);
    vector<int> granular_indices;
    for (int idx : possible) if (contains(granular_nodes, ns.truck_route[idx])) granular_indices.push_back(idx);
    int v_index = random_choice(granular_indices.empty() ? possible : granular_indices);
    int v = ns.truck_route[v_index];
    ns.truck_route.erase(ns.truck_route.begin() + pi, ns.truck_route.begin() + pi + 2);
    if (v_index > pi) v_index -= 2;
    ns.truck_route.erase(ns.truck_route.begin() + v_index);
    ns.truck_route.insert(ns.truck_route.begin() + pi, v);
    int insert_pair_index = v_index > pi ? v_index : v_index + 1;
    ns.truck_route.insert(ns.truck_route.begin() + insert_pair_index, {u1, u2});
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n6(Solution s, const map<int, map<int, double>>& dist, double endurance) {
    if (s.truck_route.size() <= 6) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    vector<int> pair_indices;
    for (int i = 1; i < static_cast<int>(ns.truck_route.size()) - 2; ++i) if (ns.truck_route[i] != 0 && ns.truck_route[i + 1] != 0) pair_indices.push_back(i);
    vector<pair<int, int>> possible;
    const map<int, vector<int>>& closest_neighbors = build_closest_neighbors(dist);
    for (int i : pair_indices) {
        for (int j : pair_indices) {
            if (i >= j || abs(i - j) <= 1) continue;
            int u1 = ns.truck_route[i], u2 = ns.truck_route[i + 1];
            int v1 = ns.truck_route[j], v2 = ns.truck_route[j + 1];
            if (
                is_close_neighbor(u1, v1, closest_neighbors)
                || is_close_neighbor(u1, v2, closest_neighbors)
                || is_close_neighbor(u2, v1, closest_neighbors)
                || is_close_neighbor(u2, v2, closest_neighbors)
            ) {
                possible.emplace_back(i, j);
            }
        }
    }
    if (possible.empty()) {
        for (int i : pair_indices) for (int j : pair_indices) if (i < j && abs(i - j) > 1) possible.emplace_back(i, j);
    }
    if (possible.empty()) return s;
    auto [i, j] = random_choice(possible);
    vector<int> p1 = {ns.truck_route[i], ns.truck_route[i + 1]}, p2 = {ns.truck_route[j], ns.truck_route[j + 1]};
    ns.truck_route[i] = p2[0]; ns.truck_route[i + 1] = p2[1];
    ns.truck_route[j] = p1[0]; ns.truck_route[j + 1] = p1[1];
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n7(Solution s, const map<int, map<int, double>>& dist, double endurance) {
    if (s.truck_route.size() <= 5) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    vector<int> edges(ns.truck_route.size() - 1);
    iota(edges.begin(), edges.end(), 0);
    const map<int, vector<int>>& closest_neighbors = build_closest_neighbors(dist);
    vector<pair<int, int>> possible;
    for (int i : edges) {
        for (int j : edges) {
            if (i >= j || abs(i - j) <= 1) continue;
            if (
                is_close_neighbor(ns.truck_route[i], ns.truck_route[j], closest_neighbors)
                || is_close_neighbor(ns.truck_route[i + 1], ns.truck_route[j + 1], closest_neighbors)
            ) {
                possible.emplace_back(i, j);
            }
        }
    }
    if (possible.empty()) {
        for (int i : edges) for (int j : edges) if (i < j && abs(i - j) > 1) possible.emplace_back(i, j);
    }
    if (possible.empty()) return s;
    auto [i, j] = random_choice(possible);
    reverse(ns.truck_route.begin() + i + 1, ns.truck_route.begin() + j + 1);
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n8(Solution s, const map<int, map<int, double>>& dist, double endurance) {
    if (s.truck_route.size() <= 5) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    vector<int> edges(ns.truck_route.size() - 1);
    iota(edges.begin(), edges.end(), 0);
    const map<int, vector<int>>& closest_neighbors = build_closest_neighbors(dist);
    vector<pair<int, int>> possible;
    for (int i : edges) {
        for (int j : edges) {
            if (i >= j || abs(i - j) <= 1 || j + 1 >= static_cast<int>(ns.truck_route.size())) continue;
            if (
                is_close_neighbor(ns.truck_route[i], ns.truck_route[j], closest_neighbors)
                || is_close_neighbor(ns.truck_route[i + 1], ns.truck_route[j + 1], closest_neighbors)
            ) {
                possible.emplace_back(i, j);
            }
        }
    }
    if (possible.empty()) {
        for (int i : edges) for (int j : edges) if (i < j && abs(i - j) > 1 && j + 1 < static_cast<int>(ns.truck_route.size())) possible.emplace_back(i, j);
    }
    if (possible.empty()) return s;
    auto [i, j] = random_choice(possible);
    vector<int> new_route;
    new_route.insert(new_route.end(), ns.truck_route.begin(), ns.truck_route.begin() + i + 1);
    new_route.insert(new_route.end(), ns.truck_route.begin() + j + 1, ns.truck_route.end());
    new_route.insert(new_route.end(), ns.truck_route.begin() + i + 1, ns.truck_route.begin() + j + 1);
    if (new_route.front() != 0 || new_route.back() != 0) return s;
    ns.truck_route = new_route;
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n9(Solution s, const map<int, map<int, double>>& dist, double endurance) {
    if (s.drone_deliveries.empty()) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    Delivery selected = random_choice(ns.drone_deliveries);
    int li = index_of(ns.truck_route, selected.launch), ri = index_of(ns.truck_route, selected.rendezvous);
    if (li < 0 || ri < 0) return s;
    if (li > ri) swap(li, ri);
    vector<int> candidates;
    for (int idx = 0; idx < static_cast<int>(ns.truck_route.size()); ++idx) {
        int node = ns.truck_route[idx];
        if (node == 0 || node == selected.launch || node == selected.rendezvous) continue;
        if (li < idx && idx < ri) continue;
        candidates.push_back(node);
    }
    if (candidates.empty()) return s;
    int u = random_choice(candidates);
    ns.truck_route[index_of(ns.truck_route, u)] = selected.customer;
    bool replaced = false;
    for (auto& d : ns.drone_deliveries) {
        if (!replaced && d == selected) {
            d.customer = u;
            replaced = true;
        }
    }
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n10_11_12(Solution s, const map<int, map<int, double>>& dist, double endurance, int mode) {
    if (s.drone_deliveries.empty()) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    Delivery selected = random_choice(ns.drone_deliveries);
    int i_idx = index_of(ns.truck_route, selected.launch), k_idx = index_of(ns.truck_route, selected.rendezvous);
    if (i_idx < 0 || k_idx < 0 || (mode != 12 && i_idx >= k_idx)) return s;
    bool replaced = false;
    if (mode == 10) {
        ns.truck_route[i_idx] = selected.customer;
        for (auto& d : ns.drone_deliveries) if (!replaced && d == selected) { d = {selected.customer, selected.launch, selected.rendezvous}; replaced = true; }
    } else if (mode == 11) {
        ns.truck_route[k_idx] = selected.customer;
        for (auto& d : ns.drone_deliveries) if (!replaced && d == selected) { d = {selected.launch, selected.rendezvous, selected.customer}; replaced = true; }
    } else {
        swap(ns.truck_route[i_idx], ns.truck_route[k_idx]);
        for (auto& d : ns.drone_deliveries) if (!replaced && d == selected) { d = {selected.rendezvous, selected.customer, selected.launch}; replaced = true; }
    }
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n13(Solution s, const map<int, map<int, double>>& dist, double endurance) {
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    vector<int> candidates = get_truck_only_nodes(ns);
    for (const auto& d : ns.drone_deliveries) candidates.push_back(d.customer);
    set<int> eligible(ns.drone_eligible_customers.begin(), ns.drone_eligible_customers.end());
    if (!eligible.empty()) {
        vector<int> filtered;
        for (int node : candidates) if (eligible.count(node)) filtered.push_back(node);
        candidates = filtered;
    }
    if (candidates.empty()) return s;
    int j = random_choice(candidates);
    vector<Delivery> updated;
    for (const auto& d : ns.drone_deliveries) if (d.customer != j) updated.push_back(d);
    ns.drone_deliveries = updated;
    remove_first(ns.truck_route, j);
    vector<pair<int, int>> pairs;
    const map<int, vector<int>>& closest_neighbors = build_closest_neighbors(dist);
    vector<int> nearby_nodes = get_granular_route_nodes(j, ns.truck_route, closest_neighbors);
    for (const auto& [i, k] : build_ordered_pairs_from_nodes(ns.truck_route, nearby_nodes)) {
        if (i == k) continue;
        if (has_drone_interference_between(ns, i, k)) continue;
        if (is_drone_delivery_time_feasible(ns, i, j, k, dist, endurance)) pairs.emplace_back(i, k);
    }
    if (pairs.empty()) {
        for (int ii = 0; ii < static_cast<int>(ns.truck_route.size()) - 1; ++ii) {
            for (int kk = ii + 1; kk < static_cast<int>(ns.truck_route.size()); ++kk) {
                int i = ns.truck_route[ii], k = ns.truck_route[kk];
                if (i == k) continue;
                if (has_drone_interference_between(ns, i, k)) continue;
                if (is_drone_delivery_time_feasible(ns, i, j, k, dist, endurance)) pairs.emplace_back(i, k);
            }
        }
    }
    if (pairs.empty()) return s;
    auto [i, k] = random_choice(pairs);
    ns.drone_deliveries.push_back({i, j, k});
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n14(Solution s, const map<int, map<int, double>>& dist, double endurance) {
    if (s.drone_deliveries.empty()) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    Delivery selected = random_choice(ns.drone_deliveries);
    ns.drone_deliveries.erase(find(ns.drone_deliveries.begin(), ns.drone_deliveries.end(), selected));
    if (contains(ns.truck_route, selected.customer)) return s;
    vector<int> positions;
    for (int i = 0; i < static_cast<int>(ns.truck_route.size()) - 1; ++i) positions.push_back(i + 1);
    if (positions.empty()) return s;
    int pos = random_choice(positions);
    ns.truck_route.insert(ns.truck_route.begin() + pos, selected.customer);
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n15(Solution s, const map<int, map<int, double>>& dist, double endurance) {
    if (s.drone_deliveries.size() < 2) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    auto sample = random_sample(ns.drone_deliveries, 2);
    Delivery d1 = sample[0], d2 = sample[1];
    Delivery nd1{d1.launch, d2.customer, d1.rendezvous}, nd2{d2.launch, d1.customer, d2.rendezvous};
    if (!is_drone_delivery_time_feasible(ns, nd1.launch, nd1.customer, nd1.rendezvous, dist, endurance)) return s;
    if (!is_drone_delivery_time_feasible(ns, nd2.launch, nd2.customer, nd2.rendezvous, dist, endurance)) return s;
    for (auto& d : ns.drone_deliveries) {
        if (d == d1) d = nd1;
        else if (d == d2) d = nd2;
    }
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution n16(Solution s, const map<int, map<int, double>>& dist, double endurance) {
    if (s.drone_deliveries.empty()) return s;
    Solution ns = s;
    double old_cost = evaluate_solution(ns, dist);
    Delivery old = random_choice(ns.drone_deliveries);
    vector<Delivery> temp;
    for (const auto& d : ns.drone_deliveries) if (!(d == old)) temp.push_back(d);
    vector<Delivery> possible;
    const map<int, vector<int>>& closest_neighbors = build_closest_neighbors(dist);
    vector<int> nearby_nodes = get_granular_route_nodes(old.customer, ns.truck_route, closest_neighbors);
    for (const auto& [launch, rendezvous] : build_ordered_pairs_from_nodes(ns.truck_route, nearby_nodes)) {
        Delivery nd{launch, old.customer, rendezvous};
        if (nd == old || nd.launch == nd.rendezvous) continue;
        if (!is_drone_delivery_time_feasible(ns, nd.launch, nd.customer, nd.rendezvous, dist, endurance)) continue;
        if (has_interval_interference(ns.truck_route, temp, nd.launch, nd.rendezvous)) continue;
        possible.push_back(nd);
    }
    if (possible.empty()) {
        for (int li = 0; li < static_cast<int>(ns.truck_route.size()) - 1; ++li) {
            for (int ri = li + 1; ri < static_cast<int>(ns.truck_route.size()); ++ri) {
                Delivery nd{ns.truck_route[li], old.customer, ns.truck_route[ri]};
                if (nd == old || nd.launch == nd.rendezvous) continue;
                if (!is_drone_delivery_time_feasible(ns, nd.launch, nd.customer, nd.rendezvous, dist, endurance)) continue;
                if (has_interval_interference(ns.truck_route, temp, nd.launch, nd.rendezvous)) continue;
                possible.push_back(nd);
            }
        }
    }
    if (possible.empty()) return s;
    ns.drone_deliveries = temp;
    ns.drone_deliveries.push_back(random_choice(possible));
    ns = repair_drone_deliveries_after_truck_change(ns, dist, endurance);
    return evaluate_solution(ns, dist) < old_cost ? ns : s;
}

Solution apply_local_search_operator(Solution solution, const string& operator_name, const map<int, map<int, double>>& dist,
                                     double drone_endurance, const vector<int>& drone_eligible_customers) {
    if (!drone_eligible_customers.empty()) solution.drone_eligible_customers = drone_eligible_customers;
    Solution candidate = solution;
    if (operator_name == "N1_truck_relocation") candidate = n1(candidate, dist);
    else if (operator_name == "N2_truck_relocation_2_1") candidate = n2(candidate, dist, false);
    else if (operator_name == "N3_truck_relocation_2_1_reversed") candidate = n2(candidate, dist, true);
    else if (operator_name == "N4_truck_swap") candidate = n4(candidate, dist, drone_endurance);
    else if (operator_name == "N5_truck_swap_2_1") candidate = n5(candidate, dist, drone_endurance);
    else if (operator_name == "N6_truck_swap_2_2") candidate = n6(candidate, dist, drone_endurance);
    else if (operator_name == "N7_truck_2opt") candidate = n7(candidate, dist, drone_endurance);
    else if (operator_name == "N8_truck_2opt_alternative") candidate = n8(candidate, dist, drone_endurance);
    else if (operator_name == "N9_drone_truck_swap") candidate = n9(candidate, dist, drone_endurance);
    else if (operator_name == "N10_intradrone_launch_swap") candidate = n10_11_12(candidate, dist, drone_endurance, 10);
    else if (operator_name == "N11_intradrone_rendezvous_swap") candidate = n10_11_12(candidate, dist, drone_endurance, 11);
    else if (operator_name == "N12_intradrone_launch_rendezvous_swap") candidate = n10_11_12(candidate, dist, drone_endurance, 12);
    else if (operator_name == "N13_drone_insertion") candidate = n13(candidate, dist, drone_endurance);
    else if (operator_name == "N14_drone_remove") candidate = n14(candidate, dist, drone_endurance);
    else if (operator_name == "N15_drone_swap_1_1") candidate = n15(candidate, dist, drone_endurance);
    else if (operator_name == "N16_drone_relocation_1_1") candidate = n16(candidate, dist, drone_endurance);
    if (!drone_eligible_customers.empty()) candidate.drone_eligible_customers = drone_eligible_customers;
    return enforce_drone_eligibility(candidate, dist);
}
