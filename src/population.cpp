#include "ahga.hpp"

double insertion_cost(const vector<int>& route, int customer, int position, const map<int, map<int, double>>& dist) {
    int previous = route[position - 1], next = route[position];
    return dist.at(previous).at(customer) + dist.at(customer).at(next) - dist.at(previous).at(next);
}

vector<int> k_cheapest_insertion(vector<int> customers, const map<int, map<int, double>>& dist, int k) {
    vector<int> unvisited = customers;
    int first = random_choice(unvisited);
    remove_first(unvisited, first);
    vector<int> route = {0, first, 0};
    while (!unvisited.empty()) {
        struct Insertion { int customer; int position; double cost; };
        vector<Insertion> insertions;
        for (int customer : unvisited) {
            for (int pos = 1; pos < static_cast<int>(route.size()); ++pos) {
                insertions.push_back({customer, pos, insertion_cost(route, customer, pos, dist)});
            }
        }
        sort(insertions.begin(), insertions.end(), [](const auto& a, const auto& b) { return a.cost < b.cost; });
        insertions.resize(min(k, static_cast<int>(insertions.size())));
        Insertion selected = random_choice(insertions);
        route.insert(route.begin() + selected.position, selected.customer);
        remove_first(unvisited, selected.customer);
    }
    return route;
}

vector<vector<int>> initialize_population(const vector<int>& customers, const map<int, map<int, double>>& dist, int size, int k) {
    vector<vector<int>> population;
    for (int i = 0; i < size; ++i) population.push_back(k_cheapest_insertion(customers, dist, k));
    return population;
}
