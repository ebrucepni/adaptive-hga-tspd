#include "ahga.hpp"

map<string, double> initialize_operator_scores() {
    vector<string> names = {
        "N1_truck_relocation", "N2_truck_relocation_2_1", "N3_truck_relocation_2_1_reversed",
        "N4_truck_swap", "N5_truck_swap_2_1", "N6_truck_swap_2_2", "N7_truck_2opt",
        "N8_truck_2opt_alternative", "N9_drone_truck_swap", "N10_intradrone_launch_swap",
        "N11_intradrone_rendezvous_swap", "N12_intradrone_launch_rendezvous_swap", "N13_drone_insertion",
        "N14_drone_remove", "N15_drone_swap_1_1", "N16_drone_relocation_1_1",
    };
    map<string, double> scores;
    for (const auto& name : names) scores[name] = 1.0;
    return scores;
}

string select_operator_adaptively(const map<string, double>& scores) {
    vector<string> names;
    vector<double> weights;
    for (const auto& [name, score] : scores) {
        names.push_back(name);
        weights.push_back(score);
    }
    discrete_distribution<int> dist(weights.begin(), weights.end());
    return names[dist(rng)];
}

map<string, double> update_operator_score(map<string, double> scores, const string& selected, bool improved) {
    scores[selected] += improved ? 0.2 : -0.1;
    if (scores[selected] < 0.1) scores[selected] = 0.1;
    return scores;
}
