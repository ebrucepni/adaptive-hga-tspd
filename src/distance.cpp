#include "ahga.hpp"

double euclidean_distance(const Node& a, const Node& b) {
    return hypot(a.x - b.x, a.y - b.y);
}

string normalize_edge_weight_type(string edge_weight_type) {
    if (edge_weight_type.empty()) return "EUC_2D";
    string raw = upper(trim(edge_weight_type));
    auto pos = raw.find(':');
    if (raw.find("EDGE_WEIGHT_TYPE") != string::npos && pos != string::npos) raw = trim(raw.substr(pos + 1));
    replace(raw.begin(), raw.end(), ' ', '_');
    replace(raw.begin(), raw.end(), '-', '_');
    if (raw == "EUC2D" || raw == "EUCLIDEAN" || raw == "EUCLIDEAN_2D") return "EUC_2D";
    if (raw == "MAN2D" || raw == "MANHATTAN" || raw == "MANHATTAN_2D") return "MAN_2D";
    if (raw == "CEIL2D") return "CEIL_2D";
    if (raw == "MAX2D" || raw == "CHEBYSHEV") return "MAX_2D";
    if (raw == "MAN_2D" || raw == "CEIL_2D" || raw == "MAX_2D") return raw;
    return "EUC_2D";
}

double pair_distance(const Node& a, const Node& b, const string& edge_weight_type) {
    string kind = normalize_edge_weight_type(edge_weight_type);
    if (kind == "MAN_2D") return abs(a.x - b.x) + abs(a.y - b.y);
    if (kind == "CEIL_2D") return ceil(euclidean_distance(a, b));
    if (kind == "MAX_2D") return max(abs(a.x - b.x), abs(a.y - b.y));
    return euclidean_distance(a, b);
}

map<int, map<int, double>> create_distance_matrix(const map<int, Node>& nodes, const string& edge_weight_type) {
    map<int, map<int, double>> dist;
    for (const auto& [i, a] : nodes) {
        for (const auto& [j, b] : nodes) {
            dist[i][j] = pair_distance(a, b, edge_weight_type);
        }
    }
    return dist;
}
