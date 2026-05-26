#include "ahga.hpp"

filesystem::path resolve_benchmark_path(const string& file_path) {
    filesystem::path path(file_path);
    if (filesystem::exists(path)) return path;

    filesystem::path project_root = filesystem::current_path();
    vector<filesystem::path> candidates = {
        project_root / "benchmarks" / path,
        project_root / "benchmarks" / path.filename(),
    };
    for (const auto& candidate : candidates) {
        if (filesystem::exists(candidate)) return candidate;
    }
    throw runtime_error("Benchmark file not found: " + file_path + ". Checked direct path and benchmarks/.");
}

Benchmark load_benchmark(const string& file_path) {
    filesystem::path path = resolve_benchmark_path(file_path);
    ifstream input(path);
    if (!input) throw runtime_error("Could not open benchmark file: " + path.string());

    Benchmark b;
    b.file_path = path.string();
    optional<int> n_customers;
    bool reading_nodes = false;
    bool reading_eligible = false;
    bool has_drone_eligible_section = false;
    set<int> eligible_from_node_column;
    vector<string> lines;
    string line;
    while (getline(input, line)) {
        line = trim(line);
        if (!line.empty()) lines.push_back(line);
    }

    for (const auto& raw_line : lines) {
        string u = upper(raw_line);
        auto value_after_colon = [&]() {
            auto pos = raw_line.find(':');
            return pos == string::npos ? string() : trim(raw_line.substr(pos + 1));
        };

        if (starts_with(u, "CUSTOMER_SIZE")) {
            n_customers = stoi(value_after_colon());
        } else if (starts_with(u, "DRONE_ENDURANCE") || starts_with(u, "ENDURANCE")) {
            b.drone_endurance = stod(value_after_colon());
        } else if (starts_with(u, "TRUCK_SPEED")) {
            b.truck_speed = stod(value_after_colon());
        } else if (starts_with(u, "DRONE_SPEED")) {
            b.drone_speed = stod(value_after_colon());
        } else if (starts_with(u, "LAUNCH_TIME")) {
            b.launch_time = stod(value_after_colon());
        } else if (starts_with(u, "RETRIEVE_TIME") || starts_with(u, "RETRIEVAL_TIME")) {
            b.retrieve_time = stod(value_after_colon());
        } else if (starts_with(u, "TIME_UNIT")) {
            b.time_unit = upper(value_after_colon());
            transform(b.time_unit.begin(), b.time_unit.end(), b.time_unit.begin(), [](unsigned char c) { return static_cast<char>(tolower(c)); });
        } else if (starts_with(u, "EDGE_WEIGHT_TYPE")) {
            auto pos = raw_line.find(':');
            if (pos != string::npos) {
                b.edge_weight_type = trim(raw_line.substr(pos + 1));
            } else {
                stringstream ss(raw_line);
                string part;
                while (ss >> part) b.edge_weight_type = part;
                if (b.edge_weight_type.empty()) b.edge_weight_type = "EUC_2D";
            }
        } else if (starts_with(u, "NODE_COORD_SECTION")) {
            reading_nodes = true;
            reading_eligible = false;
            continue;
        } else if (starts_with(u, "DRONE_ELIGIBLE_SECTION")) {
            reading_nodes = false;
            reading_eligible = true;
            has_drone_eligible_section = true;
            continue;
        } else if (starts_with(u, "EOF")) {
            break;
        } else if (reading_nodes) {
            stringstream ss(raw_line);
            int node_id;
            double x, y;
            if (ss >> node_id >> x >> y) {
                b.nodes[node_id] = {x, y};
                int flag;
                if (ss >> flag) {
                    if (node_id != 0 && flag == 1) eligible_from_node_column.insert(node_id);
                }
            }
        } else if (reading_eligible) {
            stringstream ss(raw_line);
            int node_id;
            while (ss >> node_id) {
                if (node_id != 0) b.drone_eligible_customers.push_back(node_id);
            }
        }
    }

    for (const auto& [node, _] : b.nodes) {
        if (node != 0) b.customers.push_back(node);
    }
    if (n_customers && static_cast<int>(b.customers.size()) != *n_customers) {
        cout << "Warning: benchmark says " << *n_customers << " customers, but "
             << b.customers.size() << " customers were read." << endl;
    }
    if (b.drone_eligible_customers.empty()) {
        if (!has_drone_eligible_section && !eligible_from_node_column.empty()) {
            b.drone_eligible_customers.assign(eligible_from_node_column.begin(), eligible_from_node_column.end());
        } else {
            b.drone_eligible_customers = b.customers;
        }
    } else {
        set<int> unique(b.drone_eligible_customers.begin(), b.drone_eligible_customers.end());
        b.drone_eligible_customers.assign(unique.begin(), unique.end());
    }
    if (b.drone_eligible_customers.empty()) b.drone_eligible_customers = b.customers;
    return b;
}
