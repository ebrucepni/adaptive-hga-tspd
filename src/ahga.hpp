#ifndef AHGA_HPP
#define AHGA_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std;

constexpr const char* BENCHMARK_FILE = "mbB102.txt";
constexpr int MU = 15;
constexpr int LAMBDA = 25;
constexpr int NB_ELITE = 6;
constexpr double EREF = 0.3;
constexpr double NCLOSE = 0.2;
constexpr double OMEGA = 1.0;
constexpr int ITER_NI = 2500;
constexpr int ITER_DIV = static_cast<int>(0.3 * ITER_NI);
constexpr int K_CHEAPEST = 3;
constexpr double PREP = 0.5;
constexpr const char* OBJECTIVE = "min_time";
constexpr double EPS = 1e-9;
constexpr double TRUCK_COST_COEFF = 1.0;
constexpr double DRONE_COST_COEFF = 1.0;
constexpr double WAIT_TRUCK_COEFF = 1.0;
constexpr double WAIT_DRONE_COEFF = 1.0;
constexpr double GRANULAR_H = 0.1;
constexpr int MAX_EDUCATION_ROUNDS = 20;

extern mt19937 rng;
void set_random_seed(unsigned int seed);

struct Node {
    double x = 0.0;
    double y = 0.0;
};

struct Delivery {
    int launch = 0;
    int customer = 0;
    int rendezvous = 0;

    bool operator==(const Delivery& other) const {
        return launch == other.launch && customer == other.customer && rendezvous == other.rendezvous;
    }
};

struct Benchmark {
    string file_path;
    map<int, Node> nodes;
    vector<int> customers;
    vector<int> drone_eligible_customers;
    double drone_endurance = 20.0;
    double truck_speed = 40.0;
    double drone_speed = 40.0;
    double launch_time = 1.0;
    double retrieve_time = 1.0;
    string time_unit = "hours";
    string edge_weight_type;
};

struct Solution {
    vector<int> giant_tour;
    vector<int> truck_route;
    vector<Delivery> drone_deliveries;
    double truck_distance = 0.0;
    int number_of_drone_deliveries = 0;
    bool is_feasible = false;
    double drone_endurance = 0.0;
    double truck_speed = 40.0;
    double drone_speed = 40.0;
    double launch_time = 1.0 / 60.0;
    double retrieve_time = 1.0 / 60.0;
    string objective = OBJECTIVE;
    double truck_cost_coeff = 1.0;
    double drone_cost_coeff = 1.0;
    double wait_truck_coeff = 1.0;
    double wait_drone_coeff = 1.0;
    vector<int> drone_eligible_customers;
    double penalty = 0.0;
    double endurance_violation = 0.0;
    double structural_violation = 0.0;
    double total_violation = 0.0;
    double completion_time = 0.0;
    double objective_value = 0.0;
    double penalized_cost = 0.0;
    double cost = 0.0;
};

struct Individual {
    vector<int> route;
    Solution solution;
    double cost = 0.0;
    int cost_rank = 0;
    double diversity = 0.0;
    int diversity_rank = 0;
    double biased_fitness = 0.0;
};

struct RunResult {
    Benchmark benchmark;
    optional<Individual> best_individual;
    optional<double> best_min_time_minutes;
    Individual best_penalized_individual;
    double best_penalized_cost_minutes = 0.0;
    double runtime_seconds = 0.0;
    int iterations = 0;
    int feasible_population_size = 0;
    int infeasible_population_size = 0;
    int repaired_solutions = 0;
    int infeasible_solutions_generated = 0;
    double final_penalty = 0.0;
    map<string, double> operator_scores;
};

string trim(const string& text);
string upper(string text);
bool starts_with(const string& value, const string& prefix);
double rand01();
int randint(int a, int b);

template <typename T>
const T& random_choice(const vector<T>& values) {
    return values.at(static_cast<size_t>(randint(0, static_cast<int>(values.size()) - 1)));
}

template <typename T>
vector<T> random_sample(vector<T> values, int k) {
    shuffle(values.begin(), values.end(), rng);
    values.resize(min(k, static_cast<int>(values.size())));
    return values;
}

int index_of(const vector<int>& route, int node);
bool contains(const vector<int>& values, int node);
void remove_first(vector<int>& values, int node);

filesystem::path resolve_benchmark_path(const string& file_path);
Benchmark load_benchmark(const string& file_path);

double euclidean_distance(const Node& a, const Node& b);
string normalize_edge_weight_type(string edge_weight_type);
double pair_distance(const Node& a, const Node& b, const string& edge_weight_type);
map<int, map<int, double>> create_distance_matrix(const map<int, Node>& nodes, const string& edge_weight_type);

double route_distance(const vector<int>& route, const map<int, map<int, double>>& dist);
double truck_time_between_nodes(const vector<int>& truck_route, int start, int end, const map<int, map<int, double>>& dist, double truck_speed);
double truck_recovery_time(const vector<Delivery>& deliveries, int rendezvous, double launch_time, double retrieve_time, optional<int> excluded_customer = nullopt);
double calculate_tuple_violation(int launch, int customer, int rendezvous, double truck_leg_time, const vector<Delivery>& deliveries, const map<int, map<int, double>>& dist, double endurance, double drone_speed, double launch_time, double retrieve_time, optional<int> excluded_customer = nullopt);
double calculate_endurance_violation(const Solution& solution, const map<int, map<int, double>>& dist, double drone_endurance, double truck_speed, double drone_speed, double launch_time, double retrieve_time);
double simulate_min_time_solution(const Solution& solution, const map<int, map<int, double>>& dist, double truck_speed, double drone_speed, double launch_time, double retrieve_time);
double calculate_completion_time(const Solution& solution, const map<int, map<int, double>>& dist, double truck_speed, double drone_speed, double launch_time, double retrieve_time);
double calculate_operational_cost(const Solution& solution, const map<int, map<int, double>>& dist, double truck_speed, double drone_speed, double launch_time, double retrieve_time);
double calculate_structural_violation(const Solution& solution);
double evaluate_feasibility(Solution& solution, const map<int, map<int, double>>& dist, optional<double> drone_endurance = nullopt);
double evaluate_solution(Solution& solution, const map<int, map<int, double>>& dist, optional<double> drone_endurance = nullopt, double penalty_coefficient = OMEGA);

double calculate_drone_delivery_time(int launch, int drone_customer, int rendezvous, const map<int, map<int, double>>& dist, double drone_speed = 40.0, double launch_time = 1.0 / 60.0, double retrieve_time = 1.0 / 60.0);
bool has_interval_interference(const vector<int>& truck_route, const vector<Delivery>& drone_deliveries, int new_launch, int new_rendezvous);
Solution attach_solution_parameters(Solution solution, double drone_endurance, double truck_speed, double drone_speed, double launch_time, double retrieve_time, const string& objective, double truck_cost_coeff, double drone_cost_coeff, double wait_truck_coeff, double wait_drone_coeff, const vector<int>& drone_eligible_customers);
Solution split_route_to_tspd_solution(const vector<int>& route, const map<int, map<int, double>>& dist, double drone_endurance, double truck_speed, double drone_speed, double launch_time, double retrieve_time, const vector<int>& drone_eligible_customers, double penalty_coefficient = OMEGA);
Solution restore_giant_tour(const Solution& solution);
Solution repair_infeasible_solution(Solution solution, const map<int, map<int, double>>& dist, double drone_endurance);
Solution repair_drone_deliveries_after_truck_change(Solution solution, const map<int, map<int, double>>& dist, double drone_endurance);

vector<int> get_granular_candidates(int base_node, vector<int> candidates, const map<int, map<int, double>>& dist);
const map<int, vector<int>>& build_closest_neighbors(const map<int, map<int, double>>& dist);
bool is_close_neighbor(int base_node, int candidate, const map<int, vector<int>>& closest_neighbors);
set<int> get_drone_related_nodes(const Solution& s);
vector<int> get_truck_only_nodes(const Solution& s);
vector<pair<int, int>> get_consecutive_truck_only_pairs(const Solution& s);
set<int> get_drone_launch_and_rendezvous_nodes(const Solution& s);
bool has_drone_interference_between(const Solution& s, int i, int k);
bool is_drone_customer_eligible(const Solution& s, int customer);
bool is_drone_delivery_time_feasible(const Solution& s, int launch, int customer, int rendezvous, const map<int, map<int, double>>& dist, double drone_endurance);
Solution enforce_drone_eligibility(Solution solution, const map<int, map<int, double>>& dist);
Solution apply_local_search_operator(Solution solution, const string& operator_name, const map<int, map<int, double>>& dist, double drone_endurance, const vector<int>& drone_eligible_customers);

double insertion_cost(const vector<int>& route, int customer, int position, const map<int, map<int, double>>& dist);
vector<int> k_cheapest_insertion(vector<int> customers, const map<int, map<int, double>>& dist, int k);
vector<vector<int>> initialize_population(const vector<int>& customers, const map<int, map<int, double>>& dist, int size, int k);

map<string, double> initialize_operator_scores();
string select_operator_adaptively(const map<string, double>& scores);
map<string, double> update_operator_score(map<string, double> scores, const string& selected, bool improved);

Individual create_individual(const vector<int>& route, const map<int, map<int, double>>& dist, const Benchmark& instance, double penalty_coefficient);
Individual create_educated_individual(const vector<int>& route, const map<int, map<int, double>>& dist, const Benchmark& instance, double penalty_coefficient, map<string, double>& operator_scores);
double normalized_hamming_distance(const vector<int>& a, const vector<int>& b);
void update_biased_fitness(vector<Individual>& population, int nb_elite = NB_ELITE, double nclose_ratio = NCLOSE);
Individual tournament_selection(const vector<Individual>& population, int tournament_size = 2);
vector<int> dx_crossover(const Individual& p1, const Individual& p2);
pair<Individual, optional<Individual>> create_offspring_ahga(const vector<Individual>& complete_population, const map<int, map<int, double>>& dist, map<string, double>& operator_scores, const Benchmark& instance, double penalty_coefficient);
vector<Individual> remove_clones(vector<Individual> population);
vector<Individual> select_survivors(vector<Individual> population, int target_size, int nb_elite = NB_ELITE);
double update_penalty_coefficient(const vector<Individual>& feasible, const vector<Individual>& infeasible, double penalty);

vector<Individual> build_individuals(const vector<vector<int>>& routes, const Benchmark& instance, const map<int, map<int, double>>& dist, double penalty);
pair<vector<Individual>, vector<Individual>> split_by_feasibility(const vector<Individual>& individuals);
pair<vector<Individual>, vector<Individual>> trim_population(vector<Individual> feasible, vector<Individual> infeasible);
vector<Individual> initialize_individuals(const vector<int>& customers, const map<int, map<int, double>>& dist, const Benchmark& instance, double penalty);
pair<vector<Individual>, vector<Individual>> diversify_population(vector<Individual> complete, const vector<int>& customers, const map<int, map<int, double>>& dist, const Benchmark& instance, double penalty, map<string, double>& operator_scores);
void add_child_to_population(const Individual& child, const optional<Individual>& repaired, vector<Individual>& feasible, vector<Individual>& infeasible);
double to_minutes(double time_value, const string& unit);
double individual_min_time(const Individual& ind);
optional<Individual> best_feasible_individual(const vector<Individual>& feasible);
RunResult run_ahga(const string& benchmark_file, optional<unsigned int> seed = nullopt, bool verbose = false);

void print_section(const string& title);
void print_vector(const vector<int>& values);
void print_deliveries(const vector<Delivery>& deliveries);
void print_best_solution(const optional<Individual>& best, const optional<double>& best_min_time_minutes, const string& time_unit);

#endif
