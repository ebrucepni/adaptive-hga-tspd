#include "ahga.hpp"

mt19937 rng(random_device{}());

void set_random_seed(unsigned int seed) {
    rng.seed(seed);
}

string trim(const string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == string::npos) return "";
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

string upper(string text) {
    transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(toupper(c)); });
    return text;
}

bool starts_with(const string& value, const string& prefix) {
    return value.rfind(prefix, 0) == 0;
}

double rand01() {
    return uniform_real_distribution<double>(0.0, 1.0)(rng);
}

int randint(int a, int b) {
    return uniform_int_distribution<int>(a, b)(rng);
}

int index_of(const vector<int>& route, int node) {
    auto it = find(route.begin(), route.end(), node);
    return it == route.end() ? -1 : static_cast<int>(distance(route.begin(), it));
}

bool contains(const vector<int>& values, int node) {
    return find(values.begin(), values.end(), node) != values.end();
}

void remove_first(vector<int>& values, int node) {
    auto it = find(values.begin(), values.end(), node);
    if (it != values.end()) values.erase(it);
}
