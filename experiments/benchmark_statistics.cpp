#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>

using namespace std;

struct Record {
    double best_min_time;
    double runtime_seconds;
};

double mean(const vector<double>& values) {
    double sum = 0.0;
    for (double v : values) {
        sum += v;
    }
    return values.empty() ? 0.0 : sum / values.size();
}

int main() {
    const string input_path = "experiments/results/results.csv";
    const string output_path = "experiments/results/summary_result.csv";

    ifstream input(input_path);

    if (!input.is_open()) {
        cerr << "Could not open " << input_path << endl;
        return 1;
    }

    string line;
    getline(input, line); // header

    map<string, vector<Record>> data;

    while (getline(input, line)) {
        if (line.empty()) continue;

        stringstream ss(line);

        string instance;
        string series;
        string seed;
        string best_time;
        string runtime;
        string iterations;
        string feasible;

        getline(ss, instance, ',');
        getline(ss, series, ',');
        getline(ss, seed, ',');
        getline(ss, best_time, ',');
        getline(ss, runtime, ',');
        getline(ss, iterations, ',');
        getline(ss, feasible, ',');

        // Only B, C, D instances
        // if (series != "B" && series != "C" && series != "D") {
        //     continue;
        // }

        // Only feasible results
        if (feasible != "True" && feasible != "true" && feasible != "1") {
            continue;
        }

        Record r;
        r.best_min_time = stod(best_time);
        r.runtime_seconds = stod(runtime);

        data[instance].push_back(r);
    }

    input.close();

    ofstream output(output_path);

    if (!output.is_open()) {
        cerr << "Could not create " << output_path << endl;
        return 1;
    }

    output << "instance,min_AHGA,avg_AHGA,avg_TAHGA,runs\n";

    cout << fixed << setprecision(4);
    output << fixed << setprecision(4);

    for (const auto& pair : data) {
        const string& instance = pair.first;
        const vector<Record>& records = pair.second;

        if (records.size() != 10) {
            cerr << "Warning: " << instance
                 << " has " << records.size()
                 << " feasible runs, expected 10." << endl;
        }

        vector<double> min_times;
        vector<double> runtimes;

        for (const Record& r : records) {
            min_times.push_back(r.best_min_time);
            runtimes.push_back(r.runtime_seconds);
        }

        double min_AHGA = *min_element(min_times.begin(), min_times.end());
        double avg_AHGA = mean(min_times);
        double avg_TAHGA = mean(runtimes) / 60.0;

        output << instance << ","
               << min_AHGA << ","
               << avg_AHGA << ","
               << avg_TAHGA << ","
               << records.size()
               << "\n";

        cout << instance
             << " | min_AHGA=" << min_AHGA
             << " | avg_AHGA=" << avg_AHGA
             << " | avg_TAHGA=" << avg_TAHGA
             << " | runs=" << records.size()
             << endl;
    }

    output.close();

    cout << "\nCreated: " << output_path << endl;

    return 0;
}