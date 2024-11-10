/**
 * @file tdd_par_relax.cpp
 * @brief parallel implementation of time-dependent dijkstra's shortest path algorithm
 * 
 * This is a parallel implementation of time-dependent dijkstra's shortest path algorithm. 
 * It uses the OpenMP library to parallelize the edge relaxation step.
 * 
 * @author Ethan Muchnik <emuchnik@andrew.cmu.edu>, Felipe Mautner <fmautner@andrew.cmu.edu>
*/

#include <queue>
#include <functional>
#include <limits>
#include <set>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <omp.h> 

using namespace std;

const int INF = numeric_limits<int>::max();

// Edge with a time-dependent weight function
struct Edge {
    int to;
    function<int(int)> travelTime;
};

typedef std::vector<std::vector<Edge>> AdjacencyList;


void write_outputs(const vector<int>& times, int source, int startTime, std::string& graph_filename, int num_threads) {
    
    if (std::size(graph_filename) >= 4 && graph_filename.substr(std::size(graph_filename) - 4) == ".mtx")
    {
        graph_filename = graph_filename.substr(9);
        graph_filename.resize(std::size(graph_filename) - 4);
    }
    const std::string output_filename = "outputs/" + graph_filename + "_" + to_string(source) + "_" + to_string(startTime) + "_"+ to_string(num_threads)+ "_par_relax_output.txt";
    cout << output_filename << endl;
    std::ofstream output_file(output_filename, std::fstream::out);
    output_file << "Source: " << source << ", Start Time: " << startTime << "\n";
    for (int i = 0; i < times.size(); i++) {
        output_file << "Node " << i+1 << ": " << times[i] << "\n";
    }
    output_file.close();
}

AdjacencyList readMTX(const std::string &filename) {
    std::ifstream file(filename);

    int numRows, numCols, numEntries;
    
    if (!file.is_open()) {
        std::cerr << "Error: file cannot be opened. " << filename << std::endl;
        return AdjacencyList();
    }

    std::string line;
    std::getline(file, line);
    std::getline(file, line);
    std::istringstream iss(line);
    iss >> numRows >> numCols >> numEntries;
    
    cout << "numRows: " << numRows << " numCols: " << numCols << " numEntries: " << numEntries << endl; 
    AdjacencyList graph(numRows);

    int u, v, c, b;

    while (file >> u >> v >> c >> b) {
        auto edge = Edge{v-1, [c, b](int t) { return (t%25)*c+b; }};
        graph[u-1].push_back(edge);
    }

    /* Printig out graph */
    cout << "Graph structure:" << endl;
    for (int i = 0; i < graph.size(); ++i) {
        cout << "Node " << i << " has edges to: ";
        for (const auto& edge : graph[i]) {
            cout << edge.to << " (Travel time function at t: " << edge.travelTime(0) << "), ";
        }
        cout << endl;
    }

    file.close();
    cout << "Graph read successfully" << endl;

    return graph;
}

// Modified Dijkstra's algorithm for time-dependent graphs
vector <int> tdd(int source, const vector<vector<Edge>>& graph, int startTime, int num_threads) {
    int n = graph.size();
    vector<int> minTime(n, INF);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    // Starting at the source
    minTime[source] = startTime;
    pq.push({startTime, source});

    while (!pq.empty()) {
        int current_time = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (current_time > minTime[u]) {
            continue;
        }

        int i;
        // Explore each edge from u
        #pragma omp parallel for num_threads(num_threads) shared(u) private(i)
        for (int i = 0; i < graph[u].size(); i++) {
            int thread_id = omp_get_thread_num();
            auto& edge = graph[u][i];
            // cout << "Current time: " << current_time << " Current node: " << u << ", Next node: " << edge.to << ", Arrival time: " << current_time + edge.travelTime(current_time) << endl;
            int v = edge.to;
            // cout << "Thread ID: " << thread_id << " curr node " << u << " going to: " << v << " at curr time " << current_time << endl;
            int weight = edge.travelTime(current_time);
            int arrivalTime = current_time + weight;

            // Relaxation step
            if (arrivalTime < minTime[v]) {
                minTime[v] = arrivalTime;

                #pragma omp critical 
                { 
                    pq.push({arrivalTime, v}); 
                }
            
            }
        }
        // cout << " out of for loop\n";
        // cout << "Priority Queue Status: ";
        // auto pq_copy = pq; // Copy the priority queue to avoid modifying the original
        // while (!pq_copy.empty()) {
        //     auto top = pq_copy.top();
        //     pq_copy.pop();
        //     cout << "(" << top.first << ", " << top.second << ") ";
        // }
        // cout << endl;
    }

    return minTime;
}

int main(int argc, char* argv[]) {
    
    int num_threads = atoi(argv[1]);
    char* graph_file = argv[2];
    int source = atoi(argv[3]);
    int startTime = atoi(argv[4]);
    
    std::string graph_filename = std::string(graph_file);
    AdjacencyList graph = readMTX(graph_filename);
    // vector<vector<Edge>> graph = readMTX("baby_example.mtx");

    // int startTime = 0;
    // time before the function is called
    const auto start = std::chrono::steady_clock::now();
    auto times = tdd(source-1, graph, startTime, num_threads);
    // time to complete 
    const auto time =std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count();
    // cout << "Time taken: " << time << " seconds" << endl;

    // print out mintimes
    // for (int i = 0; i < times.size(); i++) {
    //     cout << "Node " << i << ": " << times[i] << "\n";
    // }
    write_outputs(times, source, startTime, graph_filename, num_threads);
    cout << "Using " << num_threads << " threads" << endl;
    cout << "Time taken: " << time << " seconds" << endl;

    cout << "\n";

    return 0;
}
