/**
 * @file tdd_par_.cpp
 * @brief parallel implementation of time-dependent dijkstra's shortest path algorithm
 * 
 * This is a parallel implementation of time-dependent dijkstra's shortest path algorithm. 
 * It uses the OpenMP library to parallelize (FILL IN DETAILS).
 * 
 * @author Felipe Mautner <fmautner@andrew.cmu.edu>
*/

#include <iostream>
#include <vector>
#include <queue>
#include <functional>
#include <limits>
#include <set>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std;

const int INF = numeric_limits<int>::max();

/* ================================ Structs and definitions ================================ */

// Edge with a time-dependent weight function
struct Edge {
    int to;
    function<int(int)> travelTime;
};

typedef std::vector<std::vector<Edge>> AdjacencyList;

/* ================================ Graph Parser & Writer ================================ */

void write_outputs(const vector<int>& times, int source, int startTime, std::string& graph_filename, int num_threads) {
    
    if (std::size(graph_filename) >= 4 && graph_filename.substr(std::size(graph_filename) - 4) == ".mtx")
    {
        graph_filename = graph_filename.substr(9);
        graph_filename.resize(std::size(graph_filename) - 4);
    }
    const std::string output_filename = "outputs/" + graph_filename + "_" + to_string(source) + "_" + to_string(startTime) + "_"+ to_string(num_threads)+ "_par_set_output.txt";
    cout << output_filename << endl;
    std::ofstream output_file(output_filename, std::fstream::out);
    output_file << "Source: " << source << ", Start Time: " << startTime << "\n";
    for (int i = 0; i < times.size(); i++) {
        output_file << "Node " << i+1 << ": " << times[i] << "\n";
    }
    output_file.close();
}

// assumes mtx file is 1-indexed
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

    //Function that returns the travel time at time t and with an array of sets of two points that describe when travel time will increase for a certain time
    

    while (file >> u >> v >> c >> b) {
        auto edge = Edge{v-1, [c, b](int t, ve) { return (t%25)*c+b; }};
        graph[u-1].push_back(edge);
    }

    /* Printig out graph */
    // cout << "Graph structure:" << endl;
    // for (int i = 0; i < graph.size(); ++i) {
    //     cout << "Node " << i << " has edges to: ";
    //     for (const auto& edge : graph[i]) {
    //         cout << edge.to << " (Travel time function at t: " << edge.travelTime(0) << "), ";
    //     }
    //     cout << endl;
    // }

    file.close();
    cout << "Graph read successfully" << endl;

    return graph;
}

/* ================================ Time-Dependent Dijkstra's ================================ */

int closest_unseen_vertex(const std::vector<bool>& visited, const vector<int>& minTime, int num_threads) {
    int min = INF;
    int u = -1;

    #pragma omp parallel
    {
        int t_min = INF;
        int t_id = -1;

        // std::set<int>::iterator it;

        #pragma omp parallel for num_threads(num_threads)
        for (int i = 0; i < minTime.size(); i++) {
            // cout << " looking at " << i << endl;
            if (!visited[i] && minTime[i] < t_min) {
                t_min = minTime[i];
                t_id = i;
            }
        }

        #pragma omp critical
        {
            if (t_min < min) {
                min = t_min;
                u = t_id;
            }
        }
    }
    return u;
}


vector <int> tdd(int source,  const vector<vector<Edge>>& graph, int startTime, int num_threads) {
    
    cout << "tdd called" << endl;
    int n = graph.size();
    vector<int> minTime(n, INF);
    minTime[source] = startTime;
    vector<bool> visited(n, false);

    // while |S| < n:
    while (true) {

        // u = closest_unseen_vertex
        int u = closest_unseen_vertex(visited, minTime, num_threads);
        if (u == -1) {
            break;
        }
        // U = U - {u}
        visited[u] = true;

        int curr_time = minTime[u];

        // parallel for each outgoing v in N(u): (shared u, thread private v)
        // #pragma omp parallel for num_threads(num_threads) shared(graph, u, minTime, curr_time) //private(v)
        for (int i = 0; i < graph[u].size(); i++) {
            auto& edge = graph[u][i];
            int v = edge.to;
            int weight = edge.travelTime(curr_time);
            int arrivalTime = curr_time + weight;
            

            if (arrivalTime < minTime[v]) {
                minTime[v] = arrivalTime;
                // predecessor[v] = u;
            }
        }
    }

    return minTime;
}

// #include <omp.h>
// #include <vector>
// #include <queue>
// #include <iostream>
// #include <limits>
// #define INF std::numeric_limits<int>::max()

// struct Edge {
//     int to;
//     int travelTime(int time) const {
//         // Define how travel time is determined based on the input time.
//         return time % 10 + 5;  // Example function
//     }
// };

// vector<int> tdd(int source, const vector<vector<Edge>>& graph, int startTime) {
//     int n = graph.size();
//     cout << n;
//     vector<int> minTime(n, INF);
//     priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

//     minTime[source] = startTime;
//     pq.push({startTime, source});

//     while (!pq.empty()) {
//         int current_time = pq.top().first;
//         int u = pq.top().second;
//         pq.pop();

//         // #pragma omp parallel for schedule(dynamic)
//         for (int i = 0; i < graph[u].size(); i++) {
//             auto& edge = graph[u][i];
//             int v = edge.to;
//             int weight = edge.travelTime(current_time);
//             int arrivalTime = current_time + weight;

//             // #pragma omp critical
//             {
//                 if (arrivalTime < minTime[v]) {
//                     minTime[v] = arrivalTime;
//                     pq.push({arrivalTime, v});
//                 }
//             }
//         }
//     }

//     return minTime;
// }



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
    cout << "starting tdd" << endl;
    auto times = tdd(source-1, graph, startTime);
    // time to complete 
    const auto time =std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count();
    // cout << "Time taken: " << time << " seconds" << endl;

    // print out mintimes
    // for (int i = 0; i < times.size(); i++) {
    //     cout << "Node " << i << ": " << times[i] << "\n";
    // }
    cout << "writing outputs" << endl;
    write_outputs(times, source, startTime, graph_filename, num_threads);
    cout << "Using " << num_threads << " threads" << endl;
    cout << "Time taken: " << time << " seconds" << endl;

    cout << "\n";

    return 0;
}
