/**
 * @file timeDependentDijkstras.cpp
 * @brief sequential implementation of time-dependent dijkstra's shortest path algorithm
 * 
 * This is a simple sequential implementation of time-dependent dijkstra's shortest path algorithm.
 * @authors Felipe Mautner <fmautner@andrew.cmu.edu>, Ethan Muchnik <emuchnik@andrew.cmu.edu>
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
#include <any>
#include  <omp.h>

using namespace std;

const int INF = numeric_limits<int>::max();

/* ================================ Structs and definitions ================================ */

// Edge with a time-dependent weight function

// create class that holds a vector tuples of two ints and a function
class timeClass
{
private:
    //create vector of size 0
    vector<tuple<int, int>> times;
    int c, b;
public:
    timeClass(int c, int b): c(c), b(b) {}
    ~timeClass() {}


    // add time to vector
    void addTime(int time, int timeEnd)
    {
        times.push_back(make_tuple(time, timeEnd));
    }

    int returnTimeOfJourney(int time)
    {   
        // remove times that are no longer relevant
       

        for (int i = 0; i < times.size(); i++)
        {
            int timeStart = get<0>(times[i]);
            int timeEnd = get<1>(times[i]);
            if (timeEnd < time)
            {
                times.erase(times.begin() + i);
            }
        }
        int initialTime = time;
        int initialTimeDuration = c%25 + b;

        for (int i = 0; i < times.size(); i++)
        {
            int timeStart = get<0>(times[i]);
            int timeEnd = get<1>(times[i]);
            if (timeStart <= time && timeEnd >= time)
            {
                initialTimeDuration+=5;
            }
        }
        return initialTimeDuration;

    }

    // add values to the vector of tuples
    void addValues(int x, int z)
    {
        times.push_back(make_tuple(x, z));
    }

    void printTimes()
    {
        for (int i = 0; i < times.size(); i++)
        {
            cout << get<0>(times[i]) << " " << get<1>(times[i]) << endl;
        }
    }
    

};



//function that uses local times variable to store the times

struct Edge {
    int to;
    timeClass travelTime;
};

typedef std::vector<std::vector<Edge>> AdjacencyList;

/* ================================ Graph Parser & Writer ================================ */

void write_outputs(const vector<vector<pair<int, vector<int>>>>& times, int source, int startTime, std::string& graph_filename) {
    
    if (std::size(graph_filename) >= 4 && graph_filename.substr(std::size(graph_filename) - 4) == ".mtx")
    {
        graph_filename = graph_filename.substr(9);
        graph_filename.resize(std::size(graph_filename) - 4);
    }
    const std::string output_filename = "outputs/" + graph_filename + "_" + to_string(source) + "_" + to_string(startTime) + "_output.txt";
    cout << output_filename << endl;
    std::ofstream output_file(output_filename, std::fstream::out);
    output_file << "Source: " << source << ", Start Time: " << startTime << "\n";
    for (const auto& time : times)

        for (auto i = 0; i < time.size(); i++) {
            output_file << "Node: " << i << " Time: " << time[i].first << "\n";
            output_file << "Path: ";
            for (auto j = 0; j < time[i].second.size(); j++) {
                output_file << time[i].second[j] << " ";
            }
            output_file << "\n";
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

    //


    while (file >> u >> v >> c >> b) {
        auto edge = Edge{v, timeClass(c, b)};
        graph[u].push_back(edge);
    }

    file.close();
    cout << "Graph read successfully" << endl;

    return graph;
}

/* ================================ Time-Dependent Dijkstra's ================================ */

// Modified Dijkstra's algorithm for time-dependent graphs
vector<pair<int, vector<int>>> tdd(int source,  vector<vector<Edge>>& graph, int startTime, int end) {
    int n = graph.size();
    // cout << n;
    //exit(0);
    vector<int> minTime(n, INF);
    vector<int> pred(n, -1); // Predecessor array to track the path

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    // Starting at the source
    minTime[source] = startTime;
    pq.push({startTime, source});
    //exit(0);
    while (!pq.empty()) {
        int current_time = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // if (current_time > minTime[u]) {
        //     cout << "Skipping edge " << u << "\n" << endl;
        //     continue;
        // }

        // Explore each edge from u
        for (auto& edge : graph[u]) {
            // cout << "Current time: " << current_time << " Current node: " << u << ", Next node: " << edge.to << ", Arrival time: " << current_time + edge.travelTime(current_time) << endl;
            int v = edge.to;
            int weight = edge.travelTime.returnTimeOfJourney(current_time);
            //exit(0);
            int arrivalTime = current_time + weight;

            // Relaxation step
            if (arrivalTime < minTime[v]) {
                minTime[v] = arrivalTime;
                pred[v] = u;
                pq.push({arrivalTime, v});
            }

        }
    }
    //exit(0);

    vector<pair<int, vector<int>>> result;
    
    int v = end;
        // cout <<"here" << endl;
        vector<int> path;
        if (minTime[v] == INF) {
            result.push_back({INF, path}); // No path to this node
            return result;
        }
        
        // Reconstruct path from source to v
        int current = v;
        while (current != source) {
            path.push_back(current);
            auto node = graph[pred[current]];
            int correctIndex = -1;
            for (int i = 0; i < node.size(); i++)
            {
                if (node[i].to == current)
                {
                    correctIndex = i;
                }
            }
            node[correctIndex].travelTime.addValues(minTime[pred[current]], minTime[current]);
            //print times
            // node[correctIndex].travelTime.printTimes();
            current = pred[current];
            //find the edge that connects the two nodes
            
        }
        reverse(path.begin(), path.end());
        // cout << "i made it here" << endl;
        
        result.push_back({minTime[v], path});

    return result;
}



int main(int argc, char* argv[]) {
    
    char* graph_file = argv[1];
    int ammountCars = atoi(argv[2]); // *NOTE* this is 1-indexed
    int startTime = atoi(argv[3]);
    int num_threads = atoi(argv[4]);




    std::string graph_filename = std::string(graph_file);
    // AdjacencyList graph = readMTX(graph_filename);

    vector<AdjacencyList> graphs;
    //create one adjacency list for each thread
    for (int i = 0; i < num_threads; i++)
    {
        graphs.push_back(readMTX(graph_filename));
    }
    


    int lenGraph = graphs[0].size();
    //randomely generate start node values from 0 to lenGraph
    auto startEndTupleVector = vector<pair<int, int>>();



    for (int i = 0; i < ammountCars; i++){
        int startNode = rand() % lenGraph;
        //randomely generate end node values from 0 to lenGraph
        int endNode = rand() % lenGraph;
        startEndTupleVector.push_back(make_pair(startNode, endNode));
    }

    // vector<vector<Edge>> graph = readMTX("baby_example.mtx");

    // int startTime = 0;
    // time before the function is called
    const auto start = std::chrono::steady_clock::now();

    //create vector to hold times
    vector<vector<pair<int, vector<int>>>> theTimes;


    
    
    #pragma omp parallel for num_threads(num_threads)
    for (int i = 0; i < ammountCars; i++){
        int thread_num = omp_get_thread_num();
        vector<pair<int, vector<int>>> times = tdd(startEndTupleVector[0].first, graphs[thread_num], startTime, startEndTupleVector[i].second);
        //exit(0);

        #pragma omp critical
        theTimes.push_back(times);
    }
    // time to complete 
    const auto time =std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count();
    cout << "Time taken: " << time << " seconds" << endl;

    // print out mintimes
    // for (int i = 0; i < times.size(); i++) {
    //     cout << "Node " << i << ": " << times[i] << "\n";
    // }

    // write_outputs(theTimes, 0, startTime, graph_filename);
    // cout<< "Time taken: " << time << " seconds" << endl;
    // cout << "Time taken: " << time << " seconds" << endl;

    // cout << "\n";

    return 0;
}
