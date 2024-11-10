#include <iostream>
#include <vector>
#include <queue>
#include <functional>
#include <limits>
#include <omp.h> // Include the OpenMP header
#include <set>

using namespace std;

const int INF = numeric_limits<int>::max();

struct Edge {
    int to;
    function<int(int)> travelTime;
};

vector<int> minTime;
priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
vector<bool> visited;

void relaxEdges(int u, int current_time, const vector<Edge>& edges) {
    #pragma omp parallel for
    for (int i = 0; i < edges.size(); ++i) {
        int v = edges[i].to;
        int weight = edges[i].travelTime(current_time);
        int arrivalTime = current_time + weight;

        #pragma omp critical
        {
            if (arrivalTime < minTime[v] && !visited[v]) {
                minTime[v] = arrivalTime;
                pq.push({arrivalTime, v});
            }
        }
    }
}

void timeDependentDijkstra(int source, const vector<vector<Edge>>& graph, int startTime) {
    int n = graph.size();
    minTime.resize(n, INF);
    visited.resize(n, false);
    minTime[source] = startTime;
    pq.push({startTime, source});

    while (!pq.empty()) {
        int current_time = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) continue;
        visited[u] = true;

        relaxEdges(u, current_time, graph[u]);
    }

    // for (int i = 0; i < n; ++i) {
    //     cout << "Minimum time to reach node " << i << " is " << (minTime[i] == INF ? -1 : minTime[i]) << endl;
    // }
}

int main() {
    auto vertices = 3000;
    vector<vector<Edge>> graph(vertices);
    // create set to keep track of existing pairs
    set<pair<int, int>> existingPairs;
    // create random edges
    for (int i = 0; i < vertices*(vertices/2); i++) {
        int from = rand() % vertices;
        int to = rand() % vertices;

        // make to biased towards being near from
        // to = from + (to-from);

        // check if the pair already exists
        if (to != from && existingPairs.find({from, to}) == existingPairs.end()) {
            existingPairs.insert({from, to});
            graph[from].push_back({to, [](int t) { return t % 10  + (rand()%3)+ 1; }});
        }
    }



    // graph[0].push_back({1, [](int t) { return 5; }});
    // graph[1].push_back({2, [](int t) { return t % 10 + 1; }});
    // graph[2].push_back({3, [](int t) { return 3; }});
    // graph[0].push_back({2, [](int t) { return 10; }});

        

    int startTime = 0;
    // time before the function is called
    double start = omp_get_wtime();
    timeDependentDijkstra(0, graph, startTime);
    // time after the function is called
    double end = omp_get_wtime();
    cout << "Time taken: " << end - start << " seconds" << endl;

    return 0;
}
