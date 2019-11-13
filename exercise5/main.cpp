#include <algorithm>
#include <cstring>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <sstream>
#include "dijkstra.h"

/// Generate random tree via random walk
/// \param numberOfNodes
/// \return random tree with (numberOfNodes) nodes and (numberOfNodes-1) edges
graph_t GenerateRandomTree(size_t numberOfNodes)
{
    if (numberOfNodes < 2){
        return graph_t(1);
    }

    graph_t result(numberOfNodes);

    std::vector<char> visited(numberOfNodes, 0);
    size_t countVisited = 1;
    size_t currNode = 0;
    visited[currNode] = 1;

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<int64_t> nextNodeGenerator(0, numberOfNodes - (int64_t)1);
    std::uniform_real_distribution<> randomWeightGenerator(0.1, 10.0);
    while(countVisited < numberOfNodes){
        size_t nextNode = nextNodeGenerator(gen);
        if(visited[nextNode]) {
            currNode = nextNode;
            continue;
        }
        else{
            visited[nextNode] = 1;
            double weight = randomWeightGenerator(gen);
            result[currNode].push_back(std::make_pair(nextNode, weight));
            result[nextNode].push_back(std::make_pair(currNode, weight));
            currNode = nextNode;
            ++countVisited;
        }
    }
    return result;
}

graph_t GenerateRandomUndirectedGraph(size_t numberOfNodes, size_t numberOfEdges, bool connected = true)
{
	graph_t result(numberOfNodes);
	if(numberOfNodes <= 1)
		return result;


	std::random_device rd;  //Will be used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_int_distribution<int64_t> randomEdge(0, numberOfNodes-(int64_t)1);
	std::uniform_real_distribution<> randomWeight(0.1, 10.0);
	
	if(numberOfEdges >= (numberOfNodes - (int64_t)1) && connected){
        result = GenerateRandomTree(numberOfNodes); //it has (numberOfNodes - 1) edge
        numberOfEdges -= (numberOfNodes - (int64_t)1);
	}
	
	for(size_t i = 0; i < numberOfEdges; i++){
		size_t node1 = 0;
		size_t node2 = 0;
		while(node1 == node2){
			node1 = randomEdge(gen);
			node2 = randomEdge(gen);
		}
		double weight = randomWeight(gen);
		if(node1 > node2)
			std::swap(node1, node2);
		result[node1].push_back(std::make_pair(node2, weight));
		result[node2].push_back(std::make_pair(node1, weight));
	}
	
	return result;
}

void WriteGraphToOstream(const std::vector<std::vector<std::pair<size_t, double>>> &graph, std::ostream &ostr)
{
	auto size = graph.size();
	ostr << size << "\n";
	for(size_t i = 0; i < size; i++){
		for(const auto &edge: graph[i]){
			ostr << i << " " << edge.first << " " << edge.second << "\n";
		}
	}
}

graph_t ReadGraphFromFile(const std::string &path)
{
	std::ifstream ifile(path);
	std::string curString;
	
	size_t nodesCount = 0;
	
	if(std::getline(ifile, curString)){
		nodesCount = std::stoull(curString);
	}
	graph_t graph(nodesCount);
	
	while(std::getline(ifile, curString)){
	    if(curString.empty())
            continue;
		std::istringstream isstr(curString);
		size_t nodeFrom = 0;
		size_t nodeTo = 0;
		double weight = 0.0;
		isstr >> nodeFrom;
		isstr >> nodeTo;
		isstr >> weight;
		graph[nodeFrom].push_back(std::make_pair(nodeTo, weight));
	}
	
	return graph;
}

void RemoveEdgesWithGivenWeight(graph_t &graph, double weight)
{
	size_t size = graph.size();
	for(size_t i = 0; i < size; i++){
		graph[i].erase(
				std::remove_if(
						graph[i].begin(),
						graph[i].end(),
						[&weight](auto &a){ return a.second == weight;}
				),
				graph[i].end()
		);
	}
}

double FindMinimumEdgeWeight(const graph_t &graph)
{
    size_t nodesCountDiv2 = (graph.size()+1) / 2;
    double minWidth = std::numeric_limits<double>::max();
    for(size_t i = 0; i < nodesCountDiv2; i++){
        for(const auto &adj: graph[i]){
            if(adj.second < minWidth)
                minWidth = adj.second;
        }
    }
    return minWidth;
}

double FindTankWidth(const graph_t &graph, size_t startingPoint, size_t aimPoint)
{
    /// 1. Find minimun width
    double minWidth = FindMinimumEdgeWeight(graph);
	/// 2. Search for the most wide road from startingPoint to aimPoint
	auto compare = [](double a, double b){ return a > b; };
	auto F = [](const double &a, const double &b){ return a < b ? a: b; };
	double maxRoadWidth =  DijkstraWithSet::Run(graph, startingPoint, aimPoint, F, compare);
	/// 3. Return result.
	if(maxRoadWidth == std::numeric_limits<double>::min())
		return maxRoadWidth;
	else
		return maxRoadWidth - minWidth;
}

void RunPerfomanceTest()
{
    auto compare = [](double a, double b){ return a > b; };
    auto F = [](const double &a, const double &b){ return a < b ? a: b; };
    size_t nodesCount = 500000;
    size_t edgesCount = nodesCount * 2;
    std::cout << "nodes count: " << nodesCount << std::endl;
    std::cout << "edges count: " << edgesCount << std::endl;
    size_t startPoint = 1;
    size_t endPoint = 2;

    graph_t graph = GenerateRandomUndirectedGraph(nodesCount, edgesCount);

    using ms = std::chrono::duration<double, std::milli>;
    using clock = std::chrono::system_clock;

    graph = GenerateRandomUndirectedGraph(nodesCount, edgesCount);
    std::cout << "Set policy:" << std::endl;
    const auto beforeSet = clock::now();
    double resultSet =  DijkstraWithSet::Run(graph, startPoint, endPoint, F, compare);
    const ms durationSet = clock::now() - beforeSet;
    std::cout << "Duration: " << durationSet.count() << "ms" << std::endl;
    std::cout << "Result = " << resultSet << std::endl << std::endl;

    std::cout << "Vector policy:" << std::endl;
    const auto beforeVector = clock::now();
    double resultVector = DijkstraWithVector::Run(graph, startPoint, endPoint, F, compare);
    const ms durationVector = clock::now() - beforeSet;
    std::cout << "Duration: " << durationVector.count() << "ms" << std::endl;
    std::cout << "Result = " << resultVector << std::endl << std::endl;

    std::cout << "Queue policy:" << std::endl;
    const auto beforeQueue = clock::now();
    double resultQueue = DijkstraWithPrioQueue::Run(graph, startPoint, endPoint, F, compare);
    const ms durationQueue = clock::now() - beforeSet;
    std::cout << "Duration: " << durationQueue.count() << "ms" << std::endl;
    std::cout << "Result = " << resultQueue << std::endl;

}

int main(int argc, char *argv[])
{
    if(argc == 2){
        if(strcmp(argv[1], "-p") == 0){
            std::cout << "Performance test" << std::endl;
            RunPerfomanceTest();
        }
        return 0;
    }
	else if(argc < 5) {
        std::cout << "Wrong count of args" << std::endl;
        return 0;
    }
	else if(argc > 5) {
        std::cout << "Too much args" << std::endl;
        return 0;
    }
    else if(strcmp(argv[1], "-g") == 0){
		size_t nodeCount = std::stoull(argv[2]);
		size_t edgeCount = std::stoull(argv[3]);
		std::string pathToWrite = argv[4];
		auto graph = GenerateRandomUndirectedGraph(nodeCount, edgeCount);
		std::ofstream output(pathToWrite);
		WriteGraphToOstream(graph, output);
        return 0;
	}
	else if(strcmp(argv[1], "-t") == 0){
		size_t startingPoint = std::stoull(argv[2]);
		size_t aimPoint = std::stoull(argv[3]);
		std::string pathWithGraph = argv[4];
		auto graph = ReadGraphFromFile(pathWithGraph);
		auto result = FindTankWidth(std::move(graph), startingPoint, aimPoint);
		if(result == std::numeric_limits<double>::min())
			std::cout << "Starting point and aim are in different connected components" << std::endl;
		else
			std::cout << "Max tank width is: " << result;
        return 0;
	}
	else
		std::cout << "Unknown parameter: " << argv[1] << std::endl;
	
    return 0;
}