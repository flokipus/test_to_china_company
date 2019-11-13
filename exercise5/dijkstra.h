//
// Created by elrond on 11/12/19.
//

#ifndef HT_DIJKSTRA_H
#define HT_DIJKSTRA_H

#include <vector>
#include <limits>
#include <set>
#include <functional>
#include <queue>

using graph_t = std::vector<std::vector<std::pair<size_t, double>>>;

namespace UnvisitedNodesStoragePolicy {
	template<class T, class Compare>
	class NonVisitedNodesVector {
	public:
		NonVisitedNodesVector(const Compare &lessFunctor,
		                      size_t nodeCount) :
				isVisited_(nodeCount, 0),
				nodesValue_(nodeCount, std::numeric_limits<T>::min()),
				visitedCount(0),
				lessFunctor_(lessFunctor),
				flagTopIsUptoDate(false),
				topNum(0)
		{
		}
		
		size_t top() const
		{
			if(flagTopIsUptoDate)
				return topNum;
			if (visitedCount >= nodesValue_.size())
				throw std::out_of_range("NonVisitedNodesVector is empty.\n");
			size_t firstNotvisited = 0;
			while (isVisited_[firstNotvisited]) {
				++firstNotvisited;
			}
			T val = nodesValue_[firstNotvisited];
			topNum = firstNotvisited;
			auto size = nodesValue_.size();
			for (size_t i = firstNotvisited + 1; i < size; ++i) {
				if (!isVisited_[i] && lessFunctor_(nodesValue_[i], nodesValue_[topNum]))
					topNum = i;
			}
			flagTopIsUptoDate = true;
			return topNum;
		}
		
		void pop()
		{
			erase(top());
			flagTopIsUptoDate = false;
		}
		
		void erase(size_t nodeNum)
		{
			flagTopIsUptoDate = false;
			isVisited_[nodeNum] = 1;
			++visitedCount;
		};
		
		/// insert node to container
		void insert(size_t nodeNum)
		{
			flagTopIsUptoDate = false;
			if (isVisited_[nodeNum])
				--visitedCount;
			isVisited_[nodeNum] = 0;
		}
		
		bool empty() const
		{
			return visitedCount == isVisited_.size();
		};
		
		void updateNodeValue(size_t nodeNum, const T &value)
		{
			flagTopIsUptoDate = false;
			nodesValue_[nodeNum] = value;
		}
		
		double getValue(size_t nodeNum) const
		{
			return nodesValue_[nodeNum];
		}
	
	private:
		std::vector<char> isVisited_;
		std::vector<T> nodesValue_;
		size_t visitedCount;
		std::function<bool(const T&, const T&)> lessFunctor_;
		mutable bool flagTopIsUptoDate;
		mutable size_t topNum;
	};
	
	template<class T, class Compare>
	class NonVisitedNodesSet {
	public:
		/// maxNodeNum is unused parameter here
		/// lessFunctor is required to compare two nodes (the less one will be on top of container)
		NonVisitedNodesSet(const Compare &lessFunctor,
		                   size_t nodeCount) :
				notVisited_(
						[lessFunctor, this](auto nodeNum1, auto nodeNum2) {
							return lessFunctor(nodesValue_[nodeNum1], nodesValue_[nodeNum2]);
						}
				),
				nodesValue_(nodeCount, std::numeric_limits<T>::min())
		{
		};
		
		/// return node from the top
		size_t top() const
		{
			return *notVisited_.begin();
		};
		
		void pop()
		{
			notVisited_.erase(notVisited_.begin());
		};
		
		/// delete node from container
		void erase(size_t nodeNum)
		{
			notVisited_.erase(nodeNum);
		};
		
		/// insert node to container
		void insert(size_t nodeNum)
		{
			notVisited_.insert(nodeNum);
		}
		
		bool empty() const
		{
			return notVisited_.empty();
		};
		
		void updateNodeValue(size_t nodeNum, const T &value)
		{
			notVisited_.erase(nodeNum);
			nodesValue_[nodeNum] = value;
			notVisited_.insert(nodeNum);
		}
		
		T getValue(size_t nodeNum) const
		{
			return nodesValue_[nodeNum];
		}
	
	private:
		std::multiset<size_t, std::function<bool(size_t, size_t)>> notVisited_;
		std::vector<T> nodesValue_;
	};
	
	template <class T, class Compare>
	class NonVisitedNodesPrioQueue
	{
	public:
		NonVisitedNodesPrioQueue(const Compare &lessFunctor,
		                         size_t nodeCount):
				nodesValue_(nodeCount, std::numeric_limits<T>::min()),
				notVisited_(
						[this, lessFunctor](auto node1, auto node2){
							return !lessFunctor(node1.second, node2.second);}
				)
		{
		}
		
		size_t top() const
		{
			return notVisited_.top().first;
		}
		
		void pop()
		{
			notVisited_.pop();
		}
		
		bool empty() const
		{
			return notVisited_.empty();
		}
		
		void updateNodeValue(size_t nodeNum, const T &value)
		{
			nodesValue_[nodeNum] = value;
			notVisited_.push({nodeNum, value});
		}
		
		T getValue(size_t nodeNum) const
		{
			return nodesValue_[nodeNum];
		}
	
	private:
		using node_t = std::pair<size_t, T>; // pair - (nodeNum, value)
		using container_t = std::vector<std::pair<size_t, T>>;
		using compare_t = std::function<bool(const node_t&, const node_t&)>;
		std::priority_queue<size_t, container_t, compare_t> notVisited_;
		std::vector<T> nodesValue_;
	};
}

//////
///	Host class for Dijkstra algorithm
/// \tparam T   type of weights
/// \tparam NotVisitedNodesContainer    storage policy of unvisited nodes
/// \param weightedEdges    vector with weights of adj vertices
/// \param startNode    starting node number
/// \param finishNode   target node number
/// \param lessFunctor  compares node values
/// \return             value of target node
template<class T, class Compare, template<class, class> class NotVisitedNodesContainer = UnvisitedNodesStoragePolicy::NonVisitedNodesSet>
class AgileDijkstra{
public:
	static T Run(const graph_t &weightedEdges,
	             size_t startNode,
	             size_t finishNode,
	             const std::function<T(const T&, const T&)> &F,
	             const Compare &compare);
};

template<class T, class Compare, template<class, class> class NotVisitedNodesContainer>
T AgileDijkstra<T, Compare, NotVisitedNodesContainer> ::
Run(const graph_t &weightedEdges,
    size_t startNode,
    size_t finishNode,
    const std::function<T(const T&, const T&)> &F,
    const Compare &compare)
{
	size_t totalNodesCount = weightedEdges.size();
	NotVisitedNodesContainer<T, Compare> notVisitedNodes(
			compare,
			totalNodesCount
	);
	notVisitedNodes.updateNodeValue(startNode, std::numeric_limits<T>::max());
	while(not notVisitedNodes.empty()){
		auto v = notVisitedNodes.top();
		if(v == finishNode)
			break;
		notVisitedNodes.pop();
		for(const auto &adj: weightedEdges[v]){
			auto nodeTo = adj.first;
			if(notVisitedNodes.getValue(v) > notVisitedNodes.getValue(nodeTo)){
				auto weight = adj.second;
				auto guarantedWidth = F(notVisitedNodes.getValue(v), weight);
				/// update sorted struct
				if(compare(guarantedWidth,notVisitedNodes.getValue(nodeTo))){
					notVisitedNodes.updateNodeValue(nodeTo, guarantedWidth);
				}
			}
		}
	}
	return notVisitedNodes.getValue(finishNode);
}

using CompareDouble = std::function<bool(double, double)>;
using DijkstraWithSet = AgileDijkstra<double, CompareDouble, UnvisitedNodesStoragePolicy::NonVisitedNodesSet>;
using DijkstraWithVector = AgileDijkstra<double, CompareDouble, UnvisitedNodesStoragePolicy::NonVisitedNodesVector>;
using DijkstraWithPrioQueue = AgileDijkstra<double, CompareDouble, UnvisitedNodesStoragePolicy::NonVisitedNodesPrioQueue>;

#endif //HT_DIJKSTRA_H
