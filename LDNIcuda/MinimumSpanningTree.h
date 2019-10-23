#pragma once
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MINIMUMSPANNINGTREE_H
#define MINIMUMSPANNINGTREE_H

#include <vector>

#include <unordered_set>
#include <unordered_map>
#include "IntpointIrfan.h"

	/*!
	 * \brief Implements Prim's algorithm to compute Minimum Spanning Trees (MST).
	 *
	 * The minimum spanning tree is always computed from a clique of vertices.
	 */
	class MinimumSpanningTree
	{
		/*!
		 * \brief Represents an edge of the tree.
		 *
		 * While edges are meant to be undirected, these do have a start and end
		 * point.
		 */
		struct Edge {
			/**
			 * The point at which this edge starts.
			 */
			const curaIrfan::PointIrfan start;

			/**
			 * The point at which this edge ends.
			 */
			const curaIrfan::PointIrfan end;
		};
	public:
		MinimumSpanningTree() = default;
		/*!
		 * \brief Constructs a minimum spanning tree that spans all given vertices.
		 */
		MinimumSpanningTree(std::unordered_set<curaIrfan::PointIrfan> vertices);

		/*!
		 * \brief Gets the nodes that are adjacent to the specified node.
		 * \return A list of nodes that are adjacent.
		 */
	std::vector<curaIrfan::PointIrfan> adjacentNodes(curaIrfan::PointIrfan node) const;

		/*!
		 * \brief Gets the leaves of the tree.
		 * \return A list of nodes that are all leaves of the tree.
	/	 */
		std::vector<curaIrfan::PointIrfan> leaves() const;

		/*!
		 * \brief Gets all vertices of the tree.
		 * \return A list of vertices of the tree.
		 */
	std::vector<curaIrfan::PointIrfan> vertices() const;

	private:
		using AdjacencyGraph_t = std::unordered_map<curaIrfan::PointIrfan, std::vector<Edge>>;
		AdjacencyGraph_t adjacency_graph;

		/*!
		 * \brief Computes the edges of a minimum spanning tree using Prim's
		 * algorithm.
		 *
		 * \param vertices The vertices to span.
		 * \return An adjacency graph with for each point one or more edges.
		 */
		AdjacencyGraph_t prim(std::unordered_set<curaIrfan::PointIrfan> vertices) const;
	};



#endif /* MINIMUMSPANNINGTREE_H */


