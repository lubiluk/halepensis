#pragma once

#include "entity.hpp"
#include "relation.hpp"
#include <string>


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <boost/graph/adjacency_list.hpp>
#include <boost/optional.hpp>
#pragma clang diagnostic pop

using SceneGraph = boost::adjacency_list
<boost::vecS, boost::vecS, boost::bidirectionalS, Entity, Relation>;
using VertexDesc = SceneGraph::vertex_descriptor;
using VertexIter = SceneGraph::vertex_iterator;
using EdgeIter = SceneGraph::edge_iterator;
using EdgeDesc = SceneGraph::edge_descriptor;
using AdjacencyIter = SceneGraph::adjacency_iterator;

auto find_vertex(const std::string& id, const SceneGraph& g) -> boost::optional<VertexDesc>;
