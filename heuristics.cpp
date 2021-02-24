/*
 * Copyright 2020 Casey Sanchez
 */

#include "heuristics.hpp"

namespace Mandoline
{
    Path::Path(std::vector<Eigen::Vector2d> const &vertices) : m_vertices(vertices)
    {
    }

    Path::Path(std::initializer_list<Eigen::Vector2d> const &vertices) : Path(std::vector<Eigen::Vector2d>{ vertices })
    {
    }

    void Path::Compute(Graph &output)
    {
        std::vector<Eigen::Array2i> output_edges(m_vertices.size() - 1);

        std::generate(std::begin(output_edges), std::end(output_edges), [index=0]() mutable -> Eigen::Array2i { 
            Eigen::Array2i edge(index, index + 1); 
            
            index++; 

            return edge; 
        });

        output = Graph(m_vertices, output_edges);
    }
    
    Polygon::Polygon(std::vector<Eigen::Vector2d> const &vertices) : m_vertices(vertices)
    {
    }

    Polygon::Polygon(std::initializer_list<Eigen::Vector2d> const &vertices) : Polygon(std::vector<Eigen::Vector2d>{ vertices })
    {
    }

    void Polygon::Compute(Graph &output)
    {
        std::vector<Eigen::Array2i> output_edges(m_vertices.size() - 1);

        std::generate(std::begin(output_edges), std::end(output_edges), [index=0]() mutable -> Eigen::Array2i { 
            Eigen::Array2i edge(index, index + 1); 
            
            index++; 

            return edge; 
        });

        output_edges.emplace_back(m_vertices.size() - 1, 0);

        output = Graph(m_vertices, output_edges);
    }

    Regular::Regular(uint32_t const &sides, double const &radius) : m_sides(sides), m_radius(radius)
    {
    }

    void Regular::Compute(Graph &output)
    {
        if (m_sides < 3) {
            throw std::domain_error("Sides < 3");
        }

        std::vector<Eigen::Vector2d> output_vertices;

        double const tau = 8.0 * std::atan(1.0);
        double const step = tau / static_cast<double>(m_sides);

        for (uint32_t side = 0; side < m_sides; ++side) {
            double theta = step * (static_cast<double>(side) + 0.5);

            output_vertices.emplace_back(std::cos(-theta) * m_radius, std::sin(-theta) * m_radius);
        }

        Polygon(output_vertices).Compute(output);
    }

    Bezier::Bezier(std::vector<Eigen::Vector2d> const &vertices, double const &bezier_gain) : m_vertices(vertices), m_bezier_gain(bezier_gain)
    {
    }

    void Bezier::Compute(Graph &output)
    {
        if (m_bezier_gain < 1.0) {
            throw std::invalid_argument("Bezier gain must be >= 1");
        }

        std::vector<Eigen::Vector2d> output_vertices;

        double const step = 1.0 / (static_cast<double>(m_vertices.size()) * m_bezier_gain - 1.0);

        for (double interpolate = 0.0; interpolate < 1.0; interpolate += step) {
            Eigen::Vector2d output_vertex = Eigen::Vector2d::Zero();

            for (size_t j = 0; j < m_vertices.size(); ++j) {
                double const binomial_coefficient = static_cast<double>(Factorial(m_vertices.size() - 1)) / static_cast<double>(Factorial(j) * Factorial(m_vertices.size() - 1 - j));
                double const basis = std::pow(1.0 - interpolate, static_cast<double>(m_vertices.size() - 1 - j)) * std::pow(interpolate, static_cast<double>(j)) * binomial_coefficient;

                output_vertex += basis * m_vertices[j];
            }

            output_vertices.push_back(output_vertex);
        }

        Path(output_vertices).Compute(output);
    }

    // TODO: Max of 20!, need a more efficient method for calculating binomial coefficients
    uint64_t Bezier::Factorial(uint64_t const &n)
    {
        if (n == 0) {
            return 1;
        }

        uint64_t factorial = n;
        
        for (uint64_t factor = n - 1; factor > 0; --factor) {
            factorial *= factor; 
        }

        return factorial;
    }

    Transform::Transform(Graph const &graph, Eigen::Affine2d const &transform) : m_graph(graph), m_transform(transform)
    {
    }

    void Transform::Compute(Graph &output)
    {
        std::vector<Eigen::Vector2d> output_vertices;

        std::vector<Eigen::Vector2d> const &graph_vertices = m_graph.Vertices();
        std::vector<Eigen::Array2i> const &graph_edges = m_graph.Edges();

        auto vertex_transform_operator = [transform = m_transform](Eigen::Vector2d const &vertex) -> Eigen::Vector2d {
            return transform * vertex;
        };

        std::transform(std::cbegin(graph_vertices), std::cend(graph_vertices), std::back_inserter(output_vertices), vertex_transform_operator);

        output = Graph(output_vertices, graph_edges);
    }

    Merge::Merge(Graph const &graph_lhs, Graph const &graph_rhs) : Merge({ graph_lhs, graph_rhs })
    {
    }

    Merge::Merge(std::initializer_list<Graph> const &graphs) : m_graphs(graphs)
    {
    }

    Merge::Merge(std::vector<Graph> const &graphs) : m_graphs(graphs)
    {
    }

    void Merge::Compute(Graph &output)
    {
        std::vector<Eigen::Vector2d> output_vertices;
        std::vector<Eigen::Array2i> output_edges;

        for (Graph const &graph : m_graphs) {
            auto edge_transform_operator = [size = output_vertices.size()](Eigen::Array2i const &edge) -> Eigen::Array2i {
                return Eigen::Array2i(edge[0] + size, edge[1] + size);
            };
             
            std::vector<Eigen::Vector2d> const &graph_vertices = graph.Vertices(); 
            std::vector<Eigen::Array2i> const &graph_edges = graph.Edges();

            std::copy(std::cbegin(graph_vertices), std::cend(graph_vertices), std::back_inserter(output_vertices));
            std::transform(std::cbegin(graph_edges), std::cend(graph_edges), std::back_inserter(output_edges), edge_transform_operator);
        }

        output = Graph(output_vertices, output_edges);
    }

    Invert::Invert(Graph const &graph) : m_graph(graph)
    {
    }

    void Invert::Compute(Graph &output)
    {
        std::vector<Eigen::Vector2d> graph_vertices = m_graph.Vertices();
        std::vector<Eigen::Array2i> graph_edges = m_graph.Edges();

        std::reverse(std::begin(graph_vertices), std::end(graph_vertices));

        output = Graph(graph_vertices, graph_edges);
    }

    Extrude::Extrude(Graph const &graph, double const &distance) : m_graph(graph), m_distance(distance)
    {
    }

    void Extrude::Compute(Graph &output)
    {
        auto edge_pair_comparator = [](std::pair<Eigen::Array2i, Eigen::Array2i> const &lhs, std::pair<Eigen::Array2i, Eigen::Array2i> const &rhs) -> bool {
            auto edge_hash = [](Eigen::Array2i const &edge) -> int32_t {
                  return 73856093 * edge[0] ^ 19349663 * edge[1];
            };

            return (edge_hash(lhs.first) ^ edge_hash(lhs.second)) < (edge_hash(rhs.first) ^ edge_hash(rhs.second));
        };

        // Create a set of pairs of neighboring edges
        std::set<std::pair<Eigen::Array2i, Eigen::Array2i>, decltype(edge_pair_comparator)> edge_pairs(edge_pair_comparator);

        std::vector<Eigen::Array2i> const &graph_edges = m_graph.Edges();

        for (auto iter_a = std::cbegin(graph_edges); iter_a != std::cend(graph_edges); ++iter_a) {
            for (auto iter_b = std::next(iter_a); iter_b != std::cend(graph_edges); ++iter_b) {
                Eigen::Array2i const &edge_a = *iter_a;
                Eigen::Array2i const &edge_b = *iter_b;

                // If any two indices are shared between both edges insert their pair into edge_pairs
                if (edge_a[0] == edge_b[0] || edge_a[1] == edge_b[1] || edge_a[0] == edge_b[1] || edge_a[1] == edge_b[0]) {
                    edge_pairs.insert({ edge_a, edge_b });
                }
            }
        }

        // Of the edge pairs, move their shared vertex to the intersection of the extruded edges
        std::vector<Eigen::Vector2d> output_vertices(m_graph.Vertices().size());

        for (auto const &[edge_a, edge_b] : edge_pairs) {
            std::array<Eigen::Vector2d, 2> const &segment_a = m_graph.Segment(edge_a);
            std::array<Eigen::Vector2d, 2> const &segment_b = m_graph.Segment(edge_b);

            Eigen::Vector2d const normal_a = m_graph.Line(edge_a).normal();
            Eigen::Vector2d const normal_b = m_graph.Line(edge_b).normal();

            std::array<Eigen::Vector2d, 2> const new_segment_a = { segment_a[0] + normal_a * m_distance, segment_a[1] + normal_a * m_distance };
            std::array<Eigen::Vector2d, 2> const new_segment_b = { segment_b[0] + normal_b * m_distance, segment_b[1] + normal_b * m_distance };

            Eigen::Hyperplane<double, 2> const line_a = Eigen::Hyperplane<double, 2>::Through(new_segment_a[0], new_segment_a[1]);
            Eigen::Hyperplane<double, 2> const line_b = Eigen::Hyperplane<double, 2>::Through(new_segment_b[0], new_segment_b[1]);

            Eigen::Vector2d const intersection = line_a.intersection(line_b);

            // Move the shared vertex to `intersection`
            if ((edge_a[0] == edge_b[0]) || (edge_a[0] == edge_b[1])) {
                output_vertices.at(edge_a[0]) = intersection;
            }
            else if ((edge_a[1] == edge_b[1]) || (edge_a[1] == edge_b[0])) {
                output_vertices.at(edge_a[1]) = intersection;
            }
        }

        output = Graph(output_vertices, graph_edges);
    }
    
    Slice::Slice(Graph const &graph, double const &spacing) : m_graph(graph), m_spacing(spacing)
    {
    }

    void Slice::Compute(Graph &output)
    {
        std::vector<Eigen::Vector2d> output_vertices;
        std::vector<Eigen::Array2i> output_edges;

        Segment(output_vertices, output_edges);
        Connect(output_vertices, output_edges);

        output = Graph(output_vertices, output_edges);
    }

    void Slice::Segment(std::vector<Eigen::Vector2d> &output_vertices, std::vector<Eigen::Array2i> &output_edges)
    {
        // Scanline segmentation
        // Input:            Output:
        // 0------------1     + 0  2  4  6 +
        // |            |       |  |  |  |
        // |            |       |  |  |  |
        // |            |       |  |  |  |
        // |            |       |  |  |  |
        // 3------------2     + 1  3  5  7 +
        std::vector<Eigen::Array2i> const &graph_edges = m_graph.Edges();

        Eigen::Vector2d const &graph_min = m_graph.Min();
        Eigen::Vector2d const &graph_max = m_graph.Max();

        // Intersect each spaced vertical line with each of the graph's edges,
        // only add the intersection to the vertex list if it falls within constraints of the line segment
        for (double x = graph_min[0]; x <= graph_max[0]; x += m_spacing) {
            size_t const back_index = output_vertices.size();

            Eigen::Hyperplane<double, 2> const cast_line = Eigen::Hyperplane<double, 2>::Through(Eigen::Vector2d(x, graph_min[1]), Eigen::Vector2d(x, graph_max[1]));

            for (Eigen::Array2i const &edge : graph_edges) {
                Eigen::Hyperplane<double, 2> const edge_line = m_graph.Line(edge);

                Eigen::Vector2d const intersection = cast_line.intersection(edge_line);

                if (m_graph.IsPointOnEdge(intersection, edge)) {
                    output_vertices.push_back(intersection);
                }
            }

            auto vertex_comparator = [](Eigen::Vector2d const &vertex_a, Eigen::Vector2d const &vertex_b) -> bool {
                return vertex_a.y() < vertex_b.y();
            };

            // Sort by y-value to guarantee that only direct vertical vertex neighbors
            // are used to properly construct edges.
            std::sort(std::next(std::begin(output_vertices), back_index), std::end(output_vertices), vertex_comparator);

            // Create edges from the new vertices.
            // Only create an edge if it is within the graph.
            // Edges are created such that the y-coordinate of the vertex at index 0
            // is always less than the y-coordinate at index 1, due to the sort above. 
            for (int32_t index = back_index, size = output_vertices.size(); index < size - 1; ++index) {
                Eigen::Vector2d const &vertex_a = output_vertices.at(index);
                Eigen::Vector2d const &vertex_b = output_vertices.at(index + 1);

                Eigen::Vector2d const midpoint = (vertex_a + vertex_b) * 0.5;

                if (m_graph.IsPointInside(midpoint)) {
                    output_edges.emplace_back(index, index + 1);
                }
            }
        }
    }

    void Slice::Connect(std::vector<Eigen::Vector2d> &output_vertices, std::vector<Eigen::Array2i> &output_edges)
    {
        // Connecting segmented lines
        // Input:              Output:
        // + 0  2  4  6 +   + 0  2--4  6 +
        //   |  |  |  |       |  |  |  |
        //   |  |  |  |       |  |  |  |
        //   |  |  |  |       |  |  |  |
        //   |  |  |  |       |  |  |  |
        // + 1  3  5  7 +   + 1--3  5--7 +
        std::vector<Eigen::Array2i> const graph_edges = m_graph.Edges();

        std::vector<Eigen::Array2i> slice_edges = output_edges;

        output_edges.clear();

        // `edge_index` specifies which of the two edge indices we are referring to, 
        //  either index 0 or index 1.
        std::array<int32_t, 2> edge_index { 0, 0 };

        auto slice_edge_it = std::begin(slice_edges);

        while (slice_edge_it != std::end(slice_edges)) {
            // Get the graph edge on which the vertex lay
            auto graph_edge_it = std::find_if(std::cbegin(graph_edges), std::cend(graph_edges), [vertex=output_vertices.at((*slice_edge_it)[edge_index[0]]), graph=m_graph](Eigen::Array2i const &edge) { return graph.IsPointOnEdge(vertex, edge); });

            if (graph_edge_it != std::cend(graph_edges)) {
                // If there are vertices still present along the current edge, find the closest
                auto closest_edge_it = std::end(slice_edges);

                double minimum_distance = std::numeric_limits<double>::infinity();

                for (auto other_edge_it = std::begin(slice_edges); other_edge_it != std::end(slice_edges); ++other_edge_it) {
                    if (other_edge_it != slice_edge_it && m_graph.IsPointOnEdge(output_vertices.at((*other_edge_it)[edge_index[1]]), *graph_edge_it)) {
                        Eigen::Vector2d vertex_to = output_vertices.at((*other_edge_it)[edge_index[1]]);
                        Eigen::Vector2d vertex_from = output_vertices.at((*slice_edge_it)[edge_index[0]]);

                        double distance = (vertex_to - vertex_from).norm();

                        if (distance < minimum_distance) {
                            closest_edge_it = other_edge_it;

                            minimum_distance = distance;
                        }
                    }
                }

                // If we did not find a vertex along the graph's edge, find the closest of all remaining vertices
                if (closest_edge_it == std::end(slice_edges)) {
                    minimum_distance = std::numeric_limits<double>::infinity();

                    for (auto other_edge_it = std::begin(slice_edges); other_edge_it != std::end(slice_edges); ++other_edge_it) {
                        if (other_edge_it != slice_edge_it) {
                            Eigen::Vector2d vertex_from = output_vertices.at((*slice_edge_it)[edge_index[0]]);
                            
                            // Check if the vertex at index 0 of `other_edge_it` is the closest
                            {
                                Eigen::Vector2d vertex_to = output_vertices.at((*other_edge_it)[0]);

                                double distance = (vertex_to - vertex_from).norm();

                                if (distance < minimum_distance) {
                                    closest_edge_it = other_edge_it;

                                    minimum_distance = distance;

                                    edge_index[1] = 0;
                                }
                            } 

                            // Also check if the vertex at index 1 of `other_edge_it` is the closest
                            {
                                Eigen::Vector2d vertex_to = output_vertices.at((*other_edge_it)[1]);

                                double distance = (vertex_to - vertex_from).norm();

                                if (distance < minimum_distance) {
                                    closest_edge_it = other_edge_it;

                                    minimum_distance = distance;

                                    edge_index[1] = 1;
                                }
                            }
                        }
                    }
                }

                if (closest_edge_it != std::end(slice_edges)) {
                    // Add the slice edge to `output_edges`
                    output_edges.push_back(*slice_edge_it);

                    // Connect and add the relevant vertices of `slice_edge_it` and `closest_edge_it`
                    output_edges.emplace_back((*slice_edge_it)[edge_index[0]], (*closest_edge_it)[edge_index[1]]);

                    // Erase the iterator from `slice_edges` and increment `closest_edge_it` if necessary
                    std::ptrdiff_t distance = std::distance(slice_edge_it, closest_edge_it) - 1;

                    slice_edge_it = slice_edges.erase(slice_edge_it);

                    if (distance >= 0) {
                        closest_edge_it = std::next(slice_edge_it, distance);
                    }

                    if (edge_index[1] == 0) {
                        // Connect upper edge indices
                        edge_index = { 1, 1 };
                    }
                    else {
                        // Connect lower edge indices
                        edge_index = { 0, 0 };
                    }
                }

                slice_edge_it = closest_edge_it;
            }
            else {
                throw std::runtime_error("Vertex is not associated with an edge on the graph");
            }
        }

        // There will always remain 1 edge that does not get added to `output_edges` so we add it here
        output_edges.push_back(slice_edges.front());
    }

    Fragment::Fragment(Graph const &graph_lhs, Graph const &graph_rhs) : m_graph_lhs(graph_lhs), m_graph_rhs(graph_rhs)
    {
    }

    void Fragment::Compute(Graph &output)
    {
        auto edge_comparator = [](Eigen::Array2i const &lhs, Eigen::Array2i const &rhs) -> bool {
            auto edge_hash = [](Eigen::Array2i const &edge) -> int32_t {
                return 73856093 * edge[0] ^ 19349663 * edge[1];
            };

            return edge_hash(lhs) < edge_hash(rhs);
        };

        std::vector<Eigen::Vector2d> output_vertices = m_graph_lhs.Vertices();

        std::vector<Eigen::Array2i> insert_edges = m_graph_lhs.Edges();
        std::vector<Eigen::Array2i> remove_edges;

        std::vector<Eigen::Array2i> const &graph_lhs_edges = m_graph_lhs.Edges();
        std::vector<Eigen::Array2i> const &graph_rhs_edges = m_graph_rhs.Edges();

        for (Eigen::Array2i const &graph_lhs_edge : graph_lhs_edges) {
            Eigen::Hyperplane<double, 2> const graph_lhs_line = m_graph_lhs.Line(graph_lhs_edge);

            for (Eigen::Array2i const &graph_rhs_edge : graph_rhs_edges) {
                Eigen::Hyperplane<double, 2> const graph_rhs_line = m_graph_rhs.Line(graph_rhs_edge);

                Eigen::Vector2d const intersection = graph_lhs_line.intersection(graph_rhs_line);

                if (m_graph_lhs.IsPointOnEdge(intersection, graph_lhs_edge) && m_graph_rhs.IsPointOnEdge(intersection, graph_rhs_edge)) {
                    size_t index = output_vertices.size();

                    output_vertices.push_back(intersection);

                    insert_edges.emplace_back(index, graph_lhs_edge[0]);
                    insert_edges.emplace_back(index, graph_lhs_edge[1]);

                    remove_edges.push_back(graph_lhs_edge);
                }
            }
        }
        
        std::sort(std::begin(insert_edges), std::end(insert_edges), edge_comparator);
        std::sort(std::begin(remove_edges), std::end(remove_edges), edge_comparator);

        std::vector<Eigen::Array2i> output_edges;

        std::set_difference(std::cbegin(insert_edges), std::cend(insert_edges), std::cbegin(remove_edges), std::cend(remove_edges), std::back_inserter(output_edges), edge_comparator);
        
        output = Graph(output_vertices, output_edges);
    }

    Difference::Difference(Graph const &graph_lhs, Graph const &graph_rhs) : m_graph_lhs(graph_lhs), m_graph_rhs(graph_rhs)
    {
    }
    
    void Difference::Compute(Graph &output)
    {
        std::vector<Eigen::Vector2d> output_vertices = m_graph_lhs.Vertices();
        std::vector<Eigen::Array2i> output_edges;

        std::vector<Eigen::Vector2d> const &graph_lhs_vertices = m_graph_lhs.Vertices();

        std::vector<Eigen::Array2i> const &graph_lhs_edges = m_graph_lhs.Edges();
        std::vector<Eigen::Array2i> const &graph_rhs_edges = m_graph_rhs.Edges();

        for (Eigen::Array2i const &graph_lhs_edge : graph_lhs_edges) {
            if (!m_graph_rhs.IsPointInside((graph_lhs_vertices[graph_lhs_edge[0]] + graph_lhs_vertices[graph_lhs_edge[1]]) * 0.5)) {
                output_edges.emplace_back(graph_lhs_edge[0], graph_lhs_edge[1]);
            }
        }

        output = Graph(output_vertices, output_edges);
    }

    Union::Union(Graph const &graph_lhs, Graph const &graph_rhs) : m_graph_lhs(graph_lhs), m_graph_rhs(graph_rhs)
    {
    }

    void Union::Compute(Graph &output)
    {
        Graph fragment_lhs;
        Graph fragment_rhs;

        Fragment(m_graph_lhs, m_graph_rhs).Compute(fragment_lhs);
        Fragment(m_graph_rhs, m_graph_lhs).Compute(fragment_rhs);

        Graph difference_lhs;
        Graph difference_rhs;

        Difference(fragment_lhs, fragment_rhs).Compute(difference_lhs);
        Difference(fragment_rhs, fragment_lhs).Compute(difference_rhs);

        Merge(difference_lhs, difference_rhs).Compute(output);
    }
}