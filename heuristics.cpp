/*
 * Copyright 2020 Casey Sanchez
 */

#include "heuristics.hpp"

namespace Mandoline
{
    Transform::Transform(Polygon const &polygon, Eigen::Affine2d const &transform) : m_polygon(polygon), m_transform(transform)
    {
    }

    void Transform::Compute(Polygon &output)
    {
        std::vector<Eigen::Vector2d> out_vertices;

        std::vector<Eigen::Vector2d> const &polygon_vertices = m_polygon.Vertices();
        std::vector<Eigen::Array2i> const &polygon_edges = m_polygon.Edges();

        auto vertex_transform_operator = [transform = m_transform](Eigen::Vector2d const &vertex) -> Eigen::Vector2d {
            return transform * vertex;
        };

        std::transform(std::cbegin(polygon_vertices), std::cend(polygon_vertices), std::back_inserter(out_vertices), vertex_transform_operator);

        output = Polygon(out_vertices, polygon_edges);
    }

    Merge::Merge(std::vector<Polygon> const &polygons) : m_polygons(polygons)
    {
    }

    Merge::Merge(std::initializer_list<Polygon> const &polygons) : m_polygons(polygons)
    {
    }

    void Merge::Compute(Polygon &output)
    {
        std::vector<Eigen::Vector2d> out_vertices;
        std::vector<Eigen::Array2i> out_edges;

        for (Polygon const &polygon : m_polygons) {
            auto edge_transform_operator = [size = out_edges.size()](Eigen::Array2i const &edge) -> Eigen::Array2i {
                return Eigen::Array2i(edge[0] + size, edge[1] + size);
            };

            std::vector<Eigen::Vector2d> const &polygon_vertices = polygon.Vertices();
            std::vector<Eigen::Array2i> const &polygon_edges = polygon.Edges();

            std::copy(std::cbegin(polygon_vertices), std::cend(polygon_vertices), std::back_inserter(out_vertices));
            std::transform(std::cbegin(polygon_edges), std::cend(polygon_edges), std::back_inserter(out_edges), edge_transform_operator);
        }

        output = Polygon(out_vertices, out_edges);
    }

    Extrude::Extrude(Polygon const &polygon, double const &distance) : m_polygon(polygon), m_distance(distance)
    {
    }

    void Extrude::Compute(Polygon &output)
    {
        auto edge_pair_comparator = [](std::pair<Eigen::Array2i, Eigen::Array2i> const &lhs, std::pair<Eigen::Array2i, Eigen::Array2i> const &rhs) -> bool {
            auto edge_hash = [](Eigen::Array2i const &edge) -> int32_t {
                  return 73856093 * edge[0] ^ 19349663 * edge[1];
            };

            return (edge_hash(lhs.first) ^ edge_hash(lhs.second)) < (edge_hash(rhs.first) ^ edge_hash(rhs.second));
        };

        // Create a set of pairs of neighboring edges
        std::set<std::pair<Eigen::Array2i, Eigen::Array2i>, decltype(edge_pair_comparator)> edge_pairs(edge_pair_comparator);

        std::vector<Eigen::Array2i> const &polygon_edges = m_polygon.Edges();

        for (auto iter_a = std::cbegin(polygon_edges); iter_a != std::cend(polygon_edges); ++iter_a) {
            for (auto iter_b = std::next(iter_a); iter_b != std::cend(polygon_edges); ++iter_b) {
                Eigen::Array2i const &edge_a = *iter_a;
                Eigen::Array2i const &edge_b = *iter_b;

                // If any two indices are shared between both edges insert their pair into edge_pairs
                if (edge_a[0] == edge_b[0] || edge_a[1] == edge_b[1] || edge_a[0] == edge_b[1] || edge_a[1] == edge_b[0]) {
                    edge_pairs.insert({ edge_a, edge_b });
                }
            }
        }

        // Of the edge pairs, move their shared vertex to the intersection of the extruded edges
        std::vector<Eigen::Vector2d> polygon_vertices(m_polygon.Vertices().size());

        for (auto const &[edge_a, edge_b] : edge_pairs) {
            std::array<Eigen::Vector2d, 2> const &segment_a = m_polygon.Segment(edge_a);
            std::array<Eigen::Vector2d, 2> const &segment_b = m_polygon.Segment(edge_b);

            Eigen::Vector2d const normal_a = m_polygon.Line(edge_a).normal();
            Eigen::Vector2d const normal_b = m_polygon.Line(edge_b).normal();

            std::array<Eigen::Vector2d, 2> const new_segment_a = { segment_a[0] + normal_a * m_distance, segment_a[1] + normal_a * m_distance };
            std::array<Eigen::Vector2d, 2> const new_segment_b = { segment_b[0] + normal_b * m_distance, segment_b[1] + normal_b * m_distance };

            Eigen::Hyperplane<double, 2> const line_a = Eigen::Hyperplane<double, 2>::Through(new_segment_a[0], new_segment_a[1]);
            Eigen::Hyperplane<double, 2> const line_b = Eigen::Hyperplane<double, 2>::Through(new_segment_b[0], new_segment_b[1]);

            Eigen::Vector2d const intersection = line_a.intersection(line_b);

            // Move the shared vertex to `intersection`
            if ((edge_a[0] == edge_b[0]) || (edge_a[0] == edge_b[1])) {
                polygon_vertices.at(edge_a[0]) = intersection;
            }
            else if ((edge_a[1] == edge_b[1]) || (edge_a[1] == edge_b[0])) {
                polygon_vertices.at(edge_a[1]) = intersection;
            }
        }

        output = Polygon(polygon_vertices, polygon_edges);
    }

    Invert::Invert(Polygon const &polygon) : m_polygon(polygon)
    {
    }

    void Invert::Compute(Polygon &output)
    {
        std::vector<Eigen::Vector2d> polygon_vertices = m_polygon.Vertices();
        std::vector<Eigen::Array2i> polygon_edges = m_polygon.Edges();

        std::reverse(std::begin(polygon_vertices), std::end(polygon_vertices));

        output = Polygon(polygon_vertices, polygon_edges);
    }
    
    Slice::Slice(Polygon const &polygon, double const &spacing) : m_polygon(polygon), m_spacing(spacing)
    {
    }

    void Slice::Compute(Graph &output)
    {
        std::vector<Eigen::Vector2d> out_vertices;
        std::vector<Eigen::Array2i> out_edges;

        Segment(out_vertices, out_edges);
        Connect(out_vertices, out_edges);

        output = Graph(out_vertices, out_edges);
    }

    void Slice::Segment(std::vector<Eigen::Vector2d> &out_vertices, std::vector<Eigen::Array2i> &out_edges)
    {
        // Scanline segmentation
        // Input:            Output:
        // 0------------1     + 0  2  4  6 +
        // |            |       |  |  |  |
        // |            |       |  |  |  |
        // |            |       |  |  |  |
        // |            |       |  |  |  |
        // 3------------2     + 1  3  5  7 +
        std::vector<Eigen::Array2i> const &polygon_edges = m_polygon.Edges();

        Eigen::Vector2d const &polygon_min = m_polygon.Min();
        Eigen::Vector2d const &polygon_max = m_polygon.Max();

        // Intersect each spaced vertical line with each of the polygon's edges,
        // only add the intersection to the vertex list if it falls within constraints of the line segment
        for (double x = polygon_min[0]; x <= polygon_max[0]; x += m_spacing) {
            int32_t const back_index = out_vertices.size();

            Eigen::Hyperplane<double, 2> const cast_line = Eigen::Hyperplane<double, 2>::Through(Eigen::Vector2d(x, polygon_min[1]), Eigen::Vector2d(x, polygon_max[1]));

            for (Eigen::Array2i const &edge : polygon_edges) {
                Eigen::Hyperplane<double, 2> const edge_line = m_polygon.Line(edge);

                Eigen::Vector2d const intersection = cast_line.intersection(edge_line);

                if (m_polygon.IsPointOnEdge(intersection, edge)) {
                    out_vertices.push_back(intersection);
                }
            }

            auto vertex_comparator = [](Eigen::Vector2d const &vertex_a, Eigen::Vector2d const &vertex_b) -> bool {
                return vertex_a.y() < vertex_b.y();
            };

            // Sort by y-value to guarantee that only direct vertical vertex neighbors
            // are used to properly construct edges.
            std::sort(std::next(std::begin(out_vertices), back_index), std::end(out_vertices), vertex_comparator);

            // Create edges from the new vertices.
            // Only create an edge if it is within the polygon.
            // Edges are created such that the y-coordinate of the vertex at index 0
            // is always less than the y-coordinate at index 1, due to the sort above. 
            for (int32_t index = back_index, size = out_vertices.size(); index < size - 1; ++index) {
                Eigen::Vector2d const &vertex_a = out_vertices.at(index);
                Eigen::Vector2d const &vertex_b = out_vertices.at(index + 1);

                Eigen::Vector2d const midpoint = (vertex_a + vertex_b) * 0.5;

                if (m_polygon.IsPointInside(midpoint)) {
                    out_edges.emplace_back(index, index + 1);
                }
            }
        }
    }

    void Slice::Connect(std::vector<Eigen::Vector2d> &out_vertices, std::vector<Eigen::Array2i> &out_edges)
    {
        std::vector<Eigen::Array2i> const polygon_edges = m_polygon.Edges();

        std::vector<Eigen::Array2i> slice_edges = out_edges;

        out_edges.clear();

        // `lower_upper` specifies which of the two edge indices we are referring to, 
        //  either index 0 or index 1.
        std::array<int32_t, 2> lower_upper { 0, 0 };

        auto slice_edge_it = std::begin(slice_edges);

        while (slice_edge_it != std::end(slice_edges)) {
            // Get the polygon edge on which the vertex lie
            auto polygon_edge_it = std::find_if(std::cbegin(polygon_edges), std::cend(polygon_edges), [vertex=out_vertices.at((*slice_edge_it)[lower_upper[0]]), polygon=m_polygon](Eigen::Array2i const &edge) { return polygon.IsPointOnEdge(vertex, edge); });

            if (polygon_edge_it != std::cend(polygon_edges)) {
                // If there are vertices still present along the polygon's edge, find the closest
                auto closest_edge_it = std::end(slice_edges);

                double minimum_distance = std::numeric_limits<double>::infinity();

                for (auto other_edge_it = std::begin(slice_edges); other_edge_it != std::end(slice_edges); ++other_edge_it) {
                    if (other_edge_it != slice_edge_it && m_polygon.IsPointOnEdge(out_vertices.at((*other_edge_it)[lower_upper[1]]), *polygon_edge_it)) {
                        Eigen::Vector2d vertex_to = out_vertices.at((*other_edge_it)[lower_upper[1]]);
                        Eigen::Vector2d vertex_from = out_vertices.at((*slice_edge_it)[lower_upper[0]]);

                        double distance = (vertex_to - vertex_from).norm();

                        if (distance < minimum_distance) {
                            closest_edge_it = other_edge_it;

                            minimum_distance = distance;
                        }
                    }
                }

                // If we did not find a vertex along the polygon's edge, find the closest of all remaining vertices
                if (closest_edge_it == std::end(slice_edges)) {
                    minimum_distance = std::numeric_limits<double>::infinity();

                    for (auto other_edge_it = std::begin(slice_edges); other_edge_it != std::end(slice_edges); ++other_edge_it) {
                        if (other_edge_it != slice_edge_it) {
                            Eigen::Vector2d vertex_from = out_vertices.at((*slice_edge_it)[lower_upper[0]]);
                            
                            // Check if the vertex at index 0 of `other_edge_it` is the closest
                            {
                                Eigen::Vector2d vertex_to = out_vertices.at((*other_edge_it)[0]);

                                double distance = (vertex_to - vertex_from).norm();

                                if (distance < minimum_distance) {
                                    closest_edge_it = other_edge_it;

                                    minimum_distance = distance;

                                    lower_upper[1] = 0;
                                }
                            } 

                            // Also check if the vertex at index 1 of `other_edge_it` is the closest
                            {
                                Eigen::Vector2d vertex_to = out_vertices.at((*other_edge_it)[1]);

                                double distance = (vertex_to - vertex_from).norm();

                                if (distance < minimum_distance) {
                                    closest_edge_it = other_edge_it;

                                    minimum_distance = distance;

                                    lower_upper[1] = 1;
                                }
                            }
                        }
                    }
                }

                if (closest_edge_it != std::end(slice_edges)) {
                    // Add the slice edge to `out_edges`
                    out_edges.push_back(*slice_edge_it);

                    // Connect and add the relevant vertices of `slice_edge_it` and `closest_edge_it`
                    out_edges.emplace_back((*slice_edge_it)[lower_upper[0]], (*closest_edge_it)[lower_upper[1]]);

                    // Erase the iterator from `slice_edges` and increment `closest_edge_it` if necessary
                    std::ptrdiff_t distance = std::distance(slice_edge_it, closest_edge_it) - 1;

                    slice_edge_it = slice_edges.erase(slice_edge_it);

                    if (distance >= 0) {
                        closest_edge_it = std::next(slice_edge_it, distance);
                    }

                    if (lower_upper[1] == 0) {
                        // Connect upper edge indices
                        lower_upper = { 1, 1 };
                    }
                    else {
                        // Connect lower edge indices
                        lower_upper = { 0, 0 };
                    }
                }

                slice_edge_it = closest_edge_it;
            }
            else {
                throw std::runtime_error("Vertex is not associated with an edge on the polygon");
            }
        }

        // There will always remain 1 edge that does not get added to `out_edges` so we add it here
        out_edges.push_back(slice_edges.front());
    }
}