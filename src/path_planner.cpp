#include "/home/daniel/path_planner/src/path_planner.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

PathPlannerGraph::PathPlannerGraph(Point start, Point goal, std::vector<Obstacle> obstacles,
                                 int points_per_obstacle, double ellipse_factor)
    : start_(start), goal_(goal), obstacles_(obstacles),
      points_per_obstacle_(points_per_obstacle), ellipse_factor_(ellipse_factor) {}

// Distance Between Two Points
double PathPlannerGraph::distance(const Point& p1, const Point& p2) const {
    auto [x1, y1] = p1;
    auto [x2, y2] = p2;
    return std::hypot(x2 - x1, y2 - y1);
}

// Check if The Point is in The Ellipse
bool PathPlannerGraph::is_point_in_ellipse(const Point& point) const {
    double sum_dist = distance(point, start_) + distance(point, goal_);
    double max_dist = ellipse_factor_ * distance(start_, goal_);
    return sum_dist <= max_dist;
}

// Check if the Edge Intersect with an Obstacle
bool PathPlannerGraph::edge_intersects_obstacles(const Point& p1, const Point& p2) const {
    auto [x1, y1] = p1;
    auto [x2, y2] = p2;
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    double line_length_sq = dx*dx + dy*dy;
    
    for (const auto& [ox, oy, r] : obstacles_) {
        if (line_length_sq == 0) continue;
        
        double t = ((ox - x1) * dx + (oy - y1) * dy) / line_length_sq;
        t = std::clamp(t, 0.0, 1.0);
        
        double closest_x = x1 + t * dx;
        double closest_y = y1 + t * dy;
        
        double dist_sq = (ox - closest_x)*(ox - closest_x) + (oy - closest_y)*(oy - closest_y);
        if (dist_sq < r*r) {
            return true;
        }
    }
    return false;
}


// Connect every Node to a Node with an Edge
void PathPlannerGraph::connect_node_to_existing_nodes(const Point& new_node) {
    for (const auto& [existing_node, edges] : graph_) {
        if (existing_node != new_node) {
            if (!edge_intersects_obstacles(new_node, existing_node)) {
                double dist = distance(new_node, existing_node);
                graph_[new_node].emplace_back(existing_node, dist);
                graph_[existing_node].emplace_back(new_node, dist);
            }
        }
    }
}

// Path Planning Algorithm
void PathPlannerGraph::generate_graph() {
    // Clear The Existence Graph
    graph_.clear();

    // Add The Inicial and Goal Points to The Graph
    graph_[start_] = {};
    graph_[goal_] = {};

    // Check if possible to have direct connection between the Inicial and Goal Points
    if (!edge_intersects_obstacles(start_, goal_)) {
        double dist = distance(start_, goal_);
        graph_[start_].emplace_back(goal_, dist);
        graph_[goal_].emplace_back(start_, dist);
        return;
    }
    
    connect_node_to_existing_nodes(start_);
    
    // Create Nodes around The Obstacles
    for (size_t idx = 0; idx < obstacles_.size(); ++idx) {
        auto [ox, oy, radius] = obstacles_[idx];
        
        for (int i = 0; i < points_per_obstacle_; ++i) {
            double angle = 2 * M_PI * i / points_per_obstacle_;
            double px = ox + (radius + 0.3) * std::cos(angle);
            double py = oy + (radius + 0.3) * std::sin(angle);
            Point pt = std::make_tuple(px, py);
            
            // Verify if Node is in another Obstacle
            bool inside_other_obstacle = false;
            for (size_t jdx = 0; jdx < obstacles_.size(); ++jdx) {
                if (jdx == idx) continue;
                auto [other_ox, other_oy, other_r] = obstacles_[jdx];
                if (distance(pt, std::make_tuple(other_ox, other_oy)) < other_r) {
                    inside_other_obstacle = true;
                    break;
                }
            }

            // Check if The Point is Valid
            if (!inside_other_obstacle && is_point_in_ellipse(pt)) {
                graph_[pt] = {};
                connect_node_to_existing_nodes(pt);
            }
        }
    }
}



PathPlannerGraph::Graph PathPlannerGraph::get_graph() const {
    return graph_;
}