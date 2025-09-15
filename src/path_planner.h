#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <functional>

// Hash function para std::tuple<double, double>
namespace std {
    template <>
    struct hash<std::tuple<double, double>> {
        size_t operator()(const std::tuple<double, double>& point) const {
            auto [x, y] = point;
            size_t h1 = std::hash<double>{}(x);
            size_t h2 = std::hash<double>{}(y);
            return h1 ^ (h2 << 1);
        }
    };
}

class PathPlannerGraph {
public:
    using Point = std::tuple<double, double>;
    using Obstacle = std::tuple<double, double, double>;
    using Graph = std::unordered_map<Point, std::vector<std::pair<Point, double>>>;
    
    PathPlannerGraph(Point start, Point goal, std::vector<Obstacle> obstacles, 
                    int points_per_obstacle = 5, double ellipse_factor = 1.1);
    
    Graph get_graph() const;
    void generate_graph();
    
private:
    Point start_;
    Point goal_;
    std::vector<Obstacle> obstacles_;
    int points_per_obstacle_;
    double ellipse_factor_;
    Graph graph_;
    
    bool is_point_in_ellipse(const Point& point) const;
    double distance(const Point& p1, const Point& p2) const;
    bool edge_intersects_obstacles(const Point& p1, const Point& p2) const;
    void connect_node_to_existing_nodes(const Point& new_node);
};

#endif // PATH_PLANNER_H