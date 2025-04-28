#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <string>

class AStarPlanner {
public:
    // Constructor
    AStarPlanner(double resolution, double robot_radius);

    // Main A* path planning function
    std::vector<std::vector<double>> planPath(
        const std::vector<std::vector<double>>& obstacles,
        double start_x, double start_y,
        double goal_x, double goal_y,
        double world_width, double world_height);

    // Save path to file
    void savePathToFile(const std::vector<std::vector<double>>& path, const std::string& filename);

    // Get inflated grid (for visualization)
    std::vector<std::vector<bool>> getInflatedGrid() const;

private:
    struct Point {
        int x, y;
        bool operator==(const Point& other) const;
    };

    struct PointHash {
        size_t operator()(const Point& p) const;
    };

    struct Node {
        Point point;
        int g_cost;
        int h_cost;
        Node* parent;
        int f_cost() const;
        bool operator>(const Node& other) const;
    };

    double resolution_;  // Map resolution (meters/cell)
    double robot_radius_;  // Robot radius (for obstacle inflation)

    // Utility functions
    int calculateHeuristic(const Point& a, const Point& b);
    bool isValid(const Point& p, int width, int height);
    bool isFree(const Point& p, const std::vector<std::vector<bool>>& grid);
    Point worldToGrid(double x, double y);
    std::vector<double> gridToWorld(const Point& p);
    void inflateObstacles(
        std::vector<std::vector<bool>>& grid,
        int width, int height);
};

#endif // ASTAR_H