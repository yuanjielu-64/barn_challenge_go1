#include "astar.hpp"
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>

// Point comparison operation
bool AStarPlanner::Point::operator==(const Point& other) const {
    return x == other.x && y == other.y;
}

// Point hash function
size_t AStarPlanner::PointHash::operator()(const Point& p) const {
    return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
}

// Node f_cost calculation
int AStarPlanner::Node::f_cost() const {
    return g_cost + h_cost;
}

// Node comparison operation (for priority queue)
bool AStarPlanner::Node::operator>(const Node& other) const {
    return f_cost() > other.f_cost();
}

// Constructor
AStarPlanner::AStarPlanner(double resolution, double robot_radius)
    : resolution_(resolution), robot_radius_(robot_radius) {}

// Calculate heuristic distance (Manhattan distance)
int AStarPlanner::calculateHeuristic(const Point& a, const Point& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// Check if point is within map bounds
bool AStarPlanner::isValid(const Point& p, int width, int height) {
    return p.x >= 0 && p.x < width && p.y >= 0 && p.y < height;
}

// Check if point is obstacle-free
bool AStarPlanner::isFree(const Point& p, const std::vector<std::vector<bool>>& grid) {
    return isValid(p, grid[0].size(), grid.size()) && !grid[p.y][p.x];
}

// World coordinates to grid coordinates
AStarPlanner::Point AStarPlanner::worldToGrid(double x, double y) {
    return {static_cast<int>(x / resolution_), static_cast<int>(y / resolution_)};
}

// Grid coordinates to world coordinates
std::vector<double> AStarPlanner::gridToWorld(const Point& p) {
    return {(p.x + 0.5) * resolution_, (p.y + 0.5) * resolution_};
}

// Inflate obstacles to account for robot size
void AStarPlanner::inflateObstacles(
    std::vector<std::vector<bool>>& grid,
    int width, int height) {

    int inflation_cells = static_cast<int>(std::ceil(robot_radius_ / resolution_));
    std::vector<std::vector<bool>> original_grid = grid;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (original_grid[y][x]) {  // If it's an obstacle
                // Inflate around the obstacle
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;

                        // Check if within map bounds
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            // Check if within inflation radius
                            if (std::sqrt(dx*dx + dy*dy) <= inflation_cells) {
                                grid[ny][nx] = true;
                            }
                        }
                    }
                }
            }
        }
    }
}

// Main A* path planning function
std::vector<std::vector<double>> AStarPlanner::planPath(
    const std::vector<std::vector<double>>& obstacles,
    double start_x, double start_y,
    double goal_x, double goal_y,
    double world_width, double world_height) {

    // Calculate grid dimensions
    int grid_width = static_cast<int>(std::ceil(world_width / resolution_));
    int grid_height = static_cast<int>(std::ceil(world_height / resolution_));

    // Initialize grid with all cells free
    std::vector<std::vector<bool>> grid(grid_height, std::vector<bool>(grid_width, false));

    // Mark obstacle cells
    for (const auto& obs : obstacles) {
        if (obs.size() >= 2) {
            Point p = worldToGrid(obs[0], obs[1]);
            if (isValid(p, grid_width, grid_height)) {
                grid[p.y][p.x] = true;  // Mark as obstacle
            }
        }
    }

    // Inflate obstacles
    inflateObstacles(grid, grid_width, grid_height);

    // Direction vectors (8-connected)
    const std::vector<Point> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},  // Up, Right, Down, Left
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}  // Diagonals
    };

    Point start_point = worldToGrid(start_x, start_y);
    Point goal_point = worldToGrid(goal_x, goal_y);

    // Check if start or goal is in obstacle
    if (!isFree(start_point, grid) || !isFree(goal_point, grid)) {
        std::cout << "Start or goal position is in obstacle!" << std::endl;
        return {};
    }

    // Priority queue for open set
    std::priority_queue<Node*, std::vector<Node*>, std::greater<Node*>> open_set;

    // Set to track visited nodes
    std::unordered_set<Point, PointHash> closed_set;

    // Initialize start node
    Node* start_node = new Node{
        start_point,
        0,
        calculateHeuristic(start_point, goal_point),
        nullptr
    };

    open_set.push(start_node);

    while (!open_set.empty()) {
        // Get node with lowest f_cost
        Node* current = open_set.top();
        open_set.pop();

        // If we've reached the goal
        if (current->point == goal_point) {
            // Reconstruct path
            std::vector<std::vector<double>> path;
            while (current != nullptr) {
                path.push_back(gridToWorld(current->point));
                Node* parent = current->parent;
                delete current;  // Clean up memory
                current = parent;
            }

            // Clean up remaining memory
            while (!open_set.empty()) {
                delete open_set.top();
                open_set.pop();
            }

            // Reverse path (from start to goal)
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Add to closed set
        closed_set.insert(current->point);

        // Check all neighbors
        for (const auto& dir : directions) {
            Point neighbor_point = {current->point.x + dir.x, current->point.y + dir.y};

            // Skip if neighbor is invalid, is an obstacle, or is already in closed set
            if (!isFree(neighbor_point, grid) || closed_set.count(neighbor_point) > 0) {
                continue;
            }

            // Calculate cost to this neighbor
            int movement_cost = (dir.x != 0 && dir.y != 0) ? 14 : 10;  // 1.4 for diagonal, 1 for straight
            int g_cost = current->g_cost + movement_cost;

            // Create neighbor node
            Node* neighbor = new Node{
                neighbor_point,
                g_cost,
                calculateHeuristic(neighbor_point, goal_point),
                current
            };

            // Add to open set
            open_set.push(neighbor);
        }
    }

    std::cout << "No path found!" << std::endl;
    return {};
}

// Save path to file
void AStarPlanner::savePathToFile(const std::vector<std::vector<double>>& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filename << std::endl;
        return;
    }

    file << "// A* Path Planning Result\n";
    file << "// Format: x y\n";
    file << "const std::vector<std::vector<double>> path = {\n";

    for (const auto& point : path) {
        if (point.size() >= 2) {
            file << "    {" << point[0] << ", " << point[1] << "},\n";
        }
    }

    file << "};\n";
    file.close();
}

// Get inflated grid (for visualization)
std::vector<std::vector<bool>> AStarPlanner::getInflatedGrid() const {
    // This function is just a placeholder, in real application you might want to store grid as a member variable
    return {};
}