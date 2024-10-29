#include <vector>
#include <queue>
#include <unordered_set>
#include <memory>
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>

/**
 * @brief Pose object to represent the position of the robot.
 * @param x x coordinate of the robot
 * @param y y-coordinate of the robot.
 */
class Pose
{
    public: 

    int x;
    int y;
    
    Pose(const int x, const int y) : x(x), y(y) 
    { 
    }

    bool operator==(const Pose &pose) const;
};

/**
 * @brief Object to represent a circular obstacle.
 * @param center Pose object to represent the coordinates of the center of the obstacle.
 * @param radius Radius of the obstacle.
 */
class CircularObstacle 
{
    public: 

    Pose center;
    int radius;

    CircularObstacle(const int x, const int y, const int r) : center(x, y), radius(r) 
    {
    }
};

/**
 * @brief Object to represent a circular robot.
 * @param radius Radius of the robot.
 */
class CircularRobot 
{
    public:

    int radius;
    CircularRobot(int r) : radius(r) 
    {
    }
};

/**
 * @brief Object to represent a rectangular map.
 * @param obstacles Vector of obstacles in the map.
 * @param min_bound Min bound of the map.
 * @param max_bound Max bound of the map.
 */
class RectangularMap 
{
    private:

    std::vector<CircularObstacle> obstacles;
    Pose min_bound;
    Pose max_bound;

    /**
    * @brief Checks if the robot is within the map bounds.
    * @param pose Pose of the robot.
    * @param radius Radius of the robot.
    * @return True if robot is within the map.
    */
    bool isWithinMapBounds(const Pose& pose, const int radius) const ;

    /**
    * @brief Checks if the robot is in collision with an obstacle in the map.
    * @param pose Pose of the robot.
    * @param radius Radius of the robot.
    * @return True if robot is in collision.
    */
    bool isCollision(const Pose& pose, const int radius) const ;

    public:

    /**
    * @brief Constructor to instantiate a map.
    * @param obstacle_vec Vector of obstacles to add to the map.
    * @param min_bound Min bound (top left corner for a rectangular map)
    * @param max_bound Max bound (bottom right corner for a rectangular map)
    */
    RectangularMap(const std::vector<CircularObstacle>& obstacle_vec, const Pose& min_bound, const Pose& max_bound):
    obstacles(obstacle_vec), min_bound(min_bound), max_bound(max_bound)
    {
    }

    /**
    * @brief Adds Obstacles to the map.
    * @param obstacle_vec Vector of obstacles to add to the map.
    */
    void addObstacles(const std::vector<CircularObstacle>& obstacle_vec);


    /**
    * @brief Removes all obstacles from the map.
    */
    void clearObstacles();

    /**
    * @brief Function to check if the pose is within the map and collision free.
    * @param pose Pose of the robot.
    * @param radius Radius of the robot.
    * @return Returns true if pose is valid.
    */
    bool isValidPose(const Pose& pose, const int radius) const ;

};

// Planner related Objects.

/**
 * @brief Object to represent a Node in the AStar Search.
 * @param position Pose represented by the node.
 * @param g_cost G cost of the node
 * @param h_cost Heuristic value of the node.
 * @param parent Parent node.
 */

class Node 
{
    public :

    Pose position;
    double g_cost;
    double h_cost;
    std::shared_ptr<Node> parent;

    Node(Pose pos, double g, double h, std::shared_ptr<Node> p = nullptr) 
        : position(pos), g_cost(g), h_cost(h), parent(p) {}

    /**
    * @brief Returns the f cost of the node.
    * @return F cost of the node.
    */
    double getCost() const ;

    bool operator==(const Node& other) const;
};

struct NodeCompare 
{
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const;
};

struct NodeComparator
{
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const;
};

struct NodeHash 
{
    std::size_t operator() (const std::shared_ptr<Node> node) const;
};

/**
 * @brief Planner object implementing A* search to find shortest path from start to goal.
 * @param map Map object 
 * @param robot Robot Object
 */

class AStarPlanner
{
    public:

    AStarPlanner(const RectangularMap& map, const CircularRobot robot): map(map), robot(robot){}

    /**
    * @brief Returns the shortest path from the start to goal pose, given a path exists.
    * @param start Start pose.
    * @param goal Goal Pose
    * @return Vector of poses representing shortest path.
    */
    std::vector<Pose> getPath(const Pose& start, const Pose& goal) const;

    private:

    RectangularMap map;
    CircularRobot robot;

    // Costs for octile distance heuristic.
    const double cardinal_cost = 1.0;
    const double diagnol_cost = std::sqrt(2);

    /**
    * @brief Given a pose and robot radius, this returns a vector of valid neighbours.
    * This function assumes a 8-connected grid representation, where the robot can move horizontally
    * vertically and diagnolly.
    * @param pose Pose whose neighbours are to be calculated.
    * @param radius Radius of robot.
    * @return Vector of poses representing valid neighbours.
    */
    std::vector<Pose> getNeighbors(const Pose& pose, const int radius) const;

    /**
    * @brief Returns octile distance heuristic value between start and goal node.
    * @param start Start pose.
    * @param goal goal pose.
    * @return Heuristic value
    */
    double getHeuristic(const Pose& start, const Pose& goal) const;

    /**
    * @brief Returns cost based on predefined cardinal and diagnol costs.
    * @param start Start pose.
    * @param goal goal pose.
    * @return Cost
    */
    double getCost(const Pose& start, const Pose& goal) const;

    /**
    * @brief Given the goal node, this function returns the path by backtracking to the source node.
    * @param node Shared ptr to the goal node.
    * @return A vector of poses representing the shortest path.
    */
    std::vector<Pose> backtrackPath(const std::shared_ptr<Node> node) const; 

};