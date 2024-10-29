#include "planner.h"

/* Define functions for the Pose class. */
bool Pose::operator==(const Pose &pose) const 
{
    return (pose.x == x && pose.y == y);
}

/* Define functions for the Map class. */

void RectangularMap::addObstacles(const std::vector<CircularObstacle>& obstacle_vec)
{
    obstacles.insert(obstacles.end(),obstacle_vec.begin(),obstacle_vec.end());
}

void RectangularMap::clearObstacles()
{
    obstacles.clear();
}

bool RectangularMap::isWithinMapBounds(const Pose& pose, const int radius) const
{
    if (pose.x < min_bound.x + radius || pose.x > max_bound.x - radius ||
        pose.y < min_bound.y + radius || pose.y > max_bound.y - radius)
    {
        return false;
    }

    return true; 
}

bool RectangularMap::isCollision(const Pose& pose, const int radius) const
{
    for (const auto& obstacle : obstacles) 
    {
        const double distance = std::sqrt(std::pow(pose.x - obstacle.center.x, 2) + 
                                    std::pow(pose.y - obstacle.center.y, 2));
        if (distance < (obstacle.radius + radius)) 
        {
            return true;
        }
    }

    return false;
}

bool RectangularMap::isValidPose(const Pose& pose, const int radius) const
{
    if (isWithinMapBounds(pose,radius) && !isCollision(pose,radius))
    {
        return true;
    }
    else
    {
        return false;
    } 
}



/* Define functions for the Node class. */

double Node::getCost() const 
{ 
    return g_cost + h_cost; 
}

bool Node::operator==(const Node& other) const 
{
    return position == other.position;
}


bool NodeCompare::operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const 
{
    return a->getCost() > b->getCost();
}



bool NodeComparator::operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const 
{
    return a->position == b->position;
}


std::size_t NodeHash::operator() (const std::shared_ptr<Node> node) const 
{
    std::string hash_str = std::to_string(node->position.x) + "," + std::to_string(node->position.y);
    return std::hash<std::string>{}(hash_str);
}


/* Define functions for the AStarPlanner classes. */

std::vector<Pose> AStarPlanner::getNeighbors(const Pose& pose, const int radius) const
{
    std::vector<Pose> neighbors;
    neighbors.reserve(8);
    for (int dx = -1; dx <= 1; ++dx) 
    {
        for (int dy = -1; dy <= 1; ++dy) 
        {
            if (dx == 0 && dy == 0) continue;
            Pose neighbor(pose.x + dx, pose.y + dy);
            if (map.isValidPose(neighbor,radius)) 
            {
                neighbors.push_back(neighbor);
            }
        }
    }
    return neighbors;
}

double AStarPlanner::getHeuristic(const Pose& start, const Pose& goal) const
{
    int dx = std::abs(start.x - goal.x);
    int dy = std::abs(start.y - goal.y);
    return cardinal_cost * (dx + dy) + (diagnol_cost - 2 * cardinal_cost) * std::min(dx, dy);
}

double AStarPlanner::getCost(const Pose& start, const Pose& goal) const
{
    if(start.x == goal.x || start.y == goal.y)
    {
        return cardinal_cost;
    }
    else
    {
        return diagnol_cost;
    }
}

std::vector<Pose> AStarPlanner::backtrackPath(std::shared_ptr<Node> node) const
{
    auto current_node = node;
    std::vector<Pose> path;
    while(current_node != nullptr)
    {
        path.push_back(current_node->position);
        current_node = current_node->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Pose> AStarPlanner::getPath(const Pose& start, const Pose& goal) const
{
    std::vector<Pose> path;
    const int radius = robot.radius;

    // Check for validity of start and goal
    if(!map.isValidPose(start,radius) || !map.isValidPose(goal,radius))
    {
        // Return empty path
        std::cout << "Start or Goal pose is not valid. Returning empty path." << std::endl;
        return path;
    }

    // Open list - Priority queue used to expand nodes in order of lowest f cost.
    std::priority_queue<std::shared_ptr<Node>, 
                        std::vector<std::shared_ptr<Node>>,
                        NodeCompare> open_list;
    // g_value_list - Contains the lowest g values for a node at a given instant.
    // This is updated if we find a lower cost path.
    std::unordered_map<std::shared_ptr<Node>, double,NodeHash, NodeComparator> g_value_list;
    // Closed list - Contains expanded nodes with optimal g values and cost.
    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeComparator> closed_list;

    std::shared_ptr<Node> start_node = std::make_shared<Node>(start, 0, getHeuristic(start,goal));
    open_list.push(start_node);
    g_value_list.insert({start_node, start_node->g_cost});

    while(!open_list.empty())
    {
        auto current_node = open_list.top();
        open_list.pop();

        // If node is already present in the closed list, skip and move to next node in queue.
        if(closed_list.find(current_node) != closed_list.end())
        {
            continue;
        }
        else
        {
            closed_list.insert(current_node);
        }
        
        // We have reached goal - return path.
        if(current_node->position == goal)
        {
            return backtrackPath(current_node);
        }

        // Get neighbours and update open list.
        auto neighbors = getNeighbors(current_node->position, radius);
        for(const auto& neighbor : neighbors)
        {
            double g = current_node->g_cost + getCost(current_node->position, neighbor);
            double h = getHeuristic(neighbor,goal);
            auto neighbor_node = std::make_shared<Node>(neighbor,g,h,current_node);

            // If node is not present in open list, or current node decreases the g cost of neighbor node
            // add node to the open list. Additionally, update g cost in g_value_list.
            auto it = g_value_list.find(neighbor_node);

            if (it == g_value_list.end() || g < it->second) 
            {
                open_list.push(neighbor_node);
                // Update g cost in g value list.
                if(it == g_value_list.end())
                {
                    g_value_list.insert({neighbor_node, g});
                }
                else
                {
                    g_value_list[neighbor_node] = g;
                }
            }
        }
    }
    return path;

}


/* Driver code to run the planner on an example map. */

int main()
{
    CircularRobot robot(1);
    std::vector<CircularObstacle> obstacles;
    obstacles.push_back(CircularObstacle(3,3,1));
    obstacles.push_back(CircularObstacle(1,3,1));
    RectangularMap map(obstacles,Pose(0,0), Pose(8,8));
    AStarPlanner planner(map,robot);
    auto path = planner.getPath(Pose(1,1),Pose(1,7));
    for(const auto& elem : path)
    {
        std::cout << elem.x << " , " << elem.y << std::endl;
    }
    return 0;
}