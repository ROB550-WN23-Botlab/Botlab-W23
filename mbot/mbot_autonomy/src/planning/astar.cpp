#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
                                            mbot_lcm_msgs::pose_xyt_t goal,
                                            const ObstacleDistanceGrid &distances,
                                            const SearchParams &params)
{
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    ////////////////// TODO: Implement your A* search here //////////////////////////
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    Node *goalNode = new Node(goalCell.x, goalCell.y);
    Node *startNode = new Node(startCell.x, startCell.y);

    PriorityQueue openList;
    std::vector<Node *> closedList;
    std::vector<Node *> searchedList;

    startNode->g_cost = 0;
    startNode->h_cost = h_cost(startNode, goalNode, distances);

    openList.push(startNode);

    bool found_path = false;
    while (!openList.empty() && !found_path)
    {

        Node *currentNode = openList.pop();
        if (!(*currentNode == *goalNode))
        {
            closedList.push_back(currentNode);
            expand_node(currentNode, goalNode, distances, params, openList, closedList, searchedList);
        }
        else
        {
            goalNode = currentNode;
            found_path = true;
        }
    }

    mbot_lcm_msgs::robot_path_t path;
    path.utime = start.utime;

    if (found_path)
    {
        std::cout << "openList len = " << openList.elements.size() << std::endl;
        std::vector<Node *> nodePath = extract_node_path(goalNode, startNode);
        std::cout << "nodePath len = " << nodePath.size() << std::endl;
        std::vector<Node *> prunedNodePath = prune_node_path(nodePath);
        std::cout << "pruned len = " << prunedNodePath.size() << std::endl;
        path.path = extract_pose_path(prunedNodePath, distances);
    }
    else
    {
        std::cout << "did not find path :< \n"
                  << std::endl;
    }
    path.path_length = path.path.size();
    return path;
}

double h_cost(Node *from, Node *goal, const ObstacleDistanceGrid &distances)
{
    // TODO: Return calculated h cost
    // diagonal distance
    int dx = abs(goal->cell.x - from->cell.x);
    int dy = abs(goal->cell.y - from->cell.y);
    double straight_distance = 1.0;
    double diag_distance = 1.414;

    double h_cost = straight_distance * (dx + dy) + (diag_distance - 2 * straight_distance) * std::min(dx, dy);
    h_cost *= distances.metersPerCell();
    return h_cost;
}

double g_cost(Node *from, Node *goal, const ObstacleDistanceGrid &distances, const SearchParams &params)
{
    // TODO: Return calculated g cost
    double g_cost = from->g_cost;

    int dx = abs(goal->cell.x - from->cell.x);
    int dy = abs(goal->cell.y - from->cell.y);

    if (dx == 1 && dy == 1)
    {
        g_cost += 1.41;
    }
    else
    {
        g_cost += 1.0;
    }

    // Penalize if close to obstacle
    double penalization = 0.0;
    if (distances(goal->cell.x, goal->cell.y) <= params.maxDistanceWithCost)
    {
        penalization = pow(params.maxDistanceWithCost - distances(goal->cell.x, goal->cell.y), params.distanceCostExponent);
    }

    g_cost = g_cost * distances.metersPerCell() + penalization;
    return g_cost;
}


void expand_node(Node* node, Node* goalNode, const ObstacleDistanceGrid& distances, const SearchParams& params, PriorityQueue& openList, std::vector<Node*>& closedList, std::vector<Node*>& searchedList)
{
    // TODO: Return children of a given node that are not obstacles
    int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};

    for(int i=0; i<8; i++)
    {
        int x = node->cell.x + dx[i];
        int y = node->cell.y + dy[i];
        Node* neighbor = new Node(x,y);
        if(is_in_list(neighbor, searchedList))
            neighbor = get_from_list(neighbor, searchedList);
        
        if(!is_in_list(neighbor, closedList) && distances.isCellInGrid(x, y) && distances(x, y) > params.minDistanceToObstacle)
        {
            if(!is_in_list(neighbor, searchedList))
            {
                neighbor->g_cost = g_cost(node, neighbor, distances, params);
                neighbor->h_cost = h_cost(neighbor, goalNode, distances);
                neighbor->parent = node;
                openList.push(neighbor);
                searchedList.push_back(neighbor);
            }

            // neighbor has been reached before but current path is better
            else if(neighbor->g_cost > g_cost(node, neighbor, distances, params))
            {
                neighbor->g_cost = g_cost(node, neighbor, distances, params);
                neighbor->parent = node;
                openList.push(neighbor);
            }
        }
    }
}




std::vector<Node *> extract_node_path(Node *goal_node, Node *start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node *> path;
    Node *curr_node = goal_node;

    while (!(*curr_node == *start_node))
    {
        path.push_back(curr_node);
        curr_node = curr_node->parent;
        // std::cout << " found parent"<<std::endl;
    }
    std::reverse(path.begin(), path.end());
    return path;
}


std::vector<Node*> prune_node_path(std::vector<Node*> nodePath){
    if(nodePath.size() < 3) return nodePath;

    std::vector<Node*> newPath;
    std::vector<Node*> finalPath;
    newPath.push_back(nodePath[0]);

    Node* prevNode = NULL;
    Node* currNode = NULL;
    Node* nextNode = NULL;
    int prev_dx, prev_dy, next_dx, next_dy, dx, dy;
    for(int i=1; i<nodePath.size()-1; i++){
        //dont add node if direction doesnt change
        prevNode = nodePath[i-1];
        currNode = nodePath[i];
        nextNode = nodePath[i+1];

        prev_dx = currNode->cell.x - prevNode->cell.x;
        prev_dy = currNode->cell.y - prevNode->cell.y;
        next_dx = nextNode->cell.x - currNode->cell.x;
        next_dy = nextNode->cell.y - currNode->cell.y;

        // double norm = std::sqrt((double)prev_dx*prev_dx + (double)prev_dy*prev_dy);

        if(prev_dx != next_dx || prev_dy != next_dy){
            newPath.push_back(currNode);
        }
    }

    newPath.push_back(nodePath.back());
    finalPath.push_back(newPath[0]);
    for(int i=1; i<newPath.size()-1; i++)
    {
        dx = newPath[i]->cell.x - newPath[i-1]->cell.x;
        dy = newPath[i]->cell.y - newPath[i-1]->cell.y;

        double norm = std::sqrt((double)dx*dx + (double)dy*dy);
        if(norm > 2)
        {
            finalPath.push_back(newPath[i]);
        }
    }
    finalPath.push_back(newPath.back());
    return finalPath;
}


// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node *> nodes, const ObstacleDistanceGrid &distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> posePath;
    int count = 0;
    int N = nodes.size();
    for (auto &&node : nodes)
    {

        Point<double> global_pose = grid_position_to_global_position(node->cell, distances);
        mbot_lcm_msgs::pose_xyt_t currPose;
        currPose.x = global_pose.x;
        currPose.y = global_pose.y;

        if (posePath.size() == 0)
        {
            currPose.theta = 0;
        }
        else
        {
            if (count == N - 1)
            {
                currPose.theta = 0;
                
            }
            else
            {
                mbot_lcm_msgs::pose_xyt_t prevPose = posePath.back();
                currPose.theta = atan2(currPose.y - prevPose.y, currPose.x - prevPose.x);
            }
        }
        count++;
        currPose.utime = 0;
        posePath.push_back(currPose);
    }
    return posePath;
}

bool is_in_list(Node *node, std::vector<Node *> list)
{
    for (auto &&item : list)
    {
        if (*node == *item)
            return true;
    }
    return false;
}

Node *get_from_list(Node *node, std::vector<Node *> list)
{
    for (auto &&n : list)
    {
        if (*node == *n)
            return n;
    }
    return NULL;
}


// #include <planning/astar.hpp>
// #include <algorithm>
// #include <chrono>
// #include <planning/obstacle_distance_grid.hpp>
// #include <common_utils/timestamp.h>


// using namespace std::chrono;


// PriorityQueue searchQueue; // open list
// std::vector<Node*> closedList; // closed list

// mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
//                                              mbot_lcm_msgs::pose_xyt_t goal,
//                                              const ObstacleDistanceGrid& distances,
//                                              const SearchParams& params)
// {
//      ////////////////// TODO: Implement your A* search here //////////////////////////
//     printf("Start the search_for_path function!!\n");

//     mbot_lcm_msgs::robot_path_t path;
//     path.utime = utime_now();
//     // Intialize the goal cell and the start cell
//     cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
//     cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
//     // std::cout << "goalcell: " << goalCell.x << ", " << goalCell.y << std::endl;
//     // std::cout << "startcell: " << startCell.x << ", " << startCell.y << std::endl;
//     // Initialize the goal node and the  start node
//     Node* goalNode = new Node(goalCell.x, goalCell.y);
//     Node* startNode = new Node(startCell.x, startCell.y);
//     // Initialize the g cost and h cost
//     goalNode->g_cost = 1.0E16; // Need to check the initial value setting!
//     goalNode->h_cost = 0.0;
//     goalNode->parent = NULL;
//     startNode->g_cost = 0.0;
//     startNode->h_cost = h_cost(startNode, goalNode, distances);
//     startNode->parent = NULL;
    
//     std::vector<Node*> node_path;
//     // startNode overlap with goalNode
//     if (*goalNode == *startNode)
//     {
//         node_path = extract_node_path(goalNode, startNode);
//         path.path = extract_pose_path(node_path, distances);
//         path.path_length = path.path.size();

//         return path;
//     }
//     // startNode not overlap with goalNode
//     else
//     {
//         // Check whether the startNode exceed the Grid
//         if (!distances.isCellInGrid(startNode->cell.x, startNode->cell.y))
//         {
//             std::cout<<"INFO: Invalid start cell" << std::endl;
//             path.path_length = 1;
//             return path;      
//         }
//         // Enqueue the startNode into the priority queue
//         searchQueue.push(startNode);
//         // std::cout << searchQueue.elements.size() << std::endl;
//         // printf("enqueue the fist node\n");

//         // when the Openlist not empty
//         while (!(searchQueue.empty()))
//         {
//             // Find the currentNode and pop it from the priority queue
//             auto currentNode = searchQueue.pop();
//             // Expand the currentNode
//             std::vector<Node*> children;
//             std::vector<Node*> kids = expand_node(currentNode, goalNode, distances, params);
//             // std::cout << "kids number: " << kids.size() << std::endl;
//             // Calculate the f cost of each kid
//             for (auto kid : kids)
//             {
//                 // std::cout << "kid parent: " << kid->parent->cell.x << "," << kid->parent->cell.y << std::endl;
//                 // printf("enter for kids for loop\n");
//                 kid->g_cost = 5 * g_cost(currentNode, kid, distances, params);
//                 kid->h_cost = h_cost(kid, goalNode, distances);
//                 // Check whether the kid reaches the goalNode
//                 // printf("if 1\n");
                
//                 if (*kid == *goalNode)
//                 {
//                     // printf("kid == goalnode\n");
//                     // std::cout << "kid parent: " << kid->parent->cell.x << "," << kid->parent->cell.y << std::endl;
//                     // printf("extract_node_path \n");
//                     node_path = extract_node_path(kid, startNode);
//                     // printf("extract_pose_path \n");
//                     path.path = extract_pose_path(node_path, distances);
//                     path.path_length = path.path.size();
//                     while (!(searchQueue.empty())) {
//                         searchQueue.pop();
//                     }
//                     closedList.clear();
//                     printf("closedList clear\n");
//                     // std :: cout << "path_length: " << path.path_length << std::endl;
//                     return path;
//                 }
//                 // printf("if 2\n");
//                 // Not reach the goalNode yet, and push the kid into the priority queue / open list
//                 if (!(is_in_list(kid, closedList)) && !(searchQueue.is_member(kid)))
//                 {
//                     searchQueue.push(kid);
//                     // std::cout << "updated searchQueue number: " << searchQueue.elements.size() << std::endl;
//                 }
//                 // printf("exit for kids for loop\n");
//             }
//             // Push the currentNode into the closedList
//             if (!(is_in_list(currentNode, closedList)))
//             {
//                 closedList.push_back(currentNode);
//             }
//         }
//     }
//     return path;
// }



// double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
// {
//     // TODO: Return calculated h cost
//     double h_cost = 0;
//     double dx = std::fabs(from->cell.x - goal->cell.x);
//     double dy = std::fabs(from->cell.y - goal->cell.y);
//     h_cost = ((dx + dy) + (1.414 - 2) * std::min(dx, dy));

//     return h_cost;
// }
// double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
// {
//     // TODO: Return calculated g cost
//     double g_cost = 0;
//     double distance = distances(goal->cell.x, goal->cell.y);
//     double dx = std::fabs(from->cell.x - goal->cell.x);
//     double dy = std::fabs(from->cell.y - goal->cell.y);
//     // obstacle distance of goal cell is greater than maxDistancewithCost, then g_cost is with penalty
//     if (distance < params.maxDistanceWithCost && distance > params.minDistanceToObstacle)
//     {
//         if (dx == 1.0 && dy == 1.0)
//     {
//         g_cost = from->g_cost + 14 + pow(params.maxDistanceWithCost - distance, params.distanceCostExponent);
//     }  
//     else
//     {
//         g_cost = from->g_cost + 10 + pow(params.maxDistanceWithCost - distance, params.distanceCostExponent);
//     }
//     }
//     // obstacle distance of goal cell is greater than maxDistancewithCost, then g_cost is without penalty
//     else {
//     if (dx == 1.0 && dy == 1.0)
//     {
//         g_cost = from->g_cost + 14;
//     }  
//     else
//     {
//         g_cost = from->g_cost + 10;
//     }
//     }

//     return g_cost;
// }


// std::vector<Node*> expand_node(Node* node, Node* goalNode, const ObstacleDistanceGrid& distances, const SearchParams& params)
// {
//     // TODO: Return children of a given node that are not obstacles
//     std::vector<Node*> children;
//     int x_cord = node->cell.x;
//     int y_cord = node->cell.y;
//     const int xindex[8] = {1, -1, 0, 0, 1, -1, 1, -1};
//     const int yindex[8] = {0, 0, 1, -1, 1, -1, -1, 1};
//     for (int n = 0; n < 8; ++n)
//     {
//         // Create a new neighbor node
//         Node* children_node;
//         children_node = new Node(x_cord + xindex[n], y_cord + yindex[n]);
//         children_node->parent = node;
//         // Check the new node in the open list or not
//         if (searchQueue.is_member(children_node))
//         {
//             // Find the existing children node from the openlist
//             Node* existed_children_node = searchQueue.get_member(children_node);
//             // Compute the f cost
//             children_node->g_cost = 5 * g_cost(node, children_node, distances, params);
//             children_node->h_cost = h_cost(children_node, goalNode, distances);
//             // Test which f_cost is smaller
//             if (children_node->f_cost() > existed_children_node->f_cost())
//             {
//                 children_node = existed_children_node;
//             }
//             // children_node = searchQueue.get_member(children_node);
//         }
        
//         // Check three conditions: distance to the obstacle, in the closed list, in the grid
//         bool distance_flag = distances(children_node->cell.x, children_node->cell.y) >= params.minDistanceToObstacle; // Need to be check!!
//         bool In_list_flag = is_in_list(children_node, closedList);
//         bool In_grid_flag = distances.isCellInGrid(children_node->cell.x, children_node->cell.y);
//         if (distance_flag && !In_list_flag && In_grid_flag)
//         {
//             children.push_back(children_node);
//         }
//     }

//     return children;
// }

// std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
// {
//     // TODO: Generate path by following parent nodes
//     // std::cout << "goal_node->cell: " << goal_node->cell.x << "," << goal_node->cell.y << std::endl;
//     // std::cout << "start_node->cell: " << start_node->cell.x << "," << start_node->cell.y << std::endl;
//     std::vector<Node*> path;
//     Node* temp_node = goal_node;
//     // int count = 0;
//     while(temp_node != NULL)
//     {
//         // std::cout << "temp_node->cell: " << temp_node->cell.x << "," << temp_node->cell.y << std::endl;
//         // std::cout << "temp_node->parent->cell: " << temp_node->parent->cell.x << "," << temp_node->parent->cell.y << std::endl;
//         path.push_back(temp_node);
//         temp_node = temp_node->parent;
//         // std::cout << ++count << std::endl;
//     }
//     path.push_back(start_node);
//     // std::cout << "extract_node_path.size(): "<< path.size() << std::endl;

//     return path;
// }

// // To prune the path for the waypoint follower
// std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
// {
//     // TODO: prune the path to generate sparse waypoints
//     std::vector<mbot_lcm_msgs::pose_xyt_t> path;
//     // reverse the nodes list
//     std::reverse(nodes.begin(), nodes.end());
//     //Define a pose to record the current_pose
//     mbot_lcm_msgs::pose_xyt_t current_pose;
//     mbot_lcm_msgs::pose_xyt_t previous_pose;

//     // printf("enter for loop in extract_pose_path\n");
//     for (auto node : nodes)
//     {
//         // printf("extrect_pose_path: node i\n");

//         Point<double> pathpoint = grid_position_to_global_position(Point<int>(node->cell.x, node->cell.y), distances);
//         // printf("pathpoint done\n");
//         current_pose.x = pathpoint.x;
//         current_pose.y = pathpoint.y;
//         current_pose.utime = utime_now();
//         // First node
//         if (node == nodes[0] || node->parent == NULL)
//         {
//             // printf("the first node\n");
//             current_pose.theta = 0.0;
//             path.push_back(current_pose);
//             continue;
//         }
//         // std::cout << "x: " << node->parent->cell.x << std::endl;
//         // std::cout << "y: " << node->parent->cell.y << std::endl;
//         Point<double> pathpoint_parent = grid_position_to_global_position(Point<int>(node->parent->cell.x, node->parent->cell.y), distances);
//         // printf("pathpoint_parent done\n");
//         previous_pose.x = pathpoint_parent.x;
//         previous_pose.y = pathpoint_parent.y;
        
//         // The other nodes
//         // printf("the other nodes\n");
//         double dx = current_pose.x - previous_pose.x;
//         double dy = current_pose.y - previous_pose.y;
//         double theta = std::atan2(dy, dx);
//         current_pose.theta = theta;
//         path.push_back(current_pose);

//     }
//     // std::cout << "pathlength: " << path.size() << std::endl;
//     return path;
// }

// bool is_in_list(Node* node, std::vector<Node*> list)
// {
//     for (auto &&item : list)
//     {
//         if (*node == *item) return true;
//     }
//     return false;
// }

// Node* get_from_list(Node* node, std::vector<Node*> list)
// {
//     for (auto &&n : list)
//     {
//         if (*node == *n) return n;
//     }
//     return NULL;
    
// }