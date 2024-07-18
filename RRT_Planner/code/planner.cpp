/*=================================================================
 *
 * planner.cpp
 * A* finally works --> This planner implementation is looking at the trace_back function
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <vector>
#include "A_star.h"
#include <map> 
#include <cfloat> // Include this header for FLT_MAX
#include <iostream>
#include <limits>
#include <algorithm>
#include <chrono>
#include <queue>

#include <unordered_map> //include this header to try and make a map to store the values of the parent

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

//Global variables

static bool planned_path = false;
double weight = 0.4;
static int current_path_index = 0;

// Define state
struct state {
        int cell_x, cell_y;  //for row and column of cell --> x,y
        double f,g,h; // f is = g + h; g is cost from start node to this state; h is value of heuristic
        int time; 
        int parent_x;
        int parent_y;

        bool operator == (const state& other) const{
            return (cell_x == other.cell_x) && (cell_y == other.cell_y);
        }

        state(int x, int y, double g_new, double h_new, double f_value, int time_curr)
            : cell_x(x), cell_y(y), g(g_new), h(h_new), f(g_new + h_new), time(time_curr), parent_x(-1),parent_y(-1) {}
        
    };


// Update the heuristic function to use Euclidean distance
double Euclidean_dist(int current_x, int current_y, int goal_x, int goal_y) {
    int dx = abs(current_x - goal_x);
    int dy = abs(current_y - goal_y);
    double eucl_dist = (double)sqrt((dx*dx + dy*dy));
    return eucl_dist;
}

// Weighted Heuristic function
double Heuristic_Fcn_Cost(int current_x, int current_y, int goal_x, int goal_y, int* map, int x_size, int y_size, double weight) {
    int dx = abs(current_x - goal_x);
    int dy = abs(current_y - goal_y);
    double eucl_dist = sqrt(dx * dx + dy * dy);

    int obstacle_cost = map[GETMAPINDEX(current_x, current_y, x_size, y_size)];
    double weighted_heuristic = (1-weight)*eucl_dist + weight * obstacle_cost;

    return weighted_heuristic;
}

Cost Time_HFcn(Node_number current_x, Node_number current_y,Node_number goal_x, Node_number goal_y, int goal_t, int current_t){
    int dt = goal_t - current_t;

    int dx_x = std::abs(static_cast<int>(goal_x - current_x));
    int dy_y = std::abs(static_cast<int>(goal_y - current_y));
    double eucl_dist = (double)sqrt((dx_x*dx_x + dy_y*dy_y));

    double h_traverse_t = static_cast<double>(eucl_dist);
    
    //In later optimization can try to apply weights to the costs or distance traversed = to help optimize the planner...
    return std::min(h_traverse_t,static_cast<double>(dt));
}


struct CompareStates {
    bool operator()(const state& a, const state& b) {
        // if (a.f == b.f) {
        //     // If f values are equal, compare based on h values
        //     return a.h > b.h;
        // }
    // Otherwise, prioritize lower f values
    return a.f > b.f;
    }
};

struct Node_Info {
    std::pair<int,int> parents = {-1, -1};
    bool is_explored = false; 
};

// Expand State that is in the priority queue and return the neighbors
std::vector<state> expand_state(const state& s, int* map, int x_size, int y_size, int collision_thresh, int target_steps, int robotposeX, int robotposeY, const int* dX, const int* dY){
    std::vector<state> neighbors;
    // 8-connected grid expansion; from greedy planner
    for(int dir = 0; dir < NUMOFDIRS; dir++){
        int newx = s.cell_x + dX[dir];
        int newy = s.cell_y + dY[dir];

        // Check bounds
        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size){ 
            if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh)){ //if free
                // Add the neighbors to the open_list
                state neighbor(newx, newy,FLT_MAX, FLT_MAX, FLT_MAX, s.time + 1);
                neighbors.push_back(neighbor);
            }
        }
    }

    return neighbors;
}



void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // Need to wrap the entire while loops in an if statement to change the static planned_path to true; this way once a path is generated it is only generated once
    // Then after this the values should then be passed to the action_ptr as is necessary...when it is called?
    
    const int time_limit_seconds = target_steps;  // Set your desired time limit in seconds

    int current_step = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    //Initialize vectors/lists for the open and closed lists
    std::unordered_map<std::string, state*> closed_list_map;
    std::priority_queue<state, std::vector<state>, CompareStates> open_list;

    //List for Path Found
    static std::vector<std::pair<int, int>> path_found; // Store (parent_x, parent_y) pairs

    // Get goal state at the end of the trajectory
    int goalposeX = target_traj[target_steps-1];
    int goalposeY = target_traj[target_steps-1+target_steps];

    // Initialize starting state 
    Node_Info default_node;
    std::vector<std::vector<Node_Info>> adjacency_list(x_size, std::vector<Node_Info>(y_size, default_node));
    
    std::cout << std::boolalpha ;
   

    // const state s_start(robotposeX,robotposeY,0.0,Heuristic_Fcn_Cost(robotposeX, robotposeY, goalposeX, goalposeY, map, x_size, y_size, weight),FLT_MAX,curr_time);
    const state s_start(robotposeX,robotposeY,0.0,Time_HFcn(robotposeX, robotposeY, goalposeX, goalposeY, target_steps, curr_time),FLT_MAX,curr_time);
    // const state s_start(robotposeX,robotposeY,0.0,Euclidean_dist(robotposeX, robotposeY, goalposeX, goalposeY),FLT_MAX,curr_time);
    // std::cout << "Starting state Parents: " << s_start.cell_x << ", " << s_start.cell_y << std::endl;

    // Goal Node  next set the heuristic to zero
    state s_goal(goalposeX,goalposeY,0.0,0,FLT_MAX,target_steps);
    
    // open_list.push_back(s_start); //Add state to initial list
    open_list.push(s_start); //Add state to initial list
    adjacency_list[s_start.cell_x - 1][s_start.cell_y - 1].is_explored = true;
    // open_list_1.push_back(s_start);
    
    // // Print the result of Time_HFcn
    // std::cout << "Time_HFcn result: " << s_start.h << std::endl;
    std::vector<std::pair<int, int>> parent_states; // Store (parent_x, parent_y) pairs
    bool found = false;
    bool found_map = false;
    // state goal_state(0,0,0.0,0, 0,0);
    
    if(planned_path == false){
        // std::cout << planned_path << std::endl;
        //Start the while loop 
        // while (!open_list.empty() && !found)
        while (!found_map)
        { 
            auto current_time = std::chrono::high_resolution_clock::now(); 
            std::chrono::duration<double> elapsed_seconds = current_time - start_time;
            // std::cout << "Time passed (s): " << elapsed_seconds.count() << std::endl;

            if (elapsed_seconds.count() > time_limit_seconds){
                // std::cout << "Time limit reached." << std::endl;
                break;
            }
            // std::cout << "Here" << std::endl;
            // found = (std::find(closed_list.begin(), closed_list.end(), s_goal) != closed_list.end());
            // std::cout << "Found: " << found << std::endl;

            //Check list for lowest f value:
            double min_f = std::numeric_limits<double>::max(); // Initialize to a high value

            state min_f_state = open_list.top();
            // size_t length_list_open = open_list.size();
            
            // auto goal_state_iterator = std::find(closed_list.begin(), closed_list.end(), s_goal);

            // closed_list.push_back(min_f_state);
            std::string state_identifier = std::to_string(min_f_state.cell_x) + "," +std::to_string(min_f_state.cell_y);
            closed_list_map[state_identifier] = &min_f_state;

            std::string goal_state_identifier = std::to_string(s_goal.cell_x) + "," + std::to_string(s_goal.cell_y);
            found_map = (closed_list_map.find(goal_state_identifier) != closed_list_map.end());

            if (closed_list_map.find(goal_state_identifier) != closed_list_map.end())
            {
                // Goal state found in closed_list
                state goal_state = *(closed_list_map[goal_state_identifier]);
                std::cout << "Goal state found: " << goal_state.cell_x << ", " << goal_state.cell_y << std::endl;
                found_map = true;
                // std::cout << "Goal state parent: " << goal_state.parent_x << ", " << goal_state.parent_y << std::endl;
                break;
            }

            open_list.pop();

            //Get the neighbors of the states
            std::vector<state> neighbors_ = expand_state(min_f_state, map,x_size,y_size,collision_thresh,target_steps,robotposeX,robotposeY,dX,dY);

            for (state& neighbor : neighbors_){
                // Check if the state is already in the closed list..if it is skip the neighbor
                // if (std::find(closed_list.begin(), closed_list.end(), neighbor) != closed_list.end()) {
                //     continue;
                // }

                // Update the g,h, f, and parents

                if (adjacency_list[neighbor.cell_x - 1][neighbor.cell_y - 1].is_explored == true)
                {
                    continue;
                } 
                double g_val = 0;
                // std::cout << "Map cost: " << map[GETMAPINDEX(neighbor.cell_x,neighbor.cell_y,x_size,y_size)] << std::endl;
                if ( neighbor.g > (min_f_state.g + map[GETMAPINDEX(neighbor.cell_x,neighbor.cell_y,x_size,y_size)]))
                {
                    g_val = min_f_state.g + map[GETMAPINDEX(neighbor.cell_x,neighbor.cell_y,x_size,y_size)]; //g(s") + c(s",s)
                    neighbor.g = g_val;
                }

                // std::cout << "neighbor_cell_updated g: " << neighbor.g << std::endl;
                // double h_val = Heuristic_Fcn_Cost(neighbor.cell_x, neighbor.cell_y, goalposeX, goalposeY, map, x_size, y_size, weight);

                // double h_val = Euclidean_dist(neighbor.cell_x, neighbor.cell_y,goalposeX,goalposeY);
                double h_val = Time_HFcn(min_f_state.cell_x, min_f_state.cell_y,goalposeX,goalposeY, target_steps,min_f_state.time);
                double f_val = g_val + h_val;

                neighbor.parent_x = min_f_state.cell_x;
                neighbor.parent_y = min_f_state.cell_y;

                // if (neighbor.cell_x == s_goal.cell_x && neighbor.cell_y == s_goal.cell_y){
                //     std::cout<< "Parents of goal: " << neighbor.parent_x << ", " << neighbor.parent_y << std::endl;
                // }

                adjacency_list[neighbor.cell_x - 1][neighbor.cell_y - 1].parents = std::make_pair(min_f_state.cell_x, min_f_state.cell_y);

                parent_states.push_back(std::make_pair(neighbor.parent_x, neighbor.parent_y));
                neighbor.f = f_val;
                neighbor.h = h_val;

                open_list.push(neighbor);
                adjacency_list[neighbor.cell_x - 1][neighbor.cell_y - 1].is_explored = true;
                
   
            }
            size_t length_list_open = open_list.size();
            // std::cout<< "# of states in open list:" << length_list_open << std::endl; 
            
        }
        // After the the goal state has been found, I need to trace back the path from the goal state to to the start state using the parent information stored in each state.
        // This will give me the optimal path. Once the optimal path is found, I can extract the next action from the path
        // Then update the action_ptr with the next action! 
        // size_t length_list_open = open_list.size();
        // std::cout<< "# of states in closed list:" << length_list_open << std::endl;
        // size_t length_list_closed = closed_list.size();
        // std::cout<< "# of states in closed list:" << length_list_closed << std::endl;
        
        // state last_state_added = closed_list.back();
        // std::cout << "Last state: " << last_state_added.cell_x << ", cell_y: " << last_state_added.cell_y << ", f: " << last_state_added.f << ", g: " << last_state_added.g << ", h: " << last_state_added.h << ", time: " << last_state_added.time << std::endl;

        //Set the goal state's parent
        // s_goal.parent_x = last_state_added.parent_x;
        // s_goal.parent_y = last_state_added.parent_y;

        
        // std::cout << "Goal State: " <<  s_goal.parent_x << ",  " <<  s_goal.parent_x << std::endl;
        
        int state_num = 0;
        if(found_map){
            state curr_state = s_goal; //Change back to last_state_Added if this doesn't work
            path_found.push_back(std::make_pair(curr_state.cell_x, curr_state.cell_y));
            while (!(curr_state.cell_x == s_start.cell_x && curr_state.cell_y == s_start.cell_y) && path_found.size() <= target_steps) {
                // Retrieve parent information from parent_states vector
                std::pair<int, int> parent = adjacency_list[curr_state.cell_x - 1][curr_state.cell_y - 1].parents;

                // int next_action_x = parent.first;
                // int next_action_y = parent.second;
                // std::cout << "Parent x: " << next_action_x << ", Parent y: " << next_action_y << std::endl;
                
                // // Update action_ptr
                // action_ptr[0] = next_action_x;
                // action_ptr[1] = next_action_y;

                curr_state.cell_x = parent.first;
                curr_state.cell_y = parent.second;

                // std::cout << "Parent x: " << curr_state.cell_x << ", Parent y: " << curr_state.cell_y << std::endl;
                path_found.push_back(parent);

                // state_num ++;
                // if (curr_state.cell_x == s_start.cell_x && curr_state.cell_y == s_start.cell_y)
                // {
                //     std::cout << "Path found" << std::endl;
                //     std::cout << "Number of states on path: " << state_num << std::endl;
                //     planned_path = true;
                //     break;
                    
            }
                std::reverse(path_found.begin(), path_found.end());
        }
            // std::cout << "Length of path: " << path_found.size() << std::endl;
            // std::cout << "Path var: " << planned_path << std::endl;y
            planned_path = true;
    }

    if (!path_found.empty() && path_found.size() <= target_steps){ //&& path_found.size() <= target_steps
        const std::pair<int, int>& next_step = path_found[current_path_index];
        
        action_ptr[0] = next_step.first;
        action_ptr[1] = next_step.second;
        // path_found.erase(std::remove(path_found.begin(), path_found.end(), path_found[0]), path_found.end()); //Delete the pair at the first index to update the path
        
        if (next_step.first == s_goal.cell_x && next_step.second == s_goal.cell_y){
            std::cout << "Planning is finished." << std::endl;
            planned_path = true;
            return;
        }
        current_path_index++;

    } else{
        if (path_found.size() > target_steps){
            int goalposeX = target_traj[target_steps-1];
            int goalposeY = target_traj[target_steps-1+target_steps];

            int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
            double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
            double disttotarget;
            for(int dir = 0; dir < NUMOFDIRS; dir++)
            {
                int newx = robotposeX + dX[dir];
                int newy = robotposeY + dY[dir];

                if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
                {
                    if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
                    {
                        disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                        if(disttotarget < olddisttotarget)
                        {
                            olddisttotarget = disttotarget;
                            bestX = dX[dir];
                            bestY = dY[dir];
                        }
                    }
                }
            }
            robotposeX = robotposeX + bestX;
            robotposeY = robotposeY + bestY;
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            std::cout <<  action_ptr[0] << ", " <<  action_ptr[1] << std::endl;
            
            return;
        }
    }
    
}
