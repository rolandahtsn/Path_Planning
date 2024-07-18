/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 
#include <iostream>
#include <chrono>
#include <ctime>
#include <unordered_map>
#include <queue>

#include "planner_header.hpp"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;
using namespace std;
double epsilon = 0.4;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
	
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

//Functions for ALL Plans; Report

double evaluate_plan(int numofDOFs, int* planlength, double*** plan) {
    
    int num_iter = *planlength - 1;
    double plan_performance = 0;

    for (int index = 0; index < num_iter; index++) {
        double squaredDifference = 0;
        for (int joint = 0; joint < numofDOFs; ++joint) {
            double angleDiff = (*plan)[index][joint] - (*plan)[index + 1][joint];
            squaredDifference += angleDiff * angleDiff;
        }
        plan_performance += sqrt(squaredDifference);
    }
    return plan_performance;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

struct Node {
    std::vector<double> jointAngles;
    Node* parent;
    int id;
    std::vector<Node*> children;
    double node_cost;

    Node() : parent(nullptr), id(-1), node_cost(0.0){}

    Node(const std::vector<double>& angles)
        : jointAngles(angles), parent(nullptr), id(-1), node_cost(0.0) {
        // Initialize other fields as needed
    }

    // Add != operator
    bool operator!=(const Node& other) const {
        return this->jointAngles != other.jointAngles;
    }
};

struct NodeStatus{
    Node* node; 
    // std::string status;
    bool Value;
};
class RRTTree {
    public:
        double *map;
        int x_size;
        int y_size;
        

        // Updated to work with the new Node structure
        std::vector<Node*> start; 
        std::vector<Node*> goal; 
        std::vector<double> start_;
        std::vector<double> goal_;

        int numofDOFs;

        RRTTree(int numofDOFs, double* map, int x_size, int y_size,std::vector<double> start,std::vector<double> goal)
            : numofDOFs(numofDOFs), map(map), x_size(x_size), y_size(y_size), start_(start), goal_(goal) {
        }
        void addNode( bool node, const Node& newNode, Node* parent);
        bool DisttoGoal(const Node& node1, const Node& node2);
        std::pair<bool, Node*> RRT_extension(const Node& targetNode, bool node);
        Node* findNearestNode(const Node& targetNode, bool node);
        Node* cons_RRT();
        
        std::vector<std::vector<double>> generate_path(Node* current_node) {
            std::vector<std::vector<double>> path;

            Node* node_in_path = current_node;
            while (node_in_path != nullptr) {
                path.push_back(node_in_path->jointAngles);
                node_in_path = node_in_path->parent;
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        
        /*
        Connect Functions
        */
        std::pair<Node*, Node*> cons_RRTConnect();
        Node* RRTconnect(const Node& targetNode, bool node);
        std::pair<std::vector<Node*>, std::vector<Node*>> generate_pathConnect(double ***plan, Node* start_node, Node* goal_node);
        /*
        RRT Star Functions
        */

        void addNode_RRTs(Node* parent_node, Node* child_node);
        Node* cons_RRTStar();
        Node* RRTS_extension(const Node& targetNode);
        Node* nearestNeighborStar(const Node& targetNode);
        vector<Node*> nearneighbors(const Node& newNode, double search_area);

        void AddEdgeStar(Node* cd_, Node* pt_);

    private:
        std::vector<Node*> tree;
        int Sample_;
};


double calculate_cost_total(const Node& node1, const Node& node2){
	double final_cost = 0.0;
	for(int i = 0; i < node1.jointAngles.size(); i++){
        double cost1 = (node1.jointAngles[i] - node2.jointAngles[i]);
        double cost2 = (node1.jointAngles[i] - node2.jointAngles[i]);
		final_cost += (node1.jointAngles[i] - node2.jointAngles[i]) * (node1.jointAngles[i] - node2.jointAngles[i]);
	}
    final_cost = sqrt(sqrt(final_cost));
	return final_cost;
}

Node* RRTTree::nearestNeighborStar(const Node& targetNode){
	Node* neighbor_node;
	double cost_least_node = std::numeric_limits<double>::max();

	for (int i = 0; i < start.size(); i++){
        Node* nearest_neighbor = start[i];
		double cost_tot = nearest_neighbor -> node_cost;
		if (cost_tot < cost_least_node){
			cost_least_node = cost_tot;
			neighbor_node = nearest_neighbor;
		}
	}
	return neighbor_node;
}

vector<Node*> RRTTree::nearneighbors(const Node& newNode, double search_area){
	std::vector<Node*> neigh_;
    double max_distance = std::numeric_limits<double>::max();

	for (int i = 0; i < start.size(); i++){
		double dist = 0.0;
        Node* nearest_node = start[i];
		for (int i = 0; i < numofDOFs; i++){
            double angles_node = nearest_node->jointAngles[i];
            double diff = angles_node - newNode.jointAngles[i];
			dist += diff*diff;
		}
		dist = sqrt(dist);
		if (dist < search_area && dist <= max_distance){
			neigh_.push_back(nearest_node);
		}
	}
	return neigh_;
}

void RRTTree::AddEdgeStar(Node* cd_, Node* pt_){
	cd_->parent = pt_;

    std::vector<double> child_joints = cd_->jointAngles;
    double current_node_cost = pt_->node_cost;

	cd_->node_cost = pt_->node_cost + calculate_cost_total(child_joints, child_joints);
}

void RRTTree::addNode_RRTs(Node* parent_node, Node* child_node){
	child_node->parent = parent_node;
    double total_cost = 0.0;

    auto angles1 = child_node->jointAngles;
    auto angles2 = parent_node->jointAngles;
	for(int i = 0; i < child_node->jointAngles.size(); i++){
		total_cost += (angles1[i] - angles2[i]) * (angles1[i] - angles2[i]);
	}
	total_cost = sqrt(total_cost);

    //update cost
	child_node->node_cost = parent_node->node_cost + total_cost;
}

Node* RRTTree::RRTS_extension(const Node& targetNode) {
    
    Node* nearestNode = findNearestNode(targetNode, true);
    Node newNode(targetNode.jointAngles);

    double distSquared = 0.0;
    for (int i = 0; i < numofDOFs; ++i) {
        double diff = targetNode.jointAngles[i] - nearestNode->jointAngles[i];
        distSquared += diff * diff;
    }

    double distance_norm = std::sqrt(distSquared);
    auto average = std::min(epsilon, distance_norm) / distance_norm;

    double interpolation_steps = (distance_norm == 0.0) ? 1.0 : average;
    bool reached_avd = false;
    double interp_by = interpolation_steps;
    
    for (; interp_by <= 1.0; interp_by += interpolation_steps) {
        Node store_angles;
        store_angles.jointAngles.reserve(numofDOFs);

        for (int i = 0; i < numofDOFs; ++i) {
            double interpolated_angle = nearestNode->jointAngles[i] + interp_by * (targetNode.jointAngles[i] - nearestNode->jointAngles[i]);
            store_angles.jointAngles.push_back(interpolated_angle);
        }
        bool configt_f = IsValidArmConfiguration(store_angles.jointAngles.data(), numofDOFs, map, x_size, y_size);
        if (configt_f) {
            newNode = store_angles;
            reached_avd = true;
        } else {
            break;
        }
    }

    if (reached_avd == true) {
        double minCost = nearestNode->node_cost + calculate_cost_total(*nearestNode, newNode);
        Node* mincostNode = nearestNode;

        std::vector<Node *> nodes = nearneighbors(newNode, 10);
        for (Node* nearest_node : nodes) {
            double init_cost = nearest_node->node_cost + calculate_cost_total(*nearest_node, newNode);
            if (init_cost < minCost) {
                minCost = init_cost;
                mincostNode = nearest_node;
            }
        }

        addNode(true, newNode, nearestNode);
        Node* new_q = start.back();

        new_q->node_cost = minCost;
        addNode_RRTs(mincostNode, new_q);

        for (size_t i = 0; i < nodes.size(); ++i) {
            Node* nearest_node = nodes[i];
            double init_cost = new_q->node_cost + calculate_cost_total(*new_q, *nearest_node);
            auto cost_node_neighbor = nearest_node->node_cost;
            if (init_cost < cost_node_neighbor) {
                cost_node_neighbor = init_cost;
                auto parent_update = nearest_node->parent;
                parent_update = new_q;
            }
        }
        return new_q;
    }
    return nullptr;
}

Node generateRandomNode(const Node& goalNode, int numofDOFs) {
    double randomBias = (double)rand() / RAND_MAX;
    Node randomNode;

    if (randomBias <= 0.1) {
        return goalNode;
    } else {
        std::vector<double> randomAngles(numofDOFs);

        std::generate(randomAngles.begin(), randomAngles.end(), []() {
            return ((double)rand() / RAND_MAX)*2*M_PI;
        });

        randomNode.jointAngles = randomAngles;
    }

    return randomNode;
}

Node* RRTTree::cons_RRTStar() {
    int Sample_ = 100000;
    Node startNode(start_); // Create the initial node with start joint angles
    addNode(true, startNode, nullptr);

    Node goalNode(goal_); // Create the goal node with goal joint angles

    for (int k = 0; k < 100000; k++) {
        Node randomNode = generateRandomNode(goalNode, numofDOFs); // Use generateRandomNode to get a random node
        Node* extended = RRTS_extension(randomNode);
        if (extended && DisttoGoal(*extended, goalNode)) {
            return extended;
        }
    }
    return nullptr;
}

//
void RRTTree::addNode( bool node, const Node& newNode, Node* parent) {
    Node* new_node = new Node(newNode);
    std::vector<Node*>& nodes = node ? start : goal;
    if (node) {
        nodes = start;
    } else {
        nodes = goal;
    }
    
    nodes.push_back(new_node);

    if (parent) {
        new_node->parent = parent;
    }
}


std::pair<bool, Node*> RRTTree::RRT_extension(const Node& targetNode, bool node) {
    Node* nearestNode = findNearestNode(targetNode, node);
    vector<Node*>& nodes = node ? start : goal;

    if (node) {
        nodes = start;
    } else {
        nodes = goal;
    }

    Node newNode(targetNode.jointAngles);

    double distSquared = 0.0;
    for (int i = 0; i < numofDOFs; ++i) {
        double diff = targetNode.jointAngles[i] - nearestNode->jointAngles[i];
        distSquared += diff * diff;
    }

    double distance_norm = std::sqrt(distSquared);
    auto average = std::min(epsilon, distance_norm) / distance_norm;

    double stepSize = (distance_norm == 0.0) ? 1.0 : average;
    bool reached_avd = false;
    double interp_steps = stepSize;

    while (interp_steps <= 1.0) {
        Node store_angles;
        store_angles.jointAngles.reserve(numofDOFs);

        for (int i = 0; i < numofDOFs; ++i) {
            double interpolated_angle = nearestNode->jointAngles[i] + interp_steps * (targetNode.jointAngles[i] - nearestNode->jointAngles[i]);
            store_angles.jointAngles.push_back(interpolated_angle);
        }

        if (IsValidArmConfiguration(store_angles.jointAngles.data(), numofDOFs, map, x_size, y_size)) {
            newNode = store_angles;
            reached_avd = true;
        } else {
            break;
        }

        interp_steps += stepSize;
    }


    if (reached_avd) {
        addNode(node, newNode, nearestNode);
        Node* newNodePtr = nodes.back();
        return std::make_pair(true, newNodePtr);
    }

    return std::make_pair(false, nearestNode);
}



Node* RRTTree::cons_RRT() {
    int Sample_ = 100000;
    Node initialNode(start_); // Create the initial node with start joint angles
    addNode(true, initialNode, nullptr);

    Node goalNode(goal_); // Create the goal node with goal joint angles

    for (int k = 0; k < 100000; k++) {
        Node randomNode = generateRandomNode(goalNode, numofDOFs);
        auto extensionResult = RRT_extension(randomNode, true); 

        if (DisttoGoal(std::get<1>(extensionResult)->jointAngles, goalNode.jointAngles)) {
            return std::get<1>(extensionResult);
        }
    }
    return nullptr;
}

pair<Node*, Node*> RRTTree::cons_RRTConnect() {
    bool node = true;
    int Sample_ = 100000;
    Node startNode(start_); // Initialize qInit with start joint angles
    addNode(true, startNode, nullptr);
    Node goalNode(goal_); // Initialize qGoal with goal joint angles
    addNode(false, goalNode, nullptr);

    for (int k = 0; k < 100000; k++) {
        Node randomNode = generateRandomNode(goalNode, numofDOFs);

        pair<bool, Node*> extended = RRT_extension(randomNode, node);
        Node* node_2 = extended.second;
        std::vector<double> angles_ = node_2->jointAngles;
        Node* node_in_path = RRTconnect(angles_, !node);

        if (node_in_path) {
            pair<Node*, Node*> pathEnds;
            
            if (node) {
                pathEnds.first = node_2;
                pathEnds.second = node_in_path;
            } else {
                pathEnds.first = node_in_path;
                pathEnds.second = node_2;
            }

            return pathEnds;
        }

        if (node == true){
            node = false;
        } else{
            node = true;
        }
    }
    std::pair<Node*, Node*> pair_1 = std::make_pair(nullptr, nullptr);
    return pair_1;
}


Node* RRTTree::RRTconnect(const Node& targetNode, bool node) {
    while (true) {
        auto connectResult = RRT_extension(targetNode, node);
    if (RRT_extension(targetNode, node).first && DisttoGoal(connectResult.second->jointAngles, targetNode.jointAngles)) {
        return connectResult.second;
    } else if (!RRT_extension(targetNode, node).first) {
        return nullptr;
    }
}

}

//Still need to change
std::pair<std::vector<Node*>, std::vector<Node*>> RRTTree::generate_pathConnect(double ***plan, Node* start_node, Node* goal_node) {
    std::vector<Node*> path1;
    std::vector<Node*> path2;

    Node* node_in_path = start_node;
    while (node_in_path != nullptr) {
        path1.push_back(node_in_path);
        node_in_path = node_in_path->parent;
    }

    node_in_path = goal_node;
    while (node_in_path != nullptr) {
        path2.push_back(node_in_path);
        node_in_path = node_in_path->parent;
    }
    return std::make_pair(path1,path2);
}

void copyPathToPlan(const vector<Node*>& startPath, const vector<Node*>& Goalpath, double ***plan, int *planlength, int numofDOFs){
    
    int k = 0;
    *planlength = startPath.size() + Goalpath.size();
	*plan = (double**) malloc(*planlength * sizeof(double*));

    bool reverse = true;
    int i = startPath.size() - 1;
    for (i; i >= 0; i--) {
        (*plan)[k] = (double*)malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; j++) {
            auto path_variable = startPath[i]->jointAngles[j];
            (*plan)[k][j] = path_variable;
        }
        k++;
    }
    int starting_index = k;
    for (int i = 0; i < Goalpath.size(); i++) {
        (*plan)[starting_index] = (double*)malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; j++) {
            auto path_variable2 = Goalpath[i]->jointAngles[j];
            (*plan)[starting_index][j] = path_variable2;
        }
        starting_index++;
    }
}

Node* RRTTree::findNearestNode(const Node& targetNode, bool node) {
    std::vector<Node*>& nodes = node ? start : goal;
    if (node) {
        nodes = start;
    } else {
        nodes = goal;
    }

    auto minDistNode = std::min_element(nodes.begin(), nodes.end(), [this, &targetNode](Node* a, Node* b) {
        double dist_a = 0.0;
        double dist_b = 0.0;

        for (size_t i = 0; i < numofDOFs; i++) {
            dist_a += std::pow(a->jointAngles[i] - targetNode.jointAngles[i], 2);
            dist_b += std::pow(b->jointAngles[i] - targetNode.jointAngles[i], 2);
        }

        dist_a = std::sqrt(dist_a);
        dist_b = std::sqrt(dist_b);

        return dist_a < dist_b;
    });

    return *minDistNode;
}


//Changed
bool RRTTree::DisttoGoal(const Node& node1, const Node& node2) {
    double dist = 0.0;
    double GOAL_TOLERANCE = 1e-4; //1e-3
    for (int i = 0; i < numofDOFs; ++i) {
        double delta = std::abs(node1.jointAngles[i] - node2.jointAngles[i]);
        dist += delta*delta;
    }
    double distance = sqrt(dist);
    if (distance <= GOAL_TOLERANCE){
        return true;
    }
    else{
        return false;
    }
}

static void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
	

    // auto planning_Start = chrono::high_resolution_clock::now();
	vector<double> start_angles(armstart_anglesV_rad, armstart_anglesV_rad+numofDOFs);
	vector<double> goal_angles(armgoal_anglesV_rad, armgoal_anglesV_rad+numofDOFs);
	RRTTree rrt(numofDOFs,map,x_size,y_size,start_angles,goal_angles);

	Node* extended = rrt.cons_RRT();
	
    //calculate_time

	// auto planning_done = chrono::high_resolution_clock::now();
	// auto time_to_plan = chrono::duration_cast<chrono::milliseconds>(planning_done - planning_Start);

	if (extended){
		std::vector<std::vector<double>> path = rrt.generate_path(extended);
        *planlength = path.size();
        *plan = (double**)malloc(path.size() * sizeof(double*));

        for (int j = 0; j < path.size(); ++j) {
            (*plan)[j] = (double*)malloc(numofDOFs * sizeof(double));
            for (int k = 0; k < numofDOFs; ++k) {
                (*plan)[j][k] = path[j][k];
            }
        }

	} else {
		std::cout << "no solution" << std::endl;
	}

	// auto finish = chrono::high_resolution_clock::now();
	// auto time_taken = chrono::duration_cast<chrono::milliseconds>(finish - planning_done);
	// int num_of_nodes = rrt.start.size() + rrt.goal.size();
    
	// double quality_angle_diff = evaluate_plan(numofDOFs, planlength, plan);

    //Print results
	// std::string success_in_5;
    // if (time_taken.count() < 5000) {
    //     success_in_5 = "Yes";
    // } else {
    //     success_in_5 = "No";
    // }
	// std::cout << "DOF: " << numofDOFs << ", Runtime: " << time_to_plan.count() << " milliseconds, Under 5 Seconds planning time?: " << success_in_5 << ", Number of nodes generated: " << num_of_nodes << ", Numerical grade: " << quality_angle_diff << ", Length " << *planlength << std::endl;
    
	return; 
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//
static void plannerRRTConnect(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    // auto planning_start = chrono::high_resolution_clock::now();

    // Initialize values
    vector<double> start_angles(armstart_anglesV_rad, armstart_anglesV_rad+numofDOFs);
	vector<double> goal_angles(armgoal_anglesV_rad, armgoal_anglesV_rad+numofDOFs);
	RRTTree rrt(numofDOFs,map,x_size,y_size,start_angles,goal_angles);

    pair<Node*, Node*> extended = rrt.cons_RRTConnect();
    auto first_pt = extended.first;
    auto second_pt = extended.second;

    // auto planning_done = chrono::high_resolution_clock::now();
    // auto time_to_plan = chrono::duration_cast<chrono::milliseconds>(planning_done - planning_start);
    
    if (first_pt && second_pt){
        //Get the path for rrt connect
		auto paths_connected = rrt.generate_pathConnect(plan,first_pt,second_pt);
        //(const vector<Node*>& startPath, const vector<Node*>& Goalpath, double ***plan, int *planlength, int numofDOFs)
        copyPathToPlan(paths_connected.first, paths_connected.second, plan, planlength, numofDOFs);
	} else {
		std::cout << "no solution" << std::endl;
	}

    // auto finish = chrono::high_resolution_clock::now();
	// auto time_taken = chrono::duration_cast<chrono::milliseconds>(finish - planning_done);
	// int num_of_nodes = rrt.start.size() + rrt.goal.size();
	// double quality_angle_diff = evaluate_plan(numofDOFs,planlength,plan);

    // //Print results
	// std::string success_in_5;
    // if (time_taken.count() < 5000) {
    //     success_in_5 = "Yes";
    // } else {
    //     success_in_5 = "No";
    // }
	// std::cout << "DOF: " << numofDOFs << ", Runtime: " << time_to_plan.count() << " milliseconds, Under 5 Seconds planning time?: " << success_in_5 << ", Number of nodes generated: " << num_of_nodes << ", Numerical Grade: " << quality_angle_diff << ", Length " << *planlength << std::endl;
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
	// auto planning_start = chrono::high_resolution_clock::now();

	//intialize values
	double epsilon = 0.5; // change this
	vector<double> start_angles(armstart_anglesV_rad, armstart_anglesV_rad+numofDOFs);
	vector<double> goal_angles(armgoal_anglesV_rad, armgoal_anglesV_rad+numofDOFs);
	RRTTree rrt(numofDOFs,map,x_size,y_size,start_angles,goal_angles);

	auto extended = rrt.cons_RRTStar();
	
	// auto planning_done = chrono::high_resolution_clock::now();
	// auto time_to_plan = chrono::duration_cast<chrono::milliseconds>(planning_done - planning_start);

	if (extended){
		std::vector<std::vector<double>> path = rrt.generate_path(extended);
        *planlength = path.size();
        *plan = (double**)malloc(path.size() * sizeof(double*));

        for (int j = 0; j < path.size(); ++j) {
            (*plan)[j] = (double*)malloc(numofDOFs * sizeof(double));
            for (int k = 0; k < numofDOFs; ++k) {
                (*plan)[j][k] = path[j][k];
            }
        }

	} else {
		std::cout << "no solution" << std::endl;
	}

	// auto finish = chrono::high_resolution_clock::now();

	// auto time_taken = chrono::duration_cast<chrono::milliseconds>(finish - planning_done);
	// int num_of_nodes = rrt.start.size() + rrt.goal.size();
	// double quality_angle_diff = evaluate_plan(numofDOFs,planlength,plan);
    // //Print results
	// std::string success_in_5;
    // if (time_taken.count() < 5000) {
    //     success_in_5 = "Yes";
    // } else {
    //     success_in_5 = "No";
    // }
	// std::cout << "DOF: " << numofDOFs << ", Runtime: " << time_to_plan.count() << " milliseconds, Under 5 Seconds planning time?: " << success_in_5 << ", Number of nodes generated: " << num_of_nodes << ", Numerical Grade: " << quality_angle_diff << ", Length " << *planlength << std::endl;
    
	return; 
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

PRMGraph::PRMGraph() : open(false), close(false) {}

void PRMGraph::angle_config(const vector<double>& gen_angles){
    	angles.assign(gen_angles.begin(), gen_angles.end());
}

double PRMGraph::generateRandomDouble(double min, double max) {
    double randomValue = (double)rand() / RAND_MAX;
    return min + randomValue * (max - min);
}

double minAngleDistanceSquared(double angle1, double angle2) {
    return std::min(pow(angle1 - angle2, 2), pow(2 * PI - fabs(angle1 - angle2), 2));
}

double PRMGraph::getDist(vector<double> angle_config1, vector<double> angle_config2) {
    double distance = 0.0;
    for (int i = 0; i < angle_config1.size(); i++) {
        distance += minAngleDistanceSquared(angle_config1[i], angle_config2[i]);
    }
    distance = sqrt(distance);
    return distance;
}

bool prm_valid_connections(PRMGraph* angle_config1, PRMGraph* angle_config2, double* map, int x_size, int y_size, int numSteps) {
    for (int step = 1; step <= numSteps; step++) {
        std::vector<double> interpolated;
        for (int joint = 0; joint < angle_config1->find_angles().size(); joint++) {
            double delta_j = angle_config2->find_angles()[joint] - angle_config1->find_angles()[joint];
            double stepSize;

            if (delta_j >= PI) {
                stepSize = delta_j - 2 * PI;
            } else if (delta_j <= -PI) {
                stepSize = delta_j + 2 * PI;
            } else {
                stepSize = delta_j;
            }

            double angle = angle_config1->find_angles()[joint] + (double)step * stepSize / numSteps;
            interpolated.push_back(angle);
        }

        bool val_1 = IsValidArmConfiguration(interpolated.data(), angle_config1->find_angles().size(), map, x_size, y_size);
        if (!val_1) {
            return false;
        }
    }
    return true;
}





static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    // Initialize data structures for searching
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> list_open_nodes;
    // stack<int, vector<int>> Astar_path;
    std::vector<int> Astar_path;

    *plan = nullptr;
    *planlength = 0;

	double step_size = numofDOFs / 20.0; //0.5
	double tol = 2.0 * PI /180;
    int const_val = 200*step_size*20;
	int sample_size = 1000 + const_val;
    

    vector<double> start_angles(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    vector<double> goal_angles(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
    PRMGraph prm;
    
	unordered_map<int, PRMGraph*> roadmap;
   
	PRMGraph* startNode = new PRMGraph();
	PRMGraph* endNode = new PRMGraph();
	vector<pair<double, int>> vector_dist = {};

	int k;
    int num_of_nodes = 0;
    bool start_config = IsValidArmConfiguration(start_angles.data(), numofDOFs, map, x_size, y_size);
    bool goal_config = IsValidArmConfiguration(goal_angles.data(), numofDOFs, map, x_size, y_size);
    int count = 0;

    int interpolation = int(200 * step_size);

    int node_inde_start = -1;
    int node_inde_goal = -2;
    if (start_config && goal_config) {

        auto planning_start = chrono::high_resolution_clock::now();

        for (; num_of_nodes < sample_size;) {
			PRMGraph* q_new = new PRMGraph();
			bool valid = false;

			while (!valid) {
				vector<double> q_rand;
				for (int m = 0; m < numofDOFs; m++) {
					double temp = prm.generateRandomDouble(0.0, 2*PI);
					q_rand.push_back(temp);
				}
                bool config_t_f = IsValidArmConfiguration(q_rand.data(), numofDOFs, map, x_size, y_size);
				if (config_t_f) {
					valid = true;
					q_new->set_index_nodes(num_of_nodes);
					q_new->angle_config(q_rand);
				}
			}

			vector<pair<double, int>> vector_dist;
			for (int i = 0; i < num_of_nodes; i++) {
                std::vector<double> node_angles = q_new->find_angles();
                std::vector<double> get_map_angles = roadmap.at(i)->find_angles();

				double dist = prm.getDist(node_angles, get_map_angles);
                std::pair<double, size_t> pair_1 = make_pair(dist, i);

				vector_dist.push_back(pair_1);
			}
			sort(vector_dist.begin(), vector_dist.end());
			
			k = 0;

			while (count < 3 && k < num_of_nodes) {
                int dist_ = vector_dist[k].second;
                bool is_connected = prm_valid_connections(q_new, roadmap.at(dist_), map, x_size, y_size, interpolation);
				if (is_connected) {
					q_new->add_connection(dist_);
					roadmap.at(dist_)->add_connection(num_of_nodes);
					count++;
				}
				k++;
			}

			roadmap[num_of_nodes] = q_new;
			num_of_nodes++;
		}

        startNode->angle_config(start_angles);
        startNode->set_index_nodes(node_inde_start);

        endNode->angle_config(goal_angles);
        endNode->set_index_nodes(node_inde_goal);

		

        bool connected = false;
		for (size_t i = 0; i < sample_size; i++) {
			auto start_config = startNode->find_angles();
			auto config_nodeMap = roadmap.at(i)->find_angles();

			double dist = prm.getDist(start_config, config_nodeMap);
			std::pair<double, size_t> pair_1 = make_pair(dist, i);
			
			vector_dist.push_back(pair_1);
		}

		sort(vector_dist.begin(), vector_dist.end());

		for (size_t k = 0; k < sample_size && !connected; k++) {
            int node_connection = vector_dist[k].second;
            bool prm_connected_ = prm_valid_connections(startNode, roadmap.at(node_connection), map, x_size, y_size, interpolation);

            if (prm_connected_) {
                startNode->add_connection(node_connection);
                roadmap.at(node_connection)->add_connection(node_inde_goal);
                connected = true;
            }
    }

        int parent_index = -1;
		startNode->new_parent(parent_index);

        //Insert the startNode into the
		roadmap[parent_index] = startNode; 


		for (int i = 0; i < sample_size; i++){
            auto end_config = endNode->find_angles();
			auto config_roadMap = roadmap.at(i)->find_angles();
			double distance = prm.getDist(end_config, config_roadMap);

			std::pair<double, size_t> pair_1 = make_pair(distance, i);
			vector_dist.push_back(pair_1);
		}
		sort(vector_dist.begin(), vector_dist.end());

		connected = false;
		for (int k = 0; k < sample_size && !connected; k++) {
            int node_connection = vector_dist[k].second;
            bool prm_connected_ = prm_valid_connections(startNode, roadmap.at(node_connection), map, x_size, y_size, interpolation);

            if (prm_connected_) {
                endNode->add_connection(node_connection);
                roadmap.at(node_connection)->add_connection(node_inde_goal);
                connected = true;
            }
        }

        roadmap[node_inde_goal] = endNode;

        list_open_nodes.emplace(make_pair(0.0, -1));

        while (!list_open_nodes.empty()) {
            pair<double, int> prior_node = list_open_nodes.top();
            int node_index = prior_node.second;
            int first_node = prior_node.first;
            list_open_nodes.pop();

            roadmap.at(node_index)->open_set();
            roadmap.at(node_index)->open_fs();

            for (int i = 0; i < roadmap.at(node_index)->isnode_connect().size(); i++) {
                int new_index = roadmap.at(node_index)->isnode_connect()[i];

                if (new_index == node_inde_goal) {
                    roadmap.at(new_index)->new_parent(node_index);

                    int current_index = -2;
                    int step_count = 0;
                    while (current_index != node_inde_start) {
                        Astar_path.push_back(current_index);
                        auto current_node = roadmap.at(current_index);
                        current_index = current_node->find_parent();
                        step_count++;
                    }
                    Astar_path.push_back(node_inde_start);

                    *planlength = step_count + 1;
                    *plan = (double**)malloc((step_count + 1) * sizeof(double*));

                    // for (int step = 0; step < step_count + 1; step++) {
                    //     (*plan)[step] = (double*)malloc(numofDOFs * sizeof(double));
                    //     auto current_angles = roadmap.at(Astar_path.back())->find_angles();
                    //     for (int i = 0; i < numofDOFs; i++) {
                    //         (*plan)[step][i] = current_angles[i];
                    //     }
                    //     Astar_path.pop_back();
                    // }

                    for (int step = 0; step < step_count + 1; step++) {
                        double* step_angles = (double*)malloc(numofDOFs * sizeof(double));
                        
                        // Get the current angles from the roadmap
                        auto current_angles = roadmap.at(Astar_path.back())->find_angles();
                        
                        // Copy the angles to the plan
                        for (int i = 0; i < numofDOFs; i++) {
                            step_angles[i] = current_angles[i];
                        }

                        // Add the step angles to the plan
                        (*plan)[step] = step_angles;
                        
                        // Remove the last element from Astar_path
                        Astar_path.pop_back();
                    }

                    auto finish = chrono::high_resolution_clock::now();
                    auto time_taken = chrono::duration_cast<chrono::milliseconds>(finish - planning_start);
                    double quality_angle_diff = evaluate_plan(numofDOFs,planlength,plan);
        
                    std::string success_in_5;
                    if (time_taken.count() < 5000) {
                        success_in_5 = "Yes";
                    } else {
                        success_in_5 = "No";
                    }

                    std::cout << "PRM, DOF: " << numofDOFs << ", Runtime: " << time_taken.count() << " milliseconds, Under 5 Seconds planning time?: " << success_in_5 << ", Number of nodes generated: " << num_of_nodes << ", Numerical Grade: " << quality_angle_diff << ", Length " << *planlength << std::endl;
                    return;
                }

                bool val_1 = roadmap.at(new_index)->val_close();
                bool val_2 =roadmap.at(new_index)->open_list();
                if (!val_1 && !val_2) {
                    double newnode = first_node + prm.getDist(roadmap.at(new_index)->find_angles(), roadmap.at(node_index)->find_angles());
                    roadmap.at(new_index)->new_parent(node_index);
                    std::pair<double, size_t> pair_index_node = make_pair(newnode, new_index);
                    
                    list_open_nodes.emplace(pair_index_node);
                    roadmap.at(new_index)->open_set();
                }
            }
        }
    }
    else {
        cout << "no solution\n";
    }
	return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
        // cout << "Start or goal state not matching" << endl;
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		// throw std::runtime_error("Start or goal position not matching");
        cout << "Start or goal state not matching" << endl;
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}