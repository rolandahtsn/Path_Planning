#pragma once

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

const double MIN_JOINT_ANGLE = -M_PI; //Are these angles correct?
const double MAX_JOINT_ANGLE = M_PI;

class PRMGraph {

public:
    PRMGraph();
    
    bool open_list() { return open; };
	double getDist(vector<double> angle_config1, vector<double> angle_config2);
    double generateRandomDouble(double minimumf, double maximumf);
	
    bool val_close() { return close; };
    vector<double> find_angles() { return this->angles; };

    
    int getID() { return index; };

    vector<int> isnode_connect() { return node_connection; };

    // void setAngles(vector<double> AnglesTobeset) { Angles = AnglesTobeset; };
	void angle_config(const vector<double>& gen_angles);

    void add_connection(int con) { node_connection.push_back(con); };
    void set_index_nodes(int index_set) { index = index_set; };

    //Parent functions:
    void new_parent(int index_parent) {
        Parent = index_parent; 
    };

    int find_parent() { 
        return Parent; 
    };

    //Functions for open and closed sets
    void open_set(){
        open = true; 
    };
    void open_fs() {
        open = false; 
    };

    void closeSetTrue() {
        close = true;
    };
    void closeSetFalse() {
        close = false;
    };

private:
    int index;
    vector<int> node_connection;
    vector<double> angles;
    int Parent;
    double G;

    bool open;
    bool close;

};
