/**
*    @author Harikrishnan Lakshmanan
*    @file rrt_utilities.cpp
*    @date 11/21/2016
*
*    @brief WAVe Lab, File which describes everything about the RRT implementation.
*
*
*    @section Updates to make
*
*    1) Update to have the non holonomic properties by making use of the car model implemented (Dubin's path).
*
*    2) Generalize the code to be independednt of the state vector dimension.
*
*    3) Not necessary, but functions could be templatized, so that it facilitates debugging even
*    if the type of data is not explicitely defined as float, because '5' as a function argument
*    in place of float gives a compilation error sttaing the argument passed was 'int'. Even if
*    it is converted to '5.0', it might throw an error stating that, it is a double and hence won't
*    be compiled even though, there is no error.
*
*    @section Optimization Issues
*    I don't see any potential drawback, if required,
*    but arrays could be switched to vectors with initial capacity being mentioned.
*/

#include <math.h>       //atan2, sin, cos
#include <iostream>     //cout
#include <stdlib.h>     //random
#include <time.h>       //random
#include "rrt.h"        //header

//Node

//Constructor
Node::Node() {for (int i = 0; i < DIM; i++) {loc.push_back(0);}}

//Destructor
Node::~Node() {}

//Sets a new position for the self node as the default is {0,0}
/*void Node::set_pos(float* pos)
{
    loc[0] = *pos;
    loc[1] = *(pos+1);
}
*/
void Node::set_pos(std::vector <float> pos) {loc.assign(pos.begin(), pos.end());}

//Returns the parent of the given node
std::shared_ptr <Node> Node::get_parent()
{
    return parent;
}

//Sets the given node as the parent of self.
void Node::set_parent(std::shared_ptr <Node> par)
{
    parent = par;
}

//Returns the position of the self node. Fills the values in 'a' and 'b'.
/*void Node::get_pos(float &a, float &b)
{
    a = loc[0];
    b = loc[1];
}*/
void Node::get_pos(std::vector <float> &ret_pos) {ret_pos.assign(loc.begin(), loc.end());}


//Tree

//Constructor
Tree::Tree()
{
    root = NULL;
}
//Destructor
Tree::~Tree()
{
    //for (unsigned int i = 0; i < node_list.size(); i++)
    //{
    //    delete node_list[i];
    //}
    node_list.clear();
}

//Checking if tree is empty
int Tree::isempty()
{
    return (root == NULL);
}

/*
Returns the nearest node in the tree.
In the future, if the nodes go above 1000 or so, a Kd-tree implementation might be useful.
As of now, it uses linear search.
*/
std::shared_ptr <Node> Tree::get_nearest_node(std::shared_ptr <Node> random_node)
{
    std::shared_ptr <Node> nn = node_list[0];
    double node_dist = 0;
    double nn_dist = 0;

    for (unsigned int i = 0; i < node_list.size(); i++)
    {
        node_dist = get_distance(node_list[i], random_node);
        nn_dist = get_distance(nn, random_node);
        if ( node_dist < nn_dist ) nn = node_list[i];
    }
    return nn;
}

/*
Checks if the nearest node is closer than EPSILON distance to the random node.
If so, returns it, else, recursively adds elements until it reaches the random node.
Can be optimized from 99 to 110.
*/
std::shared_ptr <Node> Tree::step(std::shared_ptr <Node> nearest_node, std::shared_ptr <Node> random_node)
{
    if (get_distance(nearest_node, random_node) < EPSILON) return random_node;
    else
    {
        double angle = 0;
        std::vector <float> nearest_pos(DIM);
        std::vector <float> random_pos(DIM);
        std::vector <float> dir_vec(DIM);
        std::vector <float> connect_node_loc(DIM);
        std::shared_ptr <Node> node = std::make_shared <Node> ();

        nearest_node->get_pos(nearest_pos);
        random_node->get_pos(random_pos);
        double magnitude = 0.0;
        for (int i = 0; i < DIM; i++)
        {
            dir_vec.push_back(random_pos[i] - nearest_pos[i]);
            magnitude += pow(dir_vec[i],2);
        }

        for (int i = 0; i< DIM; i++) connect_node_loc.push_back((float) nearest_pos[i] + EPSILON*(dir_vec[i]/magnitude));
/**********************************************************************************
        angle = atan2((yr-yn),(xr-xn));
**********************************************************************************/
        std::shared_ptr <Node> connect_node = std::make_shared <Node> ();
/**********************************************************************************
        connect_node_loc.push_back(xn + EPSILON*cos(angle));
        connect_node_loc.push_back(yn + EPSILON*sin(angle));
**********************************************************************************/
        connect_node->set_pos(connect_node_loc);
        if (check_collision(connect_node))
        {
            node = this->insert_node(connect_node);
            if (node != NULL) return step(connect_node, random_node);
        }
        else return NULL;
    }
}

/*
Node Insertion

Overloaded functions
insert_node(Node* node) - If the argument is a node. All the connect nodes come through this.
insert_node(float* pos) - If the argument is the propective position of a node.
*/
std::shared_ptr <Node> Tree::insert_node(std::shared_ptr <Node> node)
{
    if (check_collision(node))
    {
        node->set_parent(get_nearest_node(node));
        node_list.push_back(node);
        return node;
    }
    else return NULL;
}

std::shared_ptr <Node> Tree::insert_node(std::vector <float> pos)
{
    if (check_collision(pos))
    {
        std::shared_ptr <Node> new_node = std::make_shared <Node> ();
        new_node->set_pos(pos);
        if (isempty())
        {
            root = new_node;
            root->set_parent(NULL);
        }
        else new_node->set_parent(get_nearest_node(new_node));
        node_list.push_back(new_node);
        return new_node;
    }
    else return NULL;
}

//Returns the distance between the two given nodes
double Tree::get_distance(std::shared_ptr <Node> nearest, std::shared_ptr <Node> random)
{
    std::vector <float> nearest_pos(DIM);
    std::vector <float> random_pos(DIM);
    nearest->get_pos(nearest_pos);
    random->get_pos(random_pos);
    double res = 0.0;

    for (int i = 0; i < DIM; i++) res += pow((random_pos[i]-nearest_pos[i]),2);
    return sqrt(res);
}

/*
Prints out all the elemnts in the tree.
Not necessary. Was created for debugging. Just leaving it over.
*/
void Tree::print_elements()
{
    std::vector <float> get_vec(DIM);
    std::cout<<"[";
    for (unsigned int i = 0; i < node_list.size(); i++)
    {
        node_list[i]->get_pos(get_vec);
        std::cout<<"("<<get_vec[0]<<", "<<get_vec[1]<<"), ";
    }
    std::cout<<"]";
}

/*
Collision Check

Overloaded functions
check_collision(float* pos) - If the argument is the propective position of a node,
return if the the position is in collision or not.

check_collision(Node* node) - If the argument is a node. Finds the position of the
node and calls the previous function. Created for coding simplicity.
*/
bool Tree::check_collision(std::vector <float> pos)
{
    if (pos[0] > 30.0 && pos[0] < 40.0 && pos[1] > 30.0 && pos[1] < 40.0) return 0;
    else return 1;
}

bool Tree::check_collision(std::shared_ptr <Node> node)
{
    std::vector <float> new_pos(DIM);
    node->get_pos(new_pos);
    return check_collision(new_pos);
}

std::shared_ptr <Path> Tree::get_path(std::shared_ptr <Node> last_node)
{
    std::shared_ptr <Path> new_path = std::make_shared <Path> ();
    for ( std::shared_ptr <Node> node = last_node; node != NULL; node = node->get_parent()) new_path->insert_node(node);
    return new_path;
}

Path::Path()
{
    head = NULL;
    tail = NULL;
}

Path::~Path() {}

void Path::insert_node(std::shared_ptr <Node> node)
{
    if (tail == NULL)
    {
        tail = node;
        head = tail;
    }
    else
    {
        head->parent = node;
        head = head->parent;
    }
}

void Path::print_path()
{
    std::vector <float> ret_pos(DIM);
    Node temp;
    temp = *tail;
    temp.get_pos(ret_pos);
    std::cout<<ret_pos[0]<<", "<<ret_pos[1]<<std::endl;
    while (temp.parent != NULL)
    {
        temp = *temp.parent;
        temp.get_pos(ret_pos);
        std::cout<<ret_pos[0]<<", "<<ret_pos[1]<<std::endl;
    }
}

float random (int low, int high) {
    if (low > high) return high;
    return low + (rand() % (high - low + 1));
}
