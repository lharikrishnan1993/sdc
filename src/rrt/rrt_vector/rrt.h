/**
*    @author Harikrishnan Lakshmanan
*    @file rrt.h
*    @date 11/21/2016
*
*    @brief WAVe Lab, Header file which describes everything about the RRT implementation.
*
*    @section Updates to make
*    Check the updates to be made in rrt_utilities.cpp
*
*    @section Optimization Issues
*    I don't see any potential drawback
*/

#ifndef RRT_H
#define RRT_H

#include <vector>
const double EPSILON = 5.0;
const int DIM = 2;

class Node
{
    private:
        //float loc[2] = {0, 0};
        std::vector <float> loc;
        //static int counter;

    public:
        std::shared_ptr <Node> parent;// = std::make_shared <Node> ();
        Node();
        ~Node();

        //void set_pos(float* pos);
        void set_pos(std::vector <float> pos);
        //void get_pos(float &a, float &b);
        void get_pos(std::vector <float> &ret_pos);

        void set_parent( std::shared_ptr <Node> );
        std::shared_ptr <Node> get_parent();
};

class Path
{
    private:
        std::shared_ptr <Node> head = std::make_shared <Node> ();
        std::shared_ptr <Node> tail = std::make_shared <Node> ();

    public:
        Path();
        ~Path();
        void insert_node(std::shared_ptr <Node>);
        void print_path();
};

class Tree
{
    private:
        std::shared_ptr <Node> root = std::make_shared <Node> ();
        std::vector <std::shared_ptr <Node>> node_list;

    public:
        Tree();
        ~Tree();

        int isempty();

        std::shared_ptr <Node> insert_node(std::shared_ptr <Node>);
        //std::shared_ptr <Node> insert_node(float* pos);
        std::shared_ptr <Node> insert_node(std::vector <float> pos);

        bool check_collision(std::shared_ptr <Node>);
        //bool check_collision(float* pos);
        bool check_collision(std::vector <float> pos);

        std::shared_ptr <Node> get_nearest_node(std::shared_ptr <Node>);
        double get_distance(std::shared_ptr <Node> , std::shared_ptr <Node>);
        std::shared_ptr <Node> step(std::shared_ptr <Node>, std::shared_ptr <Node>);

        void print_elements();
        std::shared_ptr <Path> get_path(std::shared_ptr <Node> last_node);
};

#endif
