/**
*    @author Harikrishnan Lakshmanan
*    @file naive_rrt.cpp
*    @date 11/21/2016
*
*    @brief WAVe Lab, File which holds the main code for the Naive RRT implementation.
*
*    @section Updates to make
*    I don't see any potential drawback as eventually there would be parallel files
*    which would handle the other variants of RRT.
*
*    @section Optimization Issues
*    I don't see any potential drawback
*/

#include <iostream>                     //cout
#include <boost/lexical_cast.hpp>       //lexical_cast
#include "rrt_utilities.cpp"            //Major chunk of code
//#include "rrt.h"

std::shared_ptr <Path> rrt_code(Tree *rrt, float &a, float &b, float &c, float &d)
{
    std::shared_ptr <Path> final_path = std::make_shared <Path> ();
    float xmin = 0, ymin = 0, xmax = 500, ymax = 500;

    if (a < xmin || a < ymin || a > xmax || a > ymax || b < xmin || b < ymin || b > xmax || b > ymax
    || c < xmin || c < ymin || c > xmax || c > ymax || d < xmin || d < ymin || d > xmax || d > ymax)
    {
        std::cout<<"Values out of bounds"<<std::endl;
    }

    float init[2] = {a, b};
    float dest[2] = {c, d};
    int node_limit = 5000;
    int node_counter = 0;

    std::shared_ptr <Node> temp = std::make_shared <Node> ();
    std::shared_ptr <Node> goal = std::make_shared <Node> ();
    std::shared_ptr <Node> temp_nearest_node = std::make_shared <Node> ();
    temp->set_pos(dest);

    srand((unsigned)time(NULL));

    rrt->insert_node(init);
    while (node_counter < node_limit)
    {
        node_counter += 1;
        if ((int) rrt->get_distance(rrt->get_nearest_node(temp), temp) > EPSILON)
        {
            float loc[2] = {random(xmin, xmax), random(ymin, ymax)};
            std::shared_ptr <Node> random_node = std::make_shared <Node> ();
            random_node->set_pos(loc);
            temp_nearest_node = rrt->get_nearest_node(random_node);
            random_node = rrt->step(temp_nearest_node, random_node);
            if (random_node != NULL) rrt->insert_node(random_node);
        }
        else break;
    }

    goal = rrt->insert_node(dest);
    final_path = rrt->get_path(goal);

    return final_path;
}

int main(int argc, char *argv[])
{
    if (argc < 5)
    {
        std::cout<<"./naive_rrt <start x> <start y> <goal x> <goal y>"<<std::endl;
        std::cout<<"Too less arguments"<<std::endl;
        return 0;
    }

    float a = boost::lexical_cast<float>(*(argv+1)), b = boost::lexical_cast<float>(*(argv+2));
    float c = boost::lexical_cast<float>(*(argv+3)), d = boost::lexical_cast<float>(*(argv+4));

    Tree *rrt = new Tree;
    std::shared_ptr <Path> path_op = rrt_code(rrt, a, b, c, d);
    std::cout<<"Reached back"<<std::endl;
    path_op->print_path();
    delete rrt;
    return 0;
}
