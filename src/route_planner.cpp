#include "route_planner.h"
#include <algorithm>
#include <cmath>
#include <set>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
	//convert inputs to %
  	start_x *=0.01;
   	start_y *=0.01;
   	end_x *=0.01;
   	end_y *=0.01;
  start_node =&m_Model.FindClosestNode(start_x,start_y);
  end_node=&m_Model.FindClosestNode(end_x,end_y);
}
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node){
      //create path found vector
      distance =0.0f;
      std::vector<RouteModel::Node> path_found;
      RouteModel::Node parent;
      while(current_node->parent !=nullptr){
      	path_found.push_back(*current_node); //dereference current node reference 
        parent=*(current_node->parent);
        //count up the distance between each node and parent till reach start_node
        distance+=current_node->distance(parent);
        current_node =current_node->parent;
      }
      path_found.push_back(*current_node);
      //convert distance to meters
      distance *=m_Model.MetricScale();
      return path_found;
}
 void RoutePlanner::AStarSearch(){
  //initialize open_list with starting node
start_node->visited=true;
open_list.push_back(start_node); 
RouteModel::Node *current_node = nullptr;
//Expand nodes until you reach the goal. Use heuristic to prioritize what node to oen first
while(open_list.size() > 0){
	//select the best node to explore next 
	current_node =NextNode();
	//check if the node selected is the goal, i.e. we reached the end node
	if (current_node->distance(*end_node) ==0){
		//Set the model path variable with the path found
		m_Model.path=ConstructFinalPath(current_node);
		return;
	}
	//you should add neighbors to the current_node if it is not in the final/goal node
	AddNeighbors(current_node);
	}
}
RouteModel::Node *RoutePlanner::NextNode(){ //lambda function, loop through all open_list
std::sort(open_list.begin(), open_list.end(), [](const auto &_1st, const auto &_2nd){
return _1st->h_value + _1st->g_value < _2nd->h_value + _2nd->g_value ;
});
RouteModel::Node * lowest_node =open_list.front(); //front return node data
open_list.erase(open_list.begin()); //begin() is the iterator that points to beginning of this vector 
return lowest_node;
}
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node){
//Expand the current node (add all unvisisted neighbors to the open_list)
current_node->FindNeighbors();
for(auto neighbor: current_node->neighbors){
	neighbor->parent =current_node; //previous in the list
	neighbor->g_value =current_node->g_value + current_node->distance(*neighbor); //calculate next step g value, g_value not only +1 like in the dashboard to the current g_value but it is the g_value of the current node and add to it the distance between the neighbor and current node
	neighbor->h_value =CalculateHValue(neighbor);
	//Add the neighbor to the open list
	open_list.push_back(neighbor);
	neighbor->visited = true;
	}
}
float RoutePlanner::CalculateHValue(const RouteModel::Node  *node){
return node->distance(*end_node);
}
