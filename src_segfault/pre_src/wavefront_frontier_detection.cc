#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include "wavefront_frontier_detection.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

const int MAP_OPEN_LIST = 1, MAP_CLOSE_LIST = 2, FRONTIER_OPEN_LIST = 3, FRONTIER_CLOSE_LIST = 4;
const int N_S = 8;

static int OCCUPIED = 0; 
static int UNKNOWN = -1;

// XXX: tunable
static double OCC_THRESHOLD_OLD = 800;
static double OCC_THRESHOLD = 0.0;
static int MIN_FOUND = 1; // how many free neighbors would be considered as a frontier point
static int MIN_FRONTIER = 1; // how many free neighbors would be considered as a frontier point

vector<vector<int> > wfd(const nav_msgs::OccupancyGrid& map, 
                         int map_height, int map_width, int pose, 
                         int t_occupied, int t_unknown) {	
    OCCUPIED = t_occupied;
    UNKNOWN = t_unknown;

    // cout << "occupied: " << OCCUPIED << endl;
    // cout << "unknown: " << UNKNOWN << endl;

	vector<vector<int> > frontiers;
	// Cell state list for map/frontier open/closed
	int map_size = map_height * map_width;

    if (pose > map_size) {
        ROS_WARN_STREAM ("pose out of map range!!");
        ROS_WARN_STREAM ("map: " << map_size << ", pose: " << pose);
        return frontiers;
    }
	std::map<int, int> cell_states;
	//
	queue<int> q_m;	
	q_m.push(pose);
	cell_states[pose] = MAP_OPEN_LIST; // mark pose as “Map-Open-List”
	int adj_vector[N_S];
	int v_neighbours[N_S];
	//
	//ROS_INFO("wfd 1");
	while(!q_m.empty()) {
		//ROS_INFO("wfd 2");
		int cur_pos = q_m.front();
		q_m.pop();
        if (cur_pos < 0) continue;
		//ROS_INFO("cur_pos: %d, cell_state: %d",cur_pos, cell_states[cur_pos]);
		// Skip if map_close_list
		if(cell_states[cur_pos] == MAP_CLOSE_LIST)
			continue;
		if(is_frontier_point(map, cur_pos, map_size, map_width)) {
            queue<int> q_f;
			vector<int> new_frontier;
			q_f.push(cur_pos);
			cell_states[cur_pos] = FRONTIER_OPEN_LIST; // mark p as “Frontier-Open-List”
			// Second BFS
			while(!q_f.empty()) { // EXTRACT-FRONTIER-2D
				//ROS_INFO("wfd 3");
				//ROS_INFO("Size: %d", q_f.size());
				int n_cell = q_f.front();
				q_f.pop();
				//
				if(cell_states[n_cell] == MAP_CLOSE_LIST || cell_states[n_cell] == FRONTIER_CLOSE_LIST)
					continue;
				//
				if(is_frontier_point(map, n_cell, map_size, map_width)) {
					//ROS_INFO("adding %d to frontiers", n_cell);
					new_frontier.push_back(n_cell);

                    // get all neighbors of current frontier point
					get_neighbours(adj_vector, cur_pos, map_width);
					//
					//ROS_INFO("wfd 3.5");
					for(int i = 0; i < N_S; i++) {
						if(adj_vector[i] < map_size && adj_vector[i] >= 0) {
							if(cell_states[adj_vector[i]] != FRONTIER_OPEN_LIST && 
							   cell_states[adj_vector[i]] != FRONTIER_CLOSE_LIST && 
							   cell_states[adj_vector[i]] != MAP_CLOSE_LIST) {
							   //ROS_INFO("wfd 4");
                                if(map.data[adj_vector[i]] != OCCUPIED) {
                                    q_f.push(adj_vector[i]);
                                    // mark w as “Frontier-Open-List”
                                    cell_states[adj_vector[i]] = FRONTIER_OPEN_LIST;
                                }
							}
						}
					}
				}
                // mark q as “Frontier-Close-List”
				cell_states[n_cell] = FRONTIER_CLOSE_LIST;
			}
			if(new_frontier.size() > MIN_FRONTIER)
				frontiers.push_back(new_frontier);
			
            // mark all points of NewFrontier as “Map-CloseList”
			//ROS_INFO("WFD 4.5");
			for(unsigned int i = 0; i < new_frontier.size(); i++) {
				cell_states[new_frontier[i]] = MAP_CLOSE_LIST;
				//ROS_INFO("WFD 5");
			}
		}
		//
		get_neighbours(adj_vector, cur_pos, map_width);

		for (int i = 0; i < N_S; ++i) {
			//ROS_INFO("wfd 6");
			if(adj_vector[i] < map_size && adj_vector[i] >= 0) {
				if(cell_states[adj_vector[i]] != MAP_OPEN_LIST &&  cell_states[adj_vector[i]] != MAP_CLOSE_LIST) {
					get_neighbours(v_neighbours, adj_vector[i], map_width);
					bool map_open_neighbor = false;
					for(int j = 0; j < N_S; j++) {
						if(v_neighbours[j] < map_size && v_neighbours[j] >= 0) {
                            // v has at least one “MapOpen-Space” neighbor
							// if(map.data[v_neighbours[j]] > OCC_THRESHOLD && map.data[v_neighbours[j]] != UNKNOWN) { //>= 0 AANPASSING // from paper's perspective
                            if(map.data[v_neighbours[j]] < OCC_THRESHOLD_OLD && map.data[v_neighbours[j]] >= 0) {
								map_open_neighbor = true;
								break;
							}
						}
					}
					if(map_open_neighbor) {
						q_m.push(adj_vector[i]);
						cell_states[adj_vector[i]] = MAP_OPEN_LIST; // mark v as “Map-Open-List”
					}
				}
			}
		}
		//ROS_INFO("wfd 7");
		cell_states[cur_pos] = MAP_CLOSE_LIST;
		//ROS_INFO("wfd 7.1");
	}
	// ROS_INFO("wfd 8");
	return frontiers;
}

void get_neighbours(int n_array[], int position, int map_width) {
	n_array[0] = position - map_width - 1;
	n_array[1] = position - map_width; 
	n_array[2] = position - map_width + 1; 
	n_array[3] = position - 1;
	n_array[4] = position + 1;
	n_array[5] = position + map_width - 1;
	n_array[6] = position + map_width;
	n_array[7] = position + map_width + 1;
}

void get_big_neighbours(int n_array[], int position, int map_width) {
	n_array[0] = position - map_width - 1;
	n_array[1] = position - map_width; 
	n_array[2] = position - map_width + 1; 
	n_array[3] = position - 1;
	n_array[4] = position + 1;
	n_array[5] = position + map_width - 1;
	n_array[6] = position + map_width;
	n_array[7] = position + map_width + 1;

	n_array[8] = position - (map_width * 2) - 2;
	n_array[9] = position - (map_width * 2) - 1; 
	n_array[10] = position - (map_width * 2); 
	n_array[11] = position - (map_width * 2) + 1;
	n_array[12] = position - (map_width * 2) + 2;
	n_array[13] = position - 2;
	n_array[14] = position + 2;
	n_array[15] = position + (map_width * 2) - 2;
	n_array[16] = position + (map_width * 2) - 1; 
	n_array[17] = position + (map_width * 2); 
	n_array[18] = position + (map_width * 2) + 1;
	n_array[19] = position + (map_width * 2) + 2;
	n_array[20] = position + (map_width) + 2;
	n_array[21] = position + (map_width) - 2;
	n_array[22] = position - (map_width) + 2;
	n_array[23] = position - (map_width) - 2;
}


bool is_frontier_point(const nav_msgs::OccupancyGrid& map, int point, int map_size, int map_width) {
	// The point under consideration must be known <- should've been unknown
	// if(map.data[point] != -1) {
	// if(map.data[point] == -1 || map.data[point] == 0) {
	// if(map.data[point] != UNKNOWN || point < 0) { // from paper's perspective
	if(map.data[point] == UNKNOWN || point < 0) {
		return false;
	}
	
	int locations[N_S]; 
	get_neighbours(locations, point, map_width);
	int found = 0;
	for(int i = 0; i < N_S; i++) {
		if(locations[i] < map_size && locations[i] >= 0) {
			// None of the neighbours should be occupied space.		
			// if(map.data[locations[i]] <= OCC_THRESHOLD) {
            if(map.data[locations[i]] > OCC_THRESHOLD_OLD) {
				return false;
			}
			//At least one of the neighbours is open and known space, hence frontier point
			// if(map.data[locations[i]] > OCC_THRESHOLD) { // from paper's perspective
			// if(map.data[locations[i]] > OCC_THRESHOLD && map.data[locations[i]] != UNKNOWN) {
			if(map.data[locations[i]] == UNKNOWN) {
				found++;
				//
				if(found == MIN_FOUND) 
					return true;
			}
			//}
		}
	}
	return false;
}

int get_row_from_offset(int offset, int width) {
	return offset / width;
}

int get_column_from_offset(int offset, int width) {
	return offset % width;	
}
