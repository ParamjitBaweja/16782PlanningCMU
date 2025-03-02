/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <math.h>
#include <queue>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <iostream>


#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

struct Node {
    int x, y;
    double g, h;
    Node* parent;
    Node(int x, int y, double g, double h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}
    double f() const { return g + h; }
};

auto heuristic = [](int x1, int y1, int x2, int y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
};

auto compare = [](Node* a, Node* b) {
    return a->f() > b->f();
};


std::vector<Node*> path;
int path_time = 0;
int valid_path = 0;


int explore_env=1;


int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

void findpaths(
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
    int* action_ptr,
    float e = 1.0
    )
{
    valid_path = 0;
    std::priority_queue<Node*, std::vector<Node*>, decltype(compare)> open(compare);
    std::unordered_set<int> closed;
    std::unordered_map<int, Node*> allNodes;

    int goalposeX = targetposeX;
    int goalposeY = targetposeY;
    // printf("goalposeX: %d, goalposeY: %d\n", goalposeX, goalposeY);

    Node* start = new Node(robotposeX, robotposeY, 0, heuristic(robotposeX, robotposeY, goalposeX, goalposeY));
    open.push(start);
    allNodes[GETMAPINDEX(robotposeX, robotposeY, x_size, y_size)] = start;    

    Node* goal = nullptr;

    int goal_flag = 0;

    while (!open.empty()) {
        Node* current = open.top();
        open.pop();

        if (current->x == goalposeX && current->y == goalposeY) {
            goal = current;
            goal_flag = 1;
            break;
        }

        closed.insert(GETMAPINDEX(current->x, current->y, x_size, y_size));

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int newx = current->x + dX[dir];
            int newy = current->y + dY[dir];

            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {
                int index = GETMAPINDEX(newx, newy, x_size, y_size);
                if (closed.find(index) == closed.end() && map[index] >= 0 && map[index] < collision_thresh) {
                    double g = current->g + map[index];
                    double h = e * heuristic(newx, newy, goalposeX, goalposeY);
                    // printf("h: %f\n", h);
                    if (allNodes.find(index) == allNodes.end() || g < allNodes[index]->g) {
                        Node* neighbor = new Node(newx, newy, g, h, current);
                        open.push(neighbor);
                        allNodes[index] = neighbor;
                    }
                }
            }
        }
    }

    if (goal_flag == 1){
        path_time = 0;
        int temp_time = curr_time;
        path.clear();
        Node* current = goal;
        valid_path = 1; 
        while (current) {
            path.push_back(current);
            temp_time+=1;
            current = current->parent;
        }
        path_time = temp_time;
        // printf("path_time_loop****************: %d\n", path_time);
        path.pop_back(); 
    }

    return;
}

void multigoal_manager(
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
    int goal_estimate = curr_time+round(heuristic(robotposeX, robotposeY, targetposeX, targetposeY));
    if (goal_estimate >= target_steps) {
        goal_estimate = target_steps-1;
    }
    int start = goal_estimate;
    int end = target_steps-1;
    // printf("start: %d, end: %d\n", start, end);

    int path_flag =0;
    int target_index = round((start+end)/2);
    int best_time = 0;
    // printf("target_index: %d\n", target_index);
    while (target_index > start && target_index <end){
        int time_left = target_index - curr_time - 1;
        // printf("time_left: %d\n", time_left);
        int targetposeX = target_traj[target_index];
        int targetposeY = target_traj[target_index+target_steps];
        // printf("targetposeX: %d, targetposeY: %d\n", targetposeX, targetposeY);
        findpaths(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time, action_ptr);
        if (valid_path == 1){
            path_flag = 1;
        }
        if (path_time < time_left){
            end = target_index;
        }
        else{
            start = target_index;
        }
        // printf("path_time: %d\n", path_time);
        best_time = path_time;
        target_index = round((start+end)/2);
        // printf("target_index: %d\n", target_index);
        // printf("goal_estimate: %d\n", goal_estimate);
        // printf("start: %d, end: %d\n", start, end);
    }
    // printf("best_time: %d\n", best_time);
    if (best_time > (target_steps - curr_time - 5) || path_flag == 0){
        // printf("changing heuristic\n");
         
        int e = 100;
        targetposeX = target_traj[target_steps-1];
        targetposeY = target_traj[target_steps-1+target_steps];
        findpaths(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time, action_ptr, e);
        // int start = 200;
        // int end = 1;
        // int e = round(start+end)/2;
        // while (e>start && e<end)
        // {
            
        //     targetposeX = target_traj[target_steps-1];
        //     targetposeY = target_traj[target_steps-1+target_steps];
        //     findpaths(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time, action_ptr, e);
        // }

    }

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
    
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    // A* algorithm implementation


    if (explore_env){
        multigoal_manager(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time, action_ptr);
        explore_env =0;
    }
    if (!path.empty()) {
        Node* next_node = path.back();
        path.pop_back();
        action_ptr[0] = next_node->x;
        action_ptr[1] = next_node->y;
        delete next_node;
    }else
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }
    
    return;

}
