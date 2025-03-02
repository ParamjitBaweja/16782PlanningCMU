/*=================================================================
 *
 * planner.cpp
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
#include <queue>

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
#define	TREE_SIZE_OUT	plhs[2]

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

#ifndef MAPS_DIR
#define MAPS_DIR "../maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "../output"
#endif


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

void planner(
    double* map,
	int x_size,
	int y_size,
	double* armstart_anglesV_rad,
	double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength,
	int* tree_size)
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
	*tree_size = numofsamples;
    
    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

// static void plannerRRT(
//     double *map,
//     int x_size,
//     int y_size,
//     double *armstart_anglesV_rad,
//     double *armgoal_anglesV_rad,
//     int numofDOFs,
//     double ***plan,
//     int *planlength)
// {
//     /* TODO: Replace with your implementation */
    
// }

void logTreeSize(int* tree_size) {
	std::ofstream csvFile;
	csvFile.open("tree_size.csv", std::ios::app); // Open in append mode
	if (!csvFile.is_open()) {
		throw std::runtime_error("Cannot open CSV file");
	}
	csvFile << *tree_size << std::endl;
	csvFile.close();
}
struct Node {
	vector<double> angles;
	Node* parent;
	double cost;
	Node(vector<double> a, Node* p = nullptr) : angles(a), parent(p) {}
};

vector<double> sampleRandomConfig(int numofDOFs) {
	vector<double> config(numofDOFs);
	for (int i = 0; i < numofDOFs; ++i) {
		config[i] = ((double) rand() / RAND_MAX) * 2 * PI;
	}
	return config;
}

double distance(const vector<double>& a, const vector<double>& b) {
	double dist = 0;
	for (int i = 0; i < a.size(); ++i) {
		// dist += pow(a[i] - b[i], 2);
		double diff = a[i] - b[i];
        // Normalize angle difference to [-pi, pi]
        while (diff > PI) diff -= 2 * PI;
        while (diff < -PI) diff += 2 * PI;
        dist += diff * diff;
	}
	return sqrt(dist);
}

Node* nearestNeighbor(const vector<Node*>& tree, const vector<double>& config) {
	Node* nearest = nullptr;
	double minDist = std::numeric_limits<double>::max();
	for (Node* node : tree) {
		double dist = distance(node->angles, config);
		if (dist < minDist) {
			minDist = dist;
			nearest = node;
		}
	}
	// printf("minDist: %f\n", minDist);
	// printf("nearest: ");
	// for (int i = 0; i < config.size(); ++i) {
	// 	printf("%f ", nearest->angles[i]);
	// }
	return nearest;
}

// vector<double> steer(const vector<double>& from, const vector<double>& to, double stepSize) {
// 	vector<double> newConfig(from.size());
// 	double dist = distance(from, to);
// 	for (size_t i = 0; i < from.size(); ++i) {
// 		newConfig[i] = from[i] + stepSize * (to[i] - from[i]) / dist;
// 	}
// 	return newConfig;
// }
// Helper function to normalize angles to [-pi, pi]
void normalizeAngles(std::vector<double>& config) {
    for (double& angle : config) {
        while (angle > PI) angle -= 2 * PI;
        while (angle < -PI) angle += 2 * PI;
    }
}
std::vector<double> interpolateConfiguration(const std::vector<double>& from, const std::vector<double>& to, double step) {
    std::vector<double> newConfig(from.size());
	double dist = distance(from, to);
    
    if (dist < step) {
        return to;  // Return target config if it's close enough
    }
    
    // Interpolate towards target
    double ratio = step / dist;
    for (size_t i = 0; i < from.size(); i++) {
        double diff = to[i] - from[i];
        // Normalize angle difference
        while (diff > PI) diff -= 2 * PI;
        while (diff < -PI) diff += 2 * PI;
        newConfig[i] = from[i] + diff * ratio;
    }
    normalizeAngles(newConfig);
    return newConfig;
}

void plannerRRT(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength,
	int *tree_size)
{
	vector<Node*> tree;
	vector<double> startConfig(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
	vector<double> goalConfig(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
	tree.push_back(new Node(startConfig));

    const int MAX_ITERATIONS = 20000;
    const double STEP_SIZE = 0.7;
    const double GOAL_BIAS = 0.1;
    const double GOAL_THRESHOLD = 0.1;

	bool goalReached = false;
	int goalIndex = -1;

	double min_dist = 10000;

	for (int iter = 0; iter < MAX_ITERATIONS && !goalReached; ++iter) {
		vector<double> randomConfig = sampleRandomConfig(numofDOFs);
		// do
		// {
			if ((double)rand()/RAND_MAX < GOAL_BIAS) {
            	randomConfig = goalConfig;
        	} else {
            	randomConfig = sampleRandomConfig(numofDOFs);
       		}
			
		// } while (!IsValidArmConfiguration(randomConfig.data(), numofDOFs, map, x_size, y_size));

		// if (iter % 1000 == 0) {
        //     printf("Iteration %d, Tree size: %lu\n", iter, tree.size());
		// 	// printf();
		// 	printf("min_dist: %f\n", min_dist);
        // }
		
		Node* nearest = nearestNeighbor(tree, randomConfig);
		vector<double> newConfig = interpolateConfiguration(nearest->angles, randomConfig, STEP_SIZE);
		// vector<double> newConfig = steer(nearest->angles, randomConfig, stepSize);

		// print randomConfig, nearest, newConfig
		// printf("Random: ");
		// for (int i = 0; i < numofDOFs; ++i) {
		// 	printf("%f ", randomConfig[i]);
		// }
		// printf("\n");
		// printf("Nearest: ");
		// for (int i = 0; i < numofDOFs; ++i) {
		// 	printf("%f ", nearest->angles[i]);
		// }
		// printf("\n");
		// printf("New: ");
		// for (int i = 0; i < numofDOFs; ++i) {
		// 	printf("%f ", newConfig[i]);
		// }


		// printf("Iter: %d\n", iter);

		if (IsValidArmConfiguration(newConfig.data(), numofDOFs, map, x_size, y_size)) {

			// printf("Valid\n");

			Node* newNode = new Node(newConfig, nearest);
			tree.push_back(newNode);

			// printf("distance: %f\n", distance(newConfig, goalConfig));
			if (distance(newConfig, goalConfig)< min_dist) {
				min_dist = distance(newConfig, goalConfig);
			}
			// printf("min_dist: %f\n", min_dist);

			if (distance(newConfig, goalConfig) < GOAL_THRESHOLD) //&& IsValidArmConfiguration(goalConfig.data(), numofDOFs, map, x_size, y_size)) {
			{
				printf("Goal reached\n");
				goalReached = true;
                goalIndex = tree.size() - 1;

				Node* goalNode = new Node(goalConfig, newNode);
				tree.push_back(goalNode);

				vector<vector<double>> path;
				for (Node* node = goalNode; node != nullptr; node = node->parent) {
					path.push_back(node->angles);
				}
				std::reverse(path.begin(), path.end());

				*planlength = path.size();
				*tree_size = tree.size();
				logTreeSize(tree_size);
				*plan = (double**) malloc(*planlength * sizeof(double*));
				for (int i = 0; i < *planlength; ++i) {
					(*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
					std::copy(path[i].begin(), path[i].end(), (*plan)[i]);
				}
				return;
			}
		}
	}

   
	// No path found
	*planlength = 0;
	*plan = NULL;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//
enum ExtendResult {
    TRAPPED,
    ADVANCED,
    REACHED
};

//extend tree toward target
ExtendResult extend(std::vector<Node*>& tree, const std::vector<double>& target, double stepSize, double* map, int x_size, int y_size) {
	Node* nearest = nearestNeighbor(tree, target);
	std::vector<double> newConfig = interpolateConfiguration(nearest->angles, target, stepSize);
	if (!IsValidArmConfiguration(newConfig.data(), newConfig.size(), map, x_size, y_size)) {
		return TRAPPED;
	}
	Node* newNode = new Node(newConfig, nearest);
	tree.push_back(newNode);
	if (distance(newConfig, target) < stepSize) {
		return REACHED;
	}
	return ADVANCED;
}

//connect tree toward target
ExtendResult connect(std::vector<Node*>& tree, const std::vector<double>& target, double stepSize, double* map, int x_size, int y_size) {
	ExtendResult result;
	do {
		result = extend(tree, target, stepSize, map, x_size, y_size);
	} while (result == ADVANCED);
	
	return result;
}


//main RRT connect logic
static void plannerRRTConnect(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength,
	int *tree_size)
{
	vector<Node*> tree;
	vector<double> startConfig(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
	vector<double> goalConfig(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
	// tree.push_back(new Node(startConfig));

	const int MAX_ITERATIONS = 20000;
	const double STEP_SIZE = 0.5;
	;
	
	std::vector<Node*> treeA, treeB;
	treeA.push_back(new Node(startConfig));
	treeB.push_back(new Node(goalConfig));

	bool treesConnected = false;
	int treeAindex = -1;
	int treeBindex = -1;

	char goalTree = 'B';

	printf("Starting RRT-Connect planning...\n");

	for (int iter = 0; iter < MAX_ITERATIONS && !treesConnected; ++iter) {
		vector<double> randomConfig = sampleRandomConfig(numofDOFs);

		ExtendResult resultA = extend(treeA, randomConfig, STEP_SIZE, map, x_size, y_size);
		if(resultA != TRAPPED)
		{
			ExtendResult resultB = connect(treeB, treeA.back()->angles, STEP_SIZE, map, x_size, y_size);
			if(resultB == REACHED)
			{
				treesConnected = true;
				
				printf("Trees connected at iteration %d\n", iter);
			} else if (resultB == ADVANCED) {
				std::swap(treeA, treeB);
				if (goalTree == 'A') {
					goalTree = 'B';
				} else {
					goalTree = 'A';
				}
			}
		}

		// std::swap(treeA, treeB);

		if (iter % 1000 == 0) {
			printf("Iteration %d, Tree A size: %lu, Tree B size: %lu\n", 
				   iter, treeA.size(), treeB.size());
		}
	}

	if (treesConnected) {
		printf("Constructing path...\n");
		//tree a index
		// printf("treeaindex: %d\n", treeAindex);
		// printf("treeA size: %lu\n", treeA.size());
		// printf("treeb size: %lu\n", treeB.size());
		// printf("treebinex: %d\n", treeBindex);
		// printf("last node in treeA: ");
		if(goalTree == 'A') {
			std::swap(treeA, treeB);
		} 
		treeAindex = treeA.size() - 1;
		treeBindex = treeB.size() - 1;
		// for (int i = 0; i < numofDOFs; ++i) {
		// 	printf("%f ", treeA[treeAindex]->angles[i]);
		// }
		vector<vector<double>> pathA;
		for (Node* node = treeA[treeAindex]; node != nullptr; node = node->parent) {
			pathA.push_back(node->angles);
		}
		std::reverse(pathA.begin(), pathA.end());
		vector<vector<double>> pathB;
		for (Node* node = treeB[treeBindex]; node != nullptr; node = node->parent) {
			pathB.push_back(node->angles);
		}
		// std::reverse(pathB.begin(), pathB.end());

		//combine paths
		printf("combining paths\n");
		std::vector<std::vector<double>> fullPath = pathA;
		fullPath.insert(fullPath.end(), pathB.begin(), pathB.end());
		printf("path combined\n");

		*planlength = fullPath.size();
		*tree_size = treeA.size() + treeB.size();
		logTreeSize(tree_size);
		*plan = (double**)malloc(fullPath.size() * sizeof(double*));
		for (size_t i = 0; i < fullPath.size(); i++) {
			(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
			for (int j = 0; j < numofDOFs; j++) {
				(*plan)[i][j] = fullPath[i][j];
			}
		}
		 printf("Path found with %d waypoints\n", *planlength);
	} else {
		// No path found
		*planlength = 0;
		*plan = NULL;
		printf("Failed to find path after %d iterations\n", MAX_ITERATIONS);
	}
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//


// Function to compute rewiring radius based on tree size
double computeRadius(int treeSize, int dimensions, double stepSize) {
    // gamma_RRT* * (log(n)/n)^(1/d), where gamma_RRT* is a constant
    double gamma = 2.0;  // This can be tuned
    return gamma * stepSize * pow(log(treeSize + 1.0) / (treeSize + 1.0), 1.0 / dimensions);
}




static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength,
	int *tree_size)
{
    std::vector<Node*> tree;
	std::vector<double> startConfig(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
	std::vector<double> goalConfig(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);


	tree.push_back(new Node(startConfig));

	const int MAX_ITERATIONS = 10000;
	const double STEP_SIZE = 1.2;
	const double GOAL_THRESHOLD = 0.1;
	const double GoalBias = 0.01;

	int bestGoalIndex = -1;
	double bestGoalCost = std::numeric_limits<double>::max();

	for(int iter =0; iter< MAX_ITERATIONS; iter++)
	{
		std::vector<double> randomConfig = sampleRandomConfig(numofDOFs);
		if((double)rand()/RAND_MAX < GoalBias)
		{
			randomConfig = goalConfig;
		}
		Node* nearest = nearestNeighbor(tree, randomConfig);
		std::vector<double> newConfig = interpolateConfiguration(nearest->angles, randomConfig, STEP_SIZE);

		if(!IsValidArmConfiguration(newConfig.data(), newConfig.size(), map, x_size, y_size))
		{
			continue;
		}

		double radius = computeRadius(tree.size(), numofDOFs, STEP_SIZE);
		std::vector<Node*> nearNodes;
		std::vector<int> neighbors;

		for (Node* node : tree) {
			if (distance(node->angles, newConfig) < radius) {
				nearNodes.push_back(node);
			}
		}

		Node* bestParent = nearest;
		double bestCost = nearest->cost + distance(nearest->angles, newConfig);

		for (Node* node : nearNodes) {
			double newCost = node->cost + distance(node->angles, newConfig);
			if (newCost < bestCost && IsValidLineSegment(node->angles[0], node->angles[1], newConfig[0], newConfig[1], map, x_size, y_size)) {
				bestParent = node;
				bestCost = newCost;
			}
		}

		Node* newNode = new Node(newConfig, bestParent);
		newNode->cost = bestCost;
		tree.push_back(newNode);

		// Rewire neighbors
		for (Node* node : nearNodes) {
			if (node == bestParent) {
				continue;
			}
			double newCost = newNode->cost + distance(newNode->angles, node->angles);
			if (newCost < node->cost && IsValidLineSegment(newNode->angles[0], newNode->angles[1], node->angles[0], node->angles[1], map, x_size, y_size)) {
				node->parent = newNode;
				node->cost = newCost;
			}
		}

		// Check if close to the goal
		// printf("distance: %f\n", distance(newConfig, goalConfig));
		if (distance(newConfig, goalConfig) < GOAL_THRESHOLD) {
			// printf("Goal reached\n");
			double cost = newNode->cost + distance(newConfig, goalConfig);
			if (cost < bestGoalCost) {
				bestGoalCost = cost;
				bestGoalIndex = tree.size() - 1;
			}
		}
		// printf("Iteration %d, Tree size: %lu\n", iter, tree.size());
		// if (iter % 500 == 0) {
		// 	printf("Iteration %d, Tree size: %lu\n", iter, tree.size());
		// }
	}

	if (bestGoalIndex !=-1)
	{
		printf("Goal reached\n");
		std::vector<std::vector<double>> path;
		path.push_back(goalConfig);
		for (Node* node = tree[bestGoalIndex]; node != nullptr; node = node->parent) {
			path.push_back(node->angles);
		}
		std::reverse(path.begin(), path.end());

		*planlength = path.size();
		*tree_size = tree.size();
		logTreeSize(tree_size);
		*plan = (double**)malloc(*planlength * sizeof(double*));

		for (size_t i = 0; i < path.size(); i++) {
			(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
			for (int j = 0; j < numofDOFs; j++) {
				(*plan)[i][j] = path[i][j];
			}
		}
		printf("Path found with %d waypoints\n", *planlength);
	} else {
		// No path found
		*planlength = 0;
		*plan = NULL;
		printf("Failed to find path after %d iterations\n", MAX_ITERATIONS);
	}

}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerPRM(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength,
	int *tree_size)
{
	const int NUM_SAMPLES = 5000;
	const double STEP_SIZE = 0.5;
	// const double NEIGHBOR_RADIUS = 1.0;
	const int K = 5;


	std::vector<Node*> nodes;
	std::vector<std::vector<int>> edges(NUM_SAMPLES);

	// Sample random configurations
	for (int i = 0; i < NUM_SAMPLES; ++i) {
		std::vector<double> config = sampleRandomConfig(numofDOFs);
		if (IsValidArmConfiguration(config.data(), numofDOFs, map, x_size, y_size)) {
			nodes.push_back(new Node(config));
		}
	}

	// Add start and goal configurations
	std::vector<double> startConfig(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
	std::vector<double> goalConfig(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
	nodes.push_back(new Node(startConfig));
	nodes.push_back(new Node(goalConfig));

	// Connect nodes using K-nearest neighbors
	for (int i = 0; i < nodes.size(); ++i) {
		std::vector<std::pair<double, int>> distances;
		for (int j = 0; j < nodes.size(); ++j) {
			if (i != j) {
				double dist = distance(nodes[i]->angles, nodes[j]->angles);
				distances.push_back(std::make_pair(dist, j));
			}
		}
		std::sort(distances.begin(), distances.end());
		for (int k = 0; k < std::min(K, (int)distances.size()); ++k) {
			int neighborIndex = distances[k].second;
			if (IsValidLineSegment(nodes[i]->angles[0], nodes[i]->angles[1], nodes[neighborIndex]->angles[0], nodes[neighborIndex]->angles[1], map, x_size, y_size)) {
				edges[i].push_back(neighborIndex);
				edges[neighborIndex].push_back(i);
			}
		}
	}
	// find the shortest path using A* algorithm
	auto heuristic = [&](const vector<double>& a, const vector<double>& b) {
		return distance(a, b);
	};


	std::priority_queue<std::tuple<double, int, double>> openSet;
	std::vector<double> gScore(nodes.size(), std::numeric_limits<double>::max());
	std::vector<int> cameFrom(nodes.size(), -1);

	int startIndex = nodes.size() - 2;
	int goalIndex = nodes.size() - 1;

	gScore[startIndex] = 0;
	openSet.emplace(heuristic(startConfig, goalConfig), startIndex, 0);

	while (!openSet.empty()) {
		// printf("openSet size: %lu\n", openSet.size());
		auto [fScore, current, currentG] = openSet.top();
		openSet.pop();
		// printf("Current: %d\n", current);

		//print size of edges
		// printf("edges size: %lu\n", edges[current].size());

		if (current == goalIndex) {
			printf("Goal reached\n");
			std::vector<std::vector<double>> path;
			for (int node = goalIndex; node != -1; node = cameFrom[node]) {
				path.push_back(nodes[node]->angles);
			}
			std::reverse(path.begin(), path.end());

			*planlength = path.size();
			*tree_size = nodes.size();
			logTreeSize(tree_size);
			*plan = (double**)malloc(*planlength * sizeof(double*));
			for (size_t i = 0; i < path.size(); i++) {
				(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
				for (int j = 0; j < numofDOFs; j++) {
					(*plan)[i][j] = path[i][j];
				}
			}
			
			return;
		}

		for (int neighbor : edges[current]) {
			// printf("Neighbor: %d\n", neighbor);
			double tentativeGScore = gScore[current] + distance(nodes[current]->angles, nodes[neighbor]->angles);
			// printf("Tentative G Score: %f\n", tentativeGScore);
			if (tentativeGScore < gScore[neighbor]) {
				// printf("Neighbor: %d\n", neighbor);
				cameFrom[neighbor] = current;
				gScore[neighbor] = tentativeGScore;
				double fScore = tentativeGScore + heuristic(nodes[neighbor]->angles, goalConfig);
				openSet.emplace(-fScore, neighbor, tentativeGScore);
			}
		}
	}
	printf("Failed to find path\n");

	// No path found
	*planlength = 0;
	*plan = NULL;

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

    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + argv[1];
    std::cout << "Reading problem definition from: " << mapFilePath << std::endl;
	tie(map, x_size, y_size) = loadMap(mapFilePath);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);

    std::string outputDir = OUTPUT_DIR;
	string outputFile = outputDir + "/" + argv[6];
	std::cout << "Writing solution to: " << outputFile << std::endl;

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;
	int treeSize = 0;

	// // generate 20 random samples of start and end configurations, and check if they are valid for the given map
	// for (int i = 0; i < 20; i++) {
	// 	vector<double> randomConfig = sampleRandomConfig(numOfDOFs);
	// 	do
	// 	{
    //         	randomConfig = sampleRandomConfig(numOfDOFs);
			
	// 	} while (!IsValidArmConfiguration(randomConfig.data(), numOfDOFs, map, x_size, y_size));

	// 	vector<double> start_angles = randomConfig;

	// 	do
	// 	{
	// 			randomConfig = sampleRandomConfig(numOfDOFs);
			
	// 	} while (!IsValidArmConfiguration(randomConfig.data(), numOfDOFs, map, x_size, y_size));

	// 	vector<double> goal_angles = randomConfig;

	// 	// Print configurations in the specified format
	// 	std::cout << "[\"" << argv[1] << "\", \"";
	// 	for (int j = 0; j < numOfDOFs; ++j) {
	// 		std::cout << start_angles[j];
	// 		if (j < numOfDOFs - 1) std::cout << ",";
	// 	}
	// 	std::cout << "\", \"";
	// 	for (int j = 0; j < numOfDOFs; ++j) {
	// 		std::cout << goal_angles[j];
	// 		if (j < numOfDOFs - 1) std::cout << ",";
	// 	}
	// 	std::cout << "\"]," << std::endl;

	// 	// // print start and goal angles
	// 	// printf("Start: ");
	// 	// for (int i = 0; i < numOfDOFs; ++i) {
	// 	// 	printf("%f ", start_angles[i]);
	// 	// }
	// 	// printf("\n");
	// 	// printf("Goal: ");
	// 	// for (int i = 0; i < numOfDOFs; ++i) {
	// 	// 	printf("%f ", goal_angles[i]);
	// 	// }
	// 	// printf("\n");


	// }

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, &treeSize);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, &treeSize);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, &treeSize);
		
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, &treeSize);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, &treeSize);
    }


	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
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
	m_log_fstream << mapFilePath << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
