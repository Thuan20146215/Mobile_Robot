#include <iostream>
#include <fstream>
#include <tuple>
#include <map>  // adding for mapping constraints
#include <chrono>
#include <vector>
#include <list>
#include <string>
#include <ostream>
#include <queue>
#include <unordered_map>
#include <algorithm>

using namespace std;

#define STRAIGHT 3
#define LEFT     1
#define RIGHT    2
#define STOP     4

int PreviousState = 0;
int CurrentState = 0;

// =============== Final Output =================
int start, destination;
std::vector<int> result;
std::vector<int> finalResult;
int turnAroundState = 0;
//================= MAPFInstance ===============================

// Vertex constraint <a, x, -1, t>
// that prohibits agent a from being at location x at timestep t
// Edge constraint <a, x, y, t>
// that prohibits agent a from moving from locations x to y from timesteps t-1 to t
typedef tuple<int, int, int, int > Constraint;

// Path is a sequence of locations,
// where path[i] represents the location at timestep i
typedef vector<int> Path;
ostream& operator<<(ostream& os, const Path& path); // used for printing paths

// A hash function used to hash a pair of any kind
// This will be used in Task 1 when you try to
// use pair as the key of an unordered_map
struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const
    {
        auto hash1 = hash<T1>{}(p.first);
        auto hash2 = hash<T2>{}(p.second);
        return hash1 ^ hash2;
    }
};

struct AStarNode {
    int location;
    int g;
    int h;
    int timestep;
    AStarNode* parent;

    AStarNode(): location(-1), g(-1), h(-1),  timestep(-1), parent(nullptr) {}
    AStarNode(int location, int g, int h, int timestep, AStarNode* parent):
            location(location), g(g), h(h), timestep(timestep), parent(parent) {}
};

// This function is used by priority_queue to prioritize nodes
struct CompareAStarNode {
    bool operator()(const AStarNode* n1, const AStarNode* n2) {
        if (n1->g + n1->h == n2->g + n2->h) // if both nodes have the same f value,
            return n1->h > n2->h; // break ties by preferring smaller h value
        else
            return n1->g + n1->h > n2->g + n2->h; // otherwise, prefer smaller f value
    }
};

class MAPFInstance {
public:
    vector<int> start_locations;
    vector<int> goal_locations;
    int num_of_agents;

    // return true if the location is blocked by an obstacle.
    inline bool blocked(int location) const {return my_map[location]; }
    inline size_t map_size() const { return rows * cols; }

    // This can be used as admissible heuristics
    int get_Manhattan_distance(int from, int to) const;

    list<int> get_adjacent_locations(int location) const; // return unblocked adjacent locations
    bool load_instance(const string& fname); // load instance from file
    void print_instance() const;

private:
  vector<bool> my_map; // my_map[i] = true iff location i is blocked
  int rows;
  int cols;
  enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size
  int moves_offset[5];

  inline int linearize_coordinate(int row, int col) const { return (this->cols * row + col); }
  inline int row_coordinate(int location) const { return location / this->cols; }
  inline int col_coordinate(int location) const { return location % this->cols; }
};

class AStarPlanner {
public:
    const MAPFInstance& ins;

    AStarPlanner(const MAPFInstance& ins): ins(ins) {}
    Path find_path(int agent_id, const list<Constraint>& constraints, int CBSorPP);
private:
    // used to retrieve the path from the goal node
    Path make_path(const AStarNode* goal_node) const;

    void clear();
};

struct CBSNode {
    list<Constraint> constraints;
    vector<Path> paths;
    int cost;

    CBSNode(): cost(0) {}

    // this constructor helps to generate child nodes
    CBSNode(const CBSNode& parent):
            constraints(parent.constraints), paths(parent.paths), cost(0) {}
};

// This function is used by priority_queue to prioritize CBS nodes
struct CompareCBSNode {
    bool operator()(const CBSNode* n1, const CBSNode* n2) {
        return n1->cost > n2->cost; // prefer smaller cost
    }
};

// collision struct to return from find_collision
struct collision{
    int agent_a;    // first agent involved
    int loc_a;      // location of the first agent
    int agent_b;    // second agent involved
    int loc_b;      // location of the second agent
    int timestep;   // timestep of the collision
    string collision_type; // can be "vertex", "edge", or "none"
};

class CBS {
public:
    vector<Path> find_solution();
    explicit CBS(const MAPFInstance& ins): a_star(ins) {}
    ~CBS();
    Constraint vert_constraint(int agent, int loc, int timestep);
    Constraint edge_constraint(int agent_a, int agent_b, int x, int y, int timestep, int num);
    //collision find_collision(vector<Path> paths);
    collision find_collision(vector<Path> paths);

private:
    AStarPlanner a_star;
    /*
        function to generate a new CBSNode every time there is a collision
        inputs: agent_0 -> first agents id
                agent_1 -> second agents id
                VorE -> vertex or edge collision vertex = 0, edge = 1
                num -> first or second constraint to generate; 0 = first form, 1 = second form
        outputs: a new CBSNode
    */

    // all_nodes stores the pointers to CBS nodes
    // so that we can release the memory properly when
    // calling the destructor ~CBS()
    list<CBSNode*> all_nodes;

};

//============================ Thang's space============================================

//========================================================================

void updateMap(vector<vector<char>>& grid, const vector<pair<int, int>>& obstacles) {
    for (const auto& obstacle : obstacles) {
        int x = obstacle.second;
        int y = obstacle.first;
        grid[y][x] = '@';
    }
}

void writeDataToFile(const string& filePath, const vector<vector<char>>& grid, int agentCount, const vector<pair<pair<int, int>, pair<int, int>>>& agents) {
    ofstream file(filePath);
    if (file.is_open()) {
        int width = grid[0].size();
        int height = grid.size();
        file << width << " " << height << "\n";
        for (const auto& row : grid) {
            for (char cell : row) {
                file << cell << " ";
            }
            file << "\n";
        }

        file << agentCount << "\n";
        for (const auto& agent : agents) {
            file << agent.first.first << " " << agent.first.second << " " << agent.second.first << " " << agent.second.second << "\n";
        }

        file.close();
    }
}

void processInputData(const string& filePath_ForPrePro, const string& filePath_AfterPrePro, const vector<pair<int, int>>& obstacles, const vector<pair<pair<int, int>, pair<int, int>>>& agents, int agentCnt, int obstacleCnt) {
    ifstream file(filePath_ForPrePro);
    if (file.is_open()) {
        int width, height;
        file >> width >> height;

        vector<vector<char>> grid(height, vector<char>(width));
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                file >> grid[i][j];
            }
        }


        updateMap(grid, obstacles);


        file.close();

        writeDataToFile(filePath_AfterPrePro, grid, agentCnt, agents);

        cout << "Các vật cản đã được ghi vào ma trận 15x15 với ký tự @" << endl;
        cout << "File new.txt đã được tạo và ghi dữ liệu." << endl;
        cout << "Có tổng cộng " << agentCnt << " dòng đã được ghi vào file new.txt." << endl;
        for (const auto& agent : agents) {
            cout << agent.first.first << " " << agent.first.second << " " << agent.second.first << " " << agent.second.second << endl;
        }
    } else {
        cout << "Không thể mở file " << filePath_ForPrePro << endl;
    }
}

bool MAPFInstance::load_instance(const string& fname) {
    ifstream myfile (fname.c_str(), ios_base::in);
    if (myfile.is_open()) {
        myfile >> rows >> cols; // read the size of the map
        my_map.resize(rows * cols);

        // read map
        char c;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                myfile >> c;
                my_map[cols * i + j] = (c != '.'); // the cell is unblocked iff it is '.'
            }
        }

        myfile >> num_of_agents; // read the number of agents
        start_locations.resize(num_of_agents);
        goal_locations.resize(num_of_agents);

        // read the start and goal locations
        int start_x, start_y, goal_x, goal_y;
        for (int i = 0; i < num_of_agents; i++) {
            myfile >> start_x >> start_y >> goal_x >> goal_y;
            start_locations[i] = linearize_coordinate(start_x, start_y);
            goal_locations[i] = linearize_coordinate(goal_x, goal_y);
        }
        myfile.close();

        // initialize moves_offset array
        moves_offset[valid_moves_t::WAIT_MOVE] = 0;
        moves_offset[valid_moves_t::NORTH] = -cols;
        moves_offset[valid_moves_t::EAST] = 1;
        moves_offset[valid_moves_t::SOUTH] = cols;
        moves_offset[valid_moves_t::WEST] = -1;
        return true;
    } else
        return false;
}

void MAPFInstance::print_instance() const {
    cout << "Map:" << endl;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (my_map[cols * i + j])
                cout << "@"; // obstacle
            else
                cout << "."; // free cell
        }
        cout << endl;
    }
    cout << num_of_agents << " agents:" << endl;
    for (int i = 0; i < num_of_agents; i++) {
        cout << "a" << i << ": " << start_locations[i] << "-->" << goal_locations[i] << endl;
    }
}

// the manhattan distance is our heuristic function for the A* search algorithm
int MAPFInstance::get_Manhattan_distance(int from, int to) const {
    int from_x = row_coordinate(from);
    int from_y = col_coordinate(from);
    int to_x = row_coordinate(to);
    int to_y = col_coordinate(to);
    return abs(from_x - to_x) + abs(from_y - to_y);
}

// function that gets all the available locations for the current agent
list<int> MAPFInstance::get_adjacent_locations(int location) const {
	list<int> locations;
	for (int direction : moves_offset) {
		int next_location = location + direction;
		if (0 <= next_location && next_location < cols * rows && // next_location is on the map
		    !my_map[next_location] && // next_location is not blocked
		    get_Manhattan_distance(location, next_location) <= 1) // it indeed moves to a neighbor location
            locations.push_back(next_location);
	}
	return locations;
}

//================= AStarPlanner ===============================



ostream& operator<<(ostream& os, const Path& path)
{
    for (auto loc : path) {
        os << loc << " ";
    }
    return os;
}

Path AStarPlanner::make_path(const AStarNode* goal_node) const {
    Path path;
    const AStarNode* curr = goal_node;
    while (curr != nullptr) {
        path.push_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
    return path;
}

// Note: I added an int to say whether we are using CBS or PP to make sure the correct cost and paths are outputed
Path AStarPlanner::find_path(int agent_id, const list<Constraint>& constraints, int CBSorPP) {
    int start_location = ins.start_locations[agent_id];
    int goal_location = ins.goal_locations[agent_id];
    bool fail_constraint;   // initialize a checker to see if a constraint is failed

    // Open list
    priority_queue<AStarNode*, vector<AStarNode*>, CompareAStarNode> open;

    // check to see the maximum timestep of the constraint on the current agent
    int largest_timestep_CBS = 0;
    for(auto constraint : constraints){
        int ai = get<0>(constraint);
        int t = get<3>(constraint);

        if(agent_id == ai && t > largest_timestep_CBS){
            largest_timestep_CBS = t;
        }
    }

    int largest_timestep_PP = 0;
    for(auto constraint : constraints){
        int ai = get<0>(constraint);
        int x = get<1>(constraint);
        int y = get<2>(constraint);
        int t = get<3>(constraint);

        if(ai == agent_id  && t > largest_timestep_PP && (goal_location == x || goal_location == y)){
            largest_timestep_PP = t;
        }
    }

    // Unordered map is an associative container that contains key-value pairs with unique keys.
    // The following unordered map is used for duplicate detection, where the key is the location of the node.

    unordered_map<pair<int, int>, AStarNode*, hash_pair> all_nodes; // new nodes mapping for

    int h = ins.get_Manhattan_distance(start_location, goal_location); // h value for the root node
    auto root = new AStarNode(start_location, 0, h, 0, nullptr); // root node starts at time step 0
    open.push(root);

    Path path; // vector <int>
    while (!open.empty()) {
        auto curr = open.top();
        open.pop();

        // goal test
        // if we are using Conflict Based Search we need to check the current agents largest timestep
        if(CBSorPP == 0){
            if (curr->location == goal_location && curr->timestep > largest_timestep_CBS){
                path = make_path(curr);
                break;
            }
            
        }
        // if we are using Prioritized Planning we need to check if there are constraints from an earlier agent past the current timestep
        else if(CBSorPP == 1){
            if(curr->location == goal_location && curr->timestep > largest_timestep_PP){
                path = make_path(curr);
                break;
            }
        }

        // generate child nodes
        for (auto next_location : ins.get_adjacent_locations(curr->location)) {

            //auto it = all_nodes.find(next_location); // original
            pair <int, int> curr_pair;   // create a pair variable for the current node and timestep pair
            curr_pair.first = next_location;     // first value of pair is the loction
            curr_pair.second = curr->timestep;   // second value of pair is the timestep
            auto it = all_nodes.find(curr_pair);     // modify it from task0 to lookup currPair as its key

            if (it == all_nodes.end()) {// the location has not been visited before
                int next_g = curr->g + 1;
                int next_h = ins.get_Manhattan_distance(next_location, goal_location);
                int next_t = curr->timestep + 1; // calculate timestep of child node
                auto next = new AStarNode(next_location, next_g, next_h, next_t, curr); // generate the next AStarNode
                
                /*
                 see if the next node satisfies the given constraints
                    we are only concerned with two types of constraints for this example (edge and vertex)
                */
               /*
                    If the constraint is a vertex constraint, then it is in the format < ai, x, −1, t >
                    which prohibits agent ai
                    from being at location x at timestep t. 
               */
               /*
                    If the constraint is an edge constraint, then it is in the format < ai, x, y, t >  
                    which prohibits agent ai from moving from location x to location y from timestep
                    t−1 to timestep t
               */

                fail_constraint = false;    //  reset the fail constraint

                // if our next time step is too large, we are in an infinite loop and should exit without finding a path
                if(next_t > 3000){
                    Path empty_path;
                    return empty_path;
                }

                for(auto constraint : constraints){
                    // get the values from the constraints
                    int ai = get<0>(constraint);
                    int x = get<1>(constraint);
                    int y = get<2>(constraint);
                    int t = get<3>(constraint);

                    // vertex constraint
                    if(y == -1){
                        // prohibits agent ai from being at location x at time t
                        if (agent_id == ai && next_location == x && next_t == t){
                            fail_constraint = true;
                        }

                        // if we have a negative timestep, then we know we have reached a higher priority agents goal
                        // node and should avoid it (only applicable to prioritized planner)
                        if(t < 0){
                            if(agent_id == ai && next_location == x){
                                fail_constraint = true;
                            }
                        }
                    }
                    // edge constraint
                    else{
                        // prohibits agent ai from moving from location x to location y
                        // from timestep t-1(current) to timestep t(next)
                        if(agent_id == ai && curr->location == x && next_location == y && next_t == t){
                            fail_constraint = true;
                        }

                        if(t <= 0){
                            if(agent_id == ai && curr->location == x && next_location == y){
                                fail_constraint = true;
                            }
                        }
                    }

                }
                
                // if constraints are met, push the AstarNode and add it to all_nodes
                if(fail_constraint == false){
                    open.push(next);
                    all_nodes[curr_pair] = next;
                }
            }
            // Note that if the location has been visited before,
            // next_g + next_h must be greater than or equal to the f value of the existing node,
            // because we are searching on a 4-neighbor grid with uniform-cost edges.
            // So we don't need to update the existing node.
        }
    }

    // release memory
    for (auto n : all_nodes){
        delete n.second;
    }
    
    return path;
}


//====================================================================

//====================================================================

vector<Path> CBS::find_solution() {
    priority_queue<CBSNode*, vector<CBSNode*>, CompareCBSNode> open; // open list

    /* generate the root CBS node */
    auto root = new CBSNode();
    all_nodes.push_back(root);  // whenever generating a new node, we need to
                                 // put it into all_nodes
                                 // so that we can release the memory properly later in ~CBS()

    // find paths for the root node
    root->paths.resize(a_star.ins.num_of_agents);
    for (int i = 0; i < a_star.ins.num_of_agents; i++) {
        // TODO: if you change the input format of function find_path()
        //  you also need to change the following line to something like
        //  root->paths[i] = a_star.find_path(i, list<Constraint>());
        root->paths[i] = a_star.find_path(i, list<Constraint>(), 0);
        if (root->paths[i].empty()) {
            cout << "Fail to find a path for agent " << i << endl;
            return vector<Path>(); // return "No solution"
        }
    }
    // compute the cost of the root node
    for (const auto& path : root->paths)
        root->cost += (int)path.size() - 1;

    // put the root node into open list
    open.push(root);

    int time = 0;
    // high-level implementation of CBS
    while (!open.empty()) {
        // get the node with the smallest cost
        CBSNode *P = open.top();

        // find collision (if there are any) from P
        collision find_col = find_collision(P->paths);

        // if there are no collisions, then we return the path
        if(find_col.collision_type == "none"){
            return(P->paths);
        }

        list<Constraint> new_constraints;   // list to store our two new constraints

        // if its a vertex collision then we generate two vertex constraints
        if(find_col.collision_type == "vertex"){
            // prohibit the first agent from being in the loc at the timestep
            Constraint v1 = vert_constraint(find_col.agent_a, find_col.loc_a, find_col.timestep);
            // prhibit the second agent from being in the loc at the timestep
            Constraint v2 = vert_constraint(find_col.agent_b, find_col.loc_b, find_col.timestep);
            new_constraints.push_back(v1);
            new_constraints.push_back(v2);
        }
        // if its an edge collision then we generate two edge constraints
        else if(find_col.collision_type == "edge"){
            // prohibit the first agent from moving from x to y at timestep t
            Constraint e1 = edge_constraint(find_col.agent_a, find_col.agent_b, find_col.loc_a, find_col.loc_b, find_col.timestep, 0);
            Constraint e2 = edge_constraint(find_col.agent_a, find_col.agent_b, find_col.loc_a, find_col.loc_b, find_col.timestep, 1);

            new_constraints.push_back(e1);
            new_constraints.push_back(e2);
        }

        // loop through the new_constraints and create new nodes
        for(auto constraint : new_constraints){
            // construnct a new Node for our conflict tree
            auto Q = new CBSNode(*P);

            // add the new constraint to the new nodes list of constraints
            Q->constraints.push_back(constraint);

            // get the agent for which the constraint takes place on
            int ai = get<0>(constraint);
            int x = get<1>(constraint);
            int y = get<2>(constraint);
            int t = get<3>(constraint);

            // generate a new path for the agent based on the new constraints
            Path ai_path = a_star.find_path(ai, Q->constraints, 0);

            // if the path isn't empty
            if(!ai_path.empty()){
                // replace the current path for the agent
                Q->paths[ai] = ai_path;
                
                // compute the cost of the root node
                for (const auto& path : Q->paths)
                    Q->cost += (int)path.size() - 1;
                
                // insert Q into open
                open.push(Q);
            }
            //return(Q->paths);
        }
        // pop off the current node and then we choose the next node with the lowest cost
        open.pop();

        time++;
    }
    return vector<Path>(); // return "No solution"
}

Constraint CBS::vert_constraint(int agent, int loc, int timestep){
    Constraint vert = make_tuple(agent,loc, -1, timestep);
    return vert;
}

Constraint CBS::edge_constraint(int agent_a, int agent_b, int x, int y, int timestep, int num){
    // edge constraint for agent a
    if(num == 0){
        Constraint edge_a = make_tuple(agent_a, x, y, timestep);
        return edge_a;
    }
    // edge constraint for agent b
    else {
        Constraint edge_b = make_tuple(agent_b, y, x , timestep);
        return edge_b;
    }
}

collision CBS::find_collision(vector<Path> paths){
    collision is_collision;

    int num_paths = paths.size();
    // find the longest path size
    int longest_path = 0;
    for(int i = 0; i < num_paths; i++){
        if(paths[i].size() > longest_path){
            longest_path = paths[i].size();
        }
    }

    // resize all of the paths to be the same length
    vector<Path> paths_same;
    paths_same.resize(num_paths);
    for(int i = 0; i < num_paths; i++){
        paths_same[i].resize(longest_path);
    }
    
    // loop through the paths and if we are past the last value, contine filling in
    // the resized paths holder with the values
    for(int i = 0; i < num_paths; i++){
        int last_value = paths[i].size()-1;
        for(int j = 0; j < longest_path; j++){
            if(j >= last_value){
               paths_same[i][j] = paths[i][last_value]; 
            }
            else{
                paths_same[i][j] = paths[i][j];
            }
        }
    }

    // Search for vertex collisions
    for(int agent = 0; agent < num_paths-1; agent++){
        for(int next_agent = agent+1; next_agent < num_paths; next_agent++){
            for(int time = 0; time < longest_path; time++){
            // VERTEX COLLISION: they are the same at the same time
                if(paths_same[agent][time] == paths_same[next_agent][time]){
                    collision vert_col;
                    int loc = paths_same[agent][time];
                    vert_col.collision_type = "vertex";
                    vert_col.agent_a = agent;
                    vert_col.agent_b = next_agent;
                    vert_col.loc_a = loc;
                    vert_col.loc_b = loc;
                    vert_col.timestep = time;
                    return vert_col;
                }
            }
        }
    }
    /*
        where agent a moves from cell x to cell y and
        agent b moves from cell y to cell x at time step t
    */
   // Search for edge collisions
    for(int agent = 0; agent < num_paths-1; agent++){
        for(int next_agent = agent+1; next_agent < num_paths; next_agent++){
            for(int time = 0; time < longest_path - 1; time++){
            int next_time = time + 1;
            int posA_1 = paths_same[agent][time];
            int posA_2 = paths_same[agent][next_time];
            int posB_1 = paths_same[next_agent][time];
            int posB_2 = paths_same[next_agent][next_time];

                // EDGE COLLISION: if they move to opposite locations on the same timestep
                if(posA_1 == posB_2 && posA_2 == posB_1){
                    collision edge_col;
                    edge_col.collision_type = "edge";
                    edge_col.agent_a = agent;
                    edge_col.agent_b = next_agent;
                    edge_col.loc_a = posA_1;
                    edge_col.loc_b = posB_1;
                    edge_col.timestep = next_time;
                    return edge_col;
                }
            }
        }
    }

    // if the function makes it this far then there are no collisions
    is_collision.collision_type = "none";
    return is_collision;
}

//  THANG'S SPACE=======================================================

std::vector<int> processPaths(const std::vector<Path>& input) {
  std::vector<int> output;
  std::unordered_map<int, int> pathToStationCount = {
    {94, 1}, {99, 2}, {104, 3},
    {148, 4}, {153, 5}, {158, 6},
    {202, 7}, {207, 8}, {212, 9},
    {132, 10}, {137, 11}, {186, 12}, {191,13}
  };

  for (const Path& path : input) {
    for (int value : path) {
      if (pathToStationCount.count(value) > 0) {
        output.push_back(pathToStationCount[value]);
      }
    }
  }

  return output;
}

int getCoordY(int index)
{
    std::unordered_map<int, int> pathToStationCount = {
    {94, 1}, {99, 2}, {104, 3},
    {148, 4}, {153, 5}, {158, 6},
    {202, 7}, {207, 8}, {212, 9},
    {132, 10}, {137, 11}, {186, 12}, {191,13}
    };

    for (auto it = pathToStationCount.begin(); it != pathToStationCount.end(); ++it) {
       if(it->second == index)
       return (it->first) % 18;
        }
        return 0;
}
int getCoordX(int index)
{
    std::unordered_map<int, int> pathToStationCount = {
    {94, 1}, {99, 2}, {104, 3},
    {148, 4}, {153, 5}, {158, 6},
    {202, 7}, {207, 8}, {212, 9},
    {132, 10}, {137, 11}, {186, 12}, {191,13}
    };

    for (auto it = pathToStationCount.begin(); it != pathToStationCount.end(); ++it) {
       if(it->second == index)
       return (it->first) / 18;
        }
        return 0;
}

std::vector<int> convertPath(const std::vector<Path>& input) {
    std::vector<int> output;
    for (int i = 0; i < input.size(); i++) {
        for (int j = 0; j < input[i].size(); j++) {

            int preValue, postValue;
            int value = input[i][j];
            if(j - 1 >= 0)
                preValue = input[i][j] - input[i][j - 1];
            if(j + 1 < input[i].size())
                postValue = input[i][j + 1] - input[i][j];

            if(value == 96 || value ==101 || value ==145 || value ==150 || value ==155 || value ==160 || value ==204 || value ==209){
                cout << value << " "<< preValue << " "<< postValue;
                if(preValue == postValue)
                    output.push_back(STRAIGHT);
                else if((preValue > 0) && (preValue == 18))
                {
                    if(postValue < 0)
                        output.push_back(RIGHT);
                    else if(postValue > 0)
                        output.push_back(LEFT);
                }

                else if((preValue < 0) && (preValue == 18))
                {
                    if(postValue < 0)
                        output.push_back(LEFT);
                    else if(postValue > 0)
                        output.push_back(RIGHT);
                }

                else if((preValue > 0) && (preValue == 1))
                {
                    if(postValue < 0)
                        output.push_back(LEFT);
                    else if(postValue > 0)
                        output.push_back(RIGHT);
                }
                else if((preValue < 0) && (preValue == 1))
                {
                    if(postValue < 0)
                        output.push_back(RIGHT);
                    else if(postValue > 0)
                        output.push_back(LEFT);
                }
                cout << " " << output[j] << endl;
            }
        }
    }
    return output;

}

void writeFinalOutput(const string& filePath){
    std::ofstream outFile(filePath, std::ios::out);

    if (outFile.is_open()) {
        outFile <<  start << " " << destination << std::endl;

        for (std::vector<int>::size_type i = 0; i < result.size() - 1; i++) {
            outFile << result[i] << " ";
        }

        outFile << std::endl;

        for (const auto& element1 : finalResult) {
            outFile << element1 << " ";}
        outFile << std::endl;

        outFile << turnAroundState << std::endl;
        outFile << "1" << std::endl;
        outFile.close();
        std::cout << "Dữ liệu đã được ghi vào file 'output.txt'." << std::endl;
    } 
    else {
        std::cout << "Không thể mở file để ghi." << std::endl;
    }
}


// =================================================================================
CBS::~CBS() {
    // release the memory
    for (auto n : all_nodes)
        delete n;
}

int main(int argc, char *argv[]) {
    MAPFInstance ins;
    string input_file_forGetInput = "embedded/DATN/linefl/GetInputFromUser.txt";
    string input_file_forPreprocess = "embedded/DATN/linefl/DataPrePro.txt";
    string input_file = "embedded/DATN/linefl/exp2_test.txt"; // Output after preprocess data
    string output_file = "embedded/DATN/linefl/exp0_paths.txt";
    string finalOutput_file = "embedded/DATN/linefl/output.txt";
    //==================================================================
    
    std::ifstream file(input_file_forGetInput);
    if (!file.is_open()) {
        std::cout << "Unable to open file." << std::endl;
        return 0;
    }

    int obstacleCount;
    file >> obstacleCount;
    vector<pair<int, int>> obstacles(obstacleCount);

    for (int i = 0; i < obstacleCount; i++) {
        int y, x;
        file >> y >> x;
        obstacles[i] = make_pair(y, x);
    }

    int agentCount;
    file >> agentCount;
    vector<pair<pair<int, int>, pair<int, int>>> agents(agentCount);

    for (int i = 0; i < agentCount; i++) {
        int y, x, destY, destX;

        file >> start >> destination;

        CurrentState = (destination % 18) - (start % 18);
        turnAroundState = 1;

        x = getCoordY(start);
        y = getCoordX(start);
        destX = getCoordY(destination);
        destY = getCoordX(destination);
        agents[i] = make_pair(make_pair(y, x), make_pair(destY, destX));
    }

    
    processInputData(input_file_forPreprocess, input_file, obstacles, agents, agentCount, obstacleCount);
	
//==================================DONOT MODIFY================================
    if (ins.load_instance(input_file)) {
        ins.print_instance();
    } else {
        cout << "Fail to load the instance " << input_file << endl;
        exit(-1);
    }

    CBS cbs(ins);
    ::std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    vector<Path> paths = cbs.find_solution();
    ::std::chrono::steady_clock::duration elapsedTime = ::std::chrono::steady_clock::now() - startTime;
    double duration = ::std::chrono::duration_cast< ::std::chrono::duration< double > >(elapsedTime).count();
    if (paths.empty()) { // Fail to find solutions
        cout << "No solutions!" << endl;
        return 0;
    }

    // print paths
    cout << "Paths:" << endl;
    int sum = 0;
    for (int i = 0; i < ins.num_of_agents; i++) {
        cout << "a" << i << ": " << paths[i] << endl;
        sum += (int)paths[i].size() - 1;
    }
    cout << "Sum of cost: " << sum << endl;
    std::cout << "Milliseconds: " << duration * 1000 << std::endl;

    // save paths
    ofstream myfile (output_file.c_str(), ios_base::out);
    if (myfile.is_open()) {
        for (int i = 0; i < ins.num_of_agents; i++) {
            myfile << paths[i] << endl;
        }
        myfile.close();
    } else {
        cout << "Fail to save the paths to " << output_file << endl;
        exit(-1);
    }
    
//======================================================================
    
    //  Thang
  result = processPaths(paths);

  std::cout << "Output: ";
  for (int value : result) {
    std::cout << value << " ";
  }
  std::cout << std::endl;

  
  finalResult = convertPath(paths);

  std::cout << "Final result: ";
  for (int value1 : finalResult) {
    std::cout << value1 << " ";
  }
  std::cout << std::endl;

    writeFinalOutput(finalOutput_file);

// ========================== Post handle =======================
  PreviousState = CurrentState;
    turnAroundState = 0;
    file.close();
    return 0;
}



