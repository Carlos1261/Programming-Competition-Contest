/**
 * Problem: HaBana Almendron Routes
 * Method:  Beam Search on a Compressed Graph with State Pooling
 * * Complexity Analysis:
 * --------------------
 * Time Complexity:  O(P * R * C) + O(K * W * P * N)
 * - Precomputation: P BFS runs on the grid (P = Points of Interest).
 * - Search: K steps (depth) * W (Beam Width) * P (transitions) * N (passenger checks).
 * * Space Complexity: O(K * W) + O(P^2 * D)
 * - State Pool: Stores approximately K * W active states.
 * - Path Cache: Stores reconstruction paths between POIs.
 * * Variables:
 * P = POIs (~2N), W = Beam Width (~3000), K = Path Length, N = Passengers.
 */

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <algorithm>
#include <map>
#include <ctime>
#include <cstring>

using namespace std;

// Configuration Constants
const int INF = 1e9;
const double TIME_LIMIT = 0.95; 

// Beam Search Parameters
// Adaptive width strategy: Start wide to find complex paths, narrow down to save memory.
const int BEAM_WIDTH_INITIAL = 2500; 
const int BEAM_WIDTH_MIN = 800;      

// Data Structures

struct Point {
    int r, c;
    Point() : r(0), c(0) {}              
    Point(int r, int c) : r(r), c(c) {}  
    bool operator==(const Point& o) const { return r == o.r && c == o.c; }
    bool operator<(const Point& o) const { return r < o.r || (r == o.r && c < o.c); }
};

struct Passenger {
    int id;
    char p_char, d_char;
    int pay, patience;
    int p_node_idx, d_node_idx; 
    bool valid;
};

// Lightweight reference used for sorting states without moving heavy objects
struct StateRef {
    int idx;     // Index in the memory pool
    int f_score; // Ranking criteria (Money + Potential Heuristic)
    int time;    // Tie-breaker: Time elapsed

    // Descending sort operator (Highest score is best)
    bool operator>(const StateRef& other) const {
        if (f_score != other.f_score) return f_score > other.f_score; 
        return time < other.time; 
    }
};

// Main state object stored in the persistent memory pool
struct StateData {
    int u;               // Current location index
    int picked, dropped; // Bitmasks tracking passenger status (32-bit int sufficient for N<=20)
    int load;            // Current number of passengers in car
    int time;            // Elapsed time
    int money;           // Current earnings
    int potential;       // Heuristic estimate of future earnings
    int parent_idx;      // Pointer to previous state for path reconstruction
};

// Represents a POI encountered while traversing a segment between two main POIs
struct IntermediateNode {
    int node_idx;
    int time_offset;
};

// Global Variables
int R, C, CAP, T_MAX, N;
Point start_pos;
vector<string> grid;
vector<Passenger> passengers;

map<Point, int> point_to_id;
vector<Point> id_to_point;

// Precomputed Adjacency and Path Tables
vector<vector<int>> dist_matrix;
vector<vector<vector<IntermediateNode>>> segment_nodes;
vector<vector<vector<Point>>> path_cache;

// Memory Pool
vector<StateData> state_pool;

clock_t start_time;

// Helper Functions

inline int get_cost(char c) {
    if (c == '#') return INF;
    if (c == 'C') return 3;
    if (c == 'T') return 5;
    return 1;
}

int get_point_id(Point p) {
    if (point_to_id.find(p) == point_to_id.end()) {
        point_to_id[p] = id_to_point.size();
        id_to_point.push_back(p);
    }
    return point_to_id[p];
}

// Calculates All-Pairs Shortest Paths from a source POI to the entire grid
// Populates the distance matrix and caches the physical path for reconstruction
void bfs_precompute(int src_idx, int num_pois) {
    Point src = id_to_point[src_idx];
    vector<vector<int>> d(R, vector<int>(C, INF));
    vector<vector<Point>> parent(R, vector<Point>(C, Point(-1, -1)));
    priority_queue<pair<int, Point>, vector<pair<int, Point>>, greater<>> pq;

    d[src.r][src.c] = 0;
    pq.push({0, src});

    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};

    while (!pq.empty()) {
        auto [dist, pos] = pq.top(); pq.pop();
        if (dist > d[pos.r][pos.c]) continue;

        for (int i = 0; i < 4; i++) {
            int nr = pos.r + dr[i], nc = pos.c + dc[i];
            if (nr >= 0 && nr < R && nc >= 0 && nc < C) {
                int cost = get_cost(grid[nr][nc]);
                if (cost != INF && d[pos.r][pos.c] + cost < d[nr][nc]) {
                    d[nr][nc] = d[pos.r][pos.c] + cost;
                    parent[nr][nc] = pos;
                    pq.push({d[nr][nc], Point(nr, nc)});
                }
            }
        }
    }

    // Extract paths and distances to other POIs
    for (int dest_idx = 0; dest_idx < num_pois; dest_idx++) {
        Point dest = id_to_point[dest_idx];
        dist_matrix[src_idx][dest_idx] = d[dest.r][dest.c];

        if (d[dest.r][dest.c] != INF && src_idx != dest_idx) {
            vector<Point> path;
            Point curr = dest;
            while (!(curr == src)) {
                path.push_back(curr);
                curr = parent[curr.r][curr.c];
            }
            reverse(path.begin(), path.end());
            path_cache[src_idx][dest_idx] = path;

            // Identify any other POIs located along this path
            int current_time_offset = 0;
            for (const Point& p : path) {
                current_time_offset += get_cost(grid[p.r][p.c]);
                if (point_to_id.count(p)) {
                    int intermediate_id = point_to_id[p];
                    if (intermediate_id != dest_idx && intermediate_id != src_idx) {
                        segment_nodes[src_idx][dest_idx].push_back({intermediate_id, current_time_offset});
                    }
                }
            }
        }
    }
}

// Simulates picking up and dropping off passengers at a specific node/time
void process_interactions(int u_idx, int arrival_time, int& load, int& picked, int& dropped, int& money) {
    // 1. Process Drop-offs
    for (int i = 0; i < N; i++) {
        if (!passengers[i].valid) continue;
        if (((picked >> i) & 1) && !((dropped >> i) & 1)) {
            if (passengers[i].d_node_idx == u_idx) {
                dropped |= (1 << i); 
                load--; 
                money += passengers[i].pay;
            }
        }
    }
    
    // 2. Identify Available Pick-ups
    vector<int> available;
    for (int i = 0; i < N; i++) {
        if (!passengers[i].valid) continue;
        if (!((picked >> i) & 1) && passengers[i].p_node_idx == u_idx) {
            if (arrival_time <= passengers[i].patience) available.push_back(i);
        }
    }
    
    // 3. Greedy Pick-up Strategy: Prioritize higher payments
    sort(available.begin(), available.end(), [](int a, int b) { 
        return passengers[a].pay > passengers[b].pay; 
    });
    
    for (int pid : available) {
        if (load < CAP) { 
            picked |= (1 << pid); 
            load++; 
        }
    }
}

// Heuristic Function: Estimates maximum potential earnings from current state
// Uses precomputed real distances to verify feasibility
int estimate_potential(int u_idx, int t, int p, int d) {
    int pot = 0;
    vector<int> candidates; // Stores potential earnings from viable passengers

    for(int i = 0; i < N; ++i) {
        // Skip if invalid, already picked (bit p), or already dropped (bit d)
        if (!passengers[i].valid || ((d >> i) & 1)) continue;
        
        // Calculate distances (O(1) lookup)
        // If already picked ((p >> i) & 1), distance to pickup is 0.
        int d_to_p = ((p >> i) & 1) ? 0 : dist_matrix[u_idx][passengers[i].p_node_idx];
        int d_p_to_d = dist_matrix[passengers[i].p_node_idx][passengers[i].d_node_idx];
        int d_home = dist_matrix[passengers[i].d_node_idx][0];

        // Feasibility Check
        int arrival_at_pickup = t + d_to_p;
        int arrival_at_drop = arrival_at_pickup + d_p_to_d;
        int finish_time = arrival_at_drop + d_home;

        // Condition 1: Must be reachable
        // Condition 2: Must arrive at pickup before patience runs out
        // Condition 3: Must be able to return home before T_MAX
        if (d_to_p != INF && d_p_to_d != INF && d_home != INF && 
            finish_time <= T_MAX && 
            arrival_at_pickup <= passengers[i].patience) {
            
            // Base score is the payment
            int score = passengers[i].pay;
            
            // Bonus for urgency: If patience is tight, this passenger is critical.
            // We verify if we are "close" to the deadline.
            int slack = passengers[i].patience - arrival_at_pickup;
            
            // If we are already carrying them, urgency is irrelevant (we secured them), 
            // but if we haven't picked them up, tight slack increases heuristic value 
            // to encourage this branch.
            if (!((p >> i) & 1)) {
                 if (slack < 50) score += (50 - slack) * 2; // Small bonus for urgency
            }

            candidates.push_back(score);
        }
    }

    // Instead of summing everyone (impossible), sum only the Top K candidates.
    // K = CAP * 1.5 (approx 6 passengers) covers immediate future + next batch.
    sort(candidates.begin(), candidates.end(), greater<int>());

    int limit = min((int)candidates.size(), 6); // Lookahead horizon
    for(int k = 0; k < limit; ++k) {
        pot += candidates[k];
    }

    return pot;
}

int main() {
    ios_base::sync_with_stdio(false); cin.tie(NULL);
    start_time = clock();

    // Input Reading
    if (!(cin >> R >> C >> CAP >> T_MAX)) return 0;
    cin >> start_pos.r >> start_pos.c;
    grid.resize(R); for (int i = 0; i < R; i++) cin >> grid[i];
    cin >> N; passengers.resize(N);

    get_point_id(start_pos); // Ensure Start is ID 0

    // Map all relevant locations
    map<char, Point> locs;
    for (int r = 0; r < R; r++) for (int c = 0; c < C; c++) 
        if (grid[r][c] != '.' && grid[r][c] != '#' && grid[r][c] != 'C' && grid[r][c] != 'T') 
            locs[grid[r][c]] = Point(r, c);

    // Robust Passenger Parsing
    // Handles formats with ID (e.g., "1 P D...") and without ID (e.g., "P D...")
    for (int i = 0; i < N; i++) {
        passengers[i].id = i;
        string t1, t2; cin >> t1 >> t2;
        
        if (isdigit(t1[0]) && isdigit(t2[0])) { 
            // Input Format: ID Pickup Dropoff ...
            passengers[i].p_char = t2[0]; 
            cin >> passengers[i].d_char >> passengers[i].pay >> passengers[i].patience;
        } else {
            // Input Format: Pickup Dropoff ...
            passengers[i].p_char = t1[0]; 
            passengers[i].d_char = t2[0]; 
            cin >> passengers[i].pay >> passengers[i].patience;
        }
        
        passengers[i].valid = locs.count(passengers[i].p_char) && locs.count(passengers[i].d_char);
        if (passengers[i].valid) {
            passengers[i].p_node_idx = get_point_id(locs[passengers[i].p_char]);
            passengers[i].d_node_idx = get_point_id(locs[passengers[i].d_char]);
        }
    }

    // Precomputation Phase
    int num_pois = id_to_point.size();
    if (num_pois == 0) { cout << "0\n0\n"; return 0; }

    dist_matrix.assign(num_pois, vector<int>(num_pois, INF));
    path_cache.assign(num_pois, vector<vector<Point>>(num_pois));
    segment_nodes.assign(num_pois, vector<vector<IntermediateNode>>(num_pois));

    for (int i = 0; i < num_pois; i++) bfs_precompute(i, num_pois);

    // Initialize Memory Pool
    // Reserve sufficient memory to prevent runtime reallocation overhead
    state_pool.reserve(3500000); 
    
    StateData root;
    root.u = 0; root.picked = 0; root.dropped = 0; root.load = 0; root.time = 0; root.money = 0;
    root.parent_idx = -1;
    
    // Process initial state interactions (e.g., picking up at start)
    process_interactions(0, 0, root.load, root.picked, root.dropped, root.money);
    root.potential = estimate_potential(0, 0, root.picked, root.dropped);

    state_pool.push_back(root);

    vector<StateRef> beam; 
    beam.push_back({0, root.money + root.potential, 0});

    int best_money_at_home = -1;
    int best_state_idx = -1;

    if (dist_matrix[0][0] == 0) {
        best_money_at_home = root.money;
        best_state_idx = 0;
    }

    // Beam Search Loop
    while (!beam.empty()) {
        if ((double)(clock() - start_time) / CLOCKS_PER_SEC > TIME_LIMIT) break;
        
        // Emergency memory safety check
        if (state_pool.size() > 3400000) break;

        vector<StateData> candidates;
        vector<StateRef> next_gen_refs; 

        for (const auto& ref : beam) {
            StateData curr = state_pool[ref.idx]; 

            // Option 1: Try to Return Home (Close the loop)
            if (curr.u != 0) { 
               int d_home = dist_matrix[curr.u][0];
               if (d_home != INF && curr.time + d_home <= T_MAX) {
                   StateData home_state = curr;
                   home_state.parent_idx = ref.idx;
                   
                   int base_time = curr.time;
                   // Simulate passing through intermediate nodes
                   for(const auto& node : segment_nodes[curr.u][0]) {
                        process_interactions(node.node_idx, base_time + node.time_offset, 
                                             home_state.load, home_state.picked, home_state.dropped, home_state.money);
                   }
                   
                   home_state.time += d_home;
                   home_state.u = 0;
                   process_interactions(0, home_state.time, home_state.load, home_state.picked, home_state.dropped, home_state.money);

                   if (home_state.money > best_money_at_home) {
                       best_money_at_home = home_state.money;
                       state_pool.push_back(home_state);
                       best_state_idx = state_pool.size() - 1;
                   }
               }
            }

            // Option 2: Expand to other Points of Interest
            for (int v = 0; v < num_pois; v++) {
                if (v == curr.u) continue;
                int dist = dist_matrix[curr.u][v];
                if (dist == INF || curr.time + dist > T_MAX) continue;

                StateData next = curr;
                next.parent_idx = ref.idx;
                int base_time = curr.time;
                
                // Simulate interactions at intermediate nodes along the edge
                for(const auto& node : segment_nodes[curr.u][v]) {
                    process_interactions(node.node_idx, base_time + node.time_offset, 
                                         next.load, next.picked, next.dropped, next.money);
                }
                
                next.time += dist;
                next.u = v;
                // Simulate interactions at the destination node
                process_interactions(v, next.time, next.load, next.picked, next.dropped, next.money);

                // Add valid state to candidate list
                if (next.time <= T_MAX) {
                    next.potential = estimate_potential(v, next.time, next.picked, next.dropped);
                    candidates.push_back(next);
                    next_gen_refs.push_back({(int)candidates.size() - 1, next.money + next.potential, next.time});
                }
            }
        }

        beam.clear();
        
        // Adjust beam width based on elapsed time to optimize final search depth
        double elapsed = (double)(clock() - start_time) / CLOCKS_PER_SEC;
        int width = (elapsed > 0.6) ? BEAM_WIDTH_MIN : BEAM_WIDTH_INITIAL;
        
        if (next_gen_refs.empty()) break;

        // Selection Phase: Keep only the Top K candidates
        if ((int)next_gen_refs.size() > width) {
            // Linear time selection (O(N))
            nth_element(next_gen_refs.begin(), next_gen_refs.begin() + width, next_gen_refs.end(), 
                        [](const StateRef& a, const StateRef& b) {
                            return a > b; 
                        });
            next_gen_refs.resize(width);
            
            // Sort selected candidates to process most promising branches first
            sort(next_gen_refs.begin(), next_gen_refs.end(), [](const StateRef& a, const StateRef& b) {
                return a > b;
            });
        }

        // Commit survivors to the persistent memory pool
        for (const auto& ref : next_gen_refs) {
            state_pool.push_back(candidates[ref.idx]);
            beam.push_back({(int)state_pool.size() - 1, ref.f_score, ref.time});
        }
    }

    if (best_state_idx == -1) { cout << "0\n0\n"; return 0; }

    // Reconstruct the full path
    vector<int> node_seq;
    int curr_idx = best_state_idx;
    while(curr_idx != -1) {
        node_seq.push_back(state_pool[curr_idx].u);
        curr_idx = state_pool[curr_idx].parent_idx;
    }
    reverse(node_seq.begin(), node_seq.end());

    // Expand POI sequence into full coordinate path
    vector<Point> final_coords;
    if (node_seq.size() > 1) {
        for (size_t i = 0; i < node_seq.size() - 1; ++i) {
            int u = node_seq[i];
            int v = node_seq[i+1];
            const vector<Point>& segment = path_cache[u][v];
            final_coords.insert(final_coords.end(), segment.begin(), segment.end());
        }
    }

    // Output Result
    cout << best_money_at_home << "\n";
    cout << final_coords.size() << "\n";
    for (const Point& p : final_coords) cout << p.r << " " << p.c << "\n";

    return 0;
}