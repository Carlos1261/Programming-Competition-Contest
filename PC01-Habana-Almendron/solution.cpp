#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <tuple>
#include <algorithm>
#include <map>
#include <ctime>

using namespace std;

const int INF = 1e9;
const double TIME_LIMIT = 0.95; 

int dr[] = {-1, 0, 1, 0};
int dc[] = {0, 1, 0, -1};

struct Point {
    int r, c;
    bool operator==(const Point& other) const { return r == other.r && c == other.c; }
};

struct Passenger {
    int id;
    char p_char, d_char;
    int pay, patience;
    int p_id, d_id; 
    Point p_loc, d_loc;
    bool valid;
};

// Guardamos camino detallado
struct PathSegment {
    int cost;
    vector<Point> path;
};

struct State {
    int u;              
    long long picked;   
    long long dropped;  
    int load;           
    int time;           
    int money;          
    int potential;      
    vector<int> path_indices; 

    // Ordenamiento F-Score
    bool operator<(const State& other) const {
        int f1 = money + potential;
        int f2 = other.money + other.potential;
        if (f1 != f2) return f1 > f2; 
        return time < other.time;     
    }
};

int R, C, CAP, T_MAX, N;
Point start_pos;
vector<string> grid;
vector<Passenger> passengers;

int dist_matrix[105][105];
// Guardamos caminos detallados para simulación exacta
PathSegment path_segments[105][105]; 
vector<Point> pois; 

// PODA PARETO
map<tuple<int, long long, long long>, vector<pair<int, int>>> pareto_frontier;

clock_t start_clock;

int get_cost(int r, int c) {
    char cell = grid[r][c];
    if (cell == '#') return INF;
    if (cell == 'C') return 3; 
    if (cell == 'T') return 5; 
    return 1; 
}

// BFS que guarda el camino y el costo
int bfs_path(Point src, Point dest, PathSegment* seg_out = nullptr) {
    if (src.r == -1 || dest.r == -1) return INF;
    if (src == dest) { 
        if (seg_out) { seg_out->cost = 0; seg_out->path.clear(); } 
        return 0; 
    }

    vector<vector<int>> d(R, vector<int>(C, INF));
    vector<vector<Point>> parent(R, vector<Point>(C, {-1, -1}));
    priority_queue<pair<int, pair<int,int>>, vector<pair<int, pair<int,int>>>, greater<>> pq;
    
    d[src.r][src.c] = 0;
    pq.push({0, {src.r, src.c}});
    
    while(!pq.empty()) {
        auto [cost, pos] = pq.top(); auto [r, c] = pos; pq.pop();
        if (cost > d[r][c]) continue;
        if (r == dest.r && c == dest.c) break;
        for(int i=0; i<4; i++) {
            int nr = r + dr[i], nc = c + dc[i];
            if (nr >= 0 && nr < R && nc >= 0 && nc < C && grid[nr][nc] != '#') {
                int move = get_cost(nr, nc);
                if (d[r][c] + move < d[nr][nc]) {
                    d[nr][nc] = d[r][c] + move;
                    parent[nr][nc] = {r, c};
                    pq.push({d[nr][nc], {nr, nc}});
                }
            }
        }
    }
    
    if (d[dest.r][dest.c] == INF) return INF;

    if (seg_out) {
        vector<Point> segment;
        Point curr = dest;
        while (!(curr == src)) {
            segment.push_back(curr);
            curr = parent[curr.r][curr.c];
        }
        reverse(segment.begin(), segment.end());
        seg_out->path = segment;
        seg_out->cost = d[dest.r][dest.c];
    }
    return d[dest.r][dest.c];
}

void process_interactions(int r, int c, int curr_time, int& load, long long& picked, long long& dropped, int& money) {
    // 1. Dropoff
    for (int i = 0; i < N; i++) {
        if (!passengers[i].valid) continue;
        if (((picked >> i) & 1) && !((dropped >> i) & 1)) {
            if (passengers[i].d_loc.r == r && passengers[i].d_loc.c == c) {
                dropped |= (1LL << i); load--; money += passengers[i].pay;
            }
        }
    }
    // 2. Pickup
    for (int i = 0; i < N; i++) {
        if (!passengers[i].valid) continue;
        if (load < CAP && !((picked >> i) & 1)) {
            if (passengers[i].p_loc.r == r && passengers[i].p_loc.c == c) {
                if (curr_time <= passengers[i].patience) { picked |= (1LL << i); load++; }
            }
        }
    }
}

// Simulación EXACTA paso a paso
void simulate_segment(const PathSegment& seg, int start_time, int& time, int& load, long long& picked, long long& dropped, int& money) {
    time = start_time;
    for (const auto& p : seg.path) {
        time += get_cost(p.r, p.c);
        process_interactions(p.r, p.c, time, load, picked, dropped, money);
    }
}

int estimate_potential(int u, int current_time, int load, long long picked, long long dropped) {
    int potential = 0;
    
    // Penalización por capacidad (heurística simple)
    int min_drop_dist = (load == CAP) ? INF : 0;
    if (load == CAP) {
        for(int i=0; i<N; ++i) {
            if (((picked >> i) & 1) && !((dropped >> i) & 1)) {
                min_drop_dist = min(min_drop_dist, dist_matrix[u][passengers[i].d_id]);
            }
        }
        if (min_drop_dist == INF) min_drop_dist = 0; 
    }

    for(int i = 0; i < N; ++i) {
        if (!passengers[i].valid || ((dropped >> i) & 1)) continue;

        if ((picked >> i) & 1) {
            int d_idx = passengers[i].d_id;
            int dist = dist_matrix[u][d_idx];
            if (dist != INF && current_time + dist + dist_matrix[d_idx][0] <= T_MAX) 
                potential += passengers[i].pay;
        } else {
            int p_idx = passengers[i].p_id;
            int d_idx = passengers[i].d_id;
            int dist_p = dist_matrix[u][p_idx];
            int effective_arrival = current_time + dist_p + min_drop_dist;

            if (dist_p != INF) {
                if (effective_arrival <= passengers[i].patience && 
                    effective_arrival + dist_matrix[p_idx][d_idx] + dist_matrix[d_idx][0] <= T_MAX) {
                    potential += passengers[i].pay;
                }
            }
        }
    }
    return potential;
}

// FIX: Poda Pareto Correcta
void add_to_pareto(tuple<int, long long, long long> key, int time, int money) {
    vector<pair<int, int>>& frontier = pareto_frontier[key];
    
    // 1. Verificar si el nuevo punto está dominado
    for (const auto& p : frontier) {
        // Si existe un p con menos tiempo y más dinero, el nuevo es inútil
        if (p.first <= time && p.second >= money) return;
    }
    
    // 2. Eliminar puntos dominados por el nuevo
    vector<pair<int, int>> next_frontier;
    for (const auto& p : frontier) {
        // Si el nuevo tiene menos tiempo y más dinero, p es inútil
        if (time <= p.first && money >= p.second) continue; 
        next_frontier.push_back(p);
    }
    
    // 3. Añadir nuevo punto
    next_frontier.push_back({time, money});
    
    // 4. Limitar tamaño de la frontera (Mantener los mejores extremos)
    if (next_frontier.size() > 20) {
        // Ordenamos por dinero descendente para quedarnos con los más ricos
        sort(next_frontier.begin(), next_frontier.end(), [](const pair<int,int>& a, const pair<int,int>& b){
            return a.second > b.second;
        });
        next_frontier.resize(20);
    }
    
    pareto_frontier[key] = next_frontier;
}

bool is_dominated(tuple<int, long long, long long> key, int time, int money) {
    if (!pareto_frontier.count(key)) return false;
    for (const auto& p : pareto_frontier[key]) {
        if (p.first <= time && p.second >= money) return true;
    }
    return false;
}

int main() {
    ios_base::sync_with_stdio(false); cin.tie(NULL);
    start_clock = clock(); 

    if (!(cin >> R >> C >> CAP >> T_MAX)) return 0;
    cin >> start_pos.r >> start_pos.c;
    grid.resize(R); for(int i=0; i<R; i++) cin >> grid[i];
    cin >> N; passengers.resize(N);
    pois.push_back(start_pos); 
    map<char, Point> locs;
    for(int r=0; r<R; r++) for(int c=0; c<C; c++) {
        char ch = grid[r][c];
        if (ch != '.' && ch != '#' && ch != 'C' && ch != 'T') locs[ch] = {r, c};
    }
    for(int i=0; i<N; i++) {
        cin >> passengers[i].p_char >> passengers[i].d_char >> passengers[i].pay >> passengers[i].patience;
        passengers[i].valid = (locs.count(passengers[i].p_char) && locs.count(passengers[i].d_char));
        if (passengers[i].valid) {
            passengers[i].p_loc = locs[passengers[i].p_char]; pois.push_back(passengers[i].p_loc); passengers[i].p_id = pois.size() - 1;
            passengers[i].d_loc = locs[passengers[i].d_char]; pois.push_back(passengers[i].d_loc); passengers[i].d_id = pois.size() - 1;
        }
    }

    // Precomputación
    for(size_t i=0; i<pois.size(); i++) {
        for(size_t j=0; j<pois.size(); j++) {
            if (i == j) {
                dist_matrix[i][j] = 0; path_segments[i][j].cost = 0;
            } else {
                dist_matrix[i][j] = bfs_path(pois[i], pois[j], &path_segments[i][j]);
            }
        }
    }

    int sl=0, sm=0; long long sp=0, sd=0;
    process_interactions(start_pos.r, start_pos.c, 0, sl, sp, sd, sm);
    
    // BEAM SEARCH
    // Width adaptativo: N=9 -> 2700, N=20 -> 5000
    int ADAPTIVE_BEAM = min(5000, 300 * max(10, N));
    
    vector<State> current_beam;
    current_beam.push_back({0, sp, sd, sl, 0, sm, estimate_potential(0, 0, sl, sp, sd), {0}});

    int max_earnings = sm;
    vector<int> best_poi_sequence = {0};

    while(!current_beam.empty()) {
        if ((double)(clock() - start_clock) / CLOCKS_PER_SEC > TIME_LIMIT) break;

        vector<State> next_beam;
        
        for (const auto& curr : current_beam) {
            auto key = make_tuple(curr.u, curr.picked, curr.dropped);
            
            // Check dominancia antes de expandir
            if (is_dominated(key, curr.time, curr.money)) continue;
            add_to_pareto(key, curr.time, curr.money);

            // 1. Check Home
            int dist_home = dist_matrix[curr.u][0];
            if (dist_home != INF && curr.time + dist_home <= T_MAX) {
                int h_t=curr.time, h_l=curr.load, h_m=curr.money; long long h_p=curr.picked, h_d=curr.dropped;
                simulate_segment(path_segments[curr.u][0], curr.time, h_t, h_l, h_p, h_d, h_m);
                if (h_t <= T_MAX && h_m > max_earnings) {
                    max_earnings = h_m; best_poi_sequence = curr.path_indices;
                }
            }

            // 2. Expandir
            for (int v = 1; v < (int)pois.size(); v++) {
                if (v == curr.u) continue;
                int dist = dist_matrix[curr.u][v];
                if (dist == INF) continue;

                int nt=curr.time, nl=curr.load, nm=curr.money; long long np=curr.picked, nd=curr.dropped;
                simulate_segment(path_segments[curr.u][v], curr.time, nt, nl, np, nd, nm);

                if (nt <= T_MAX) {
                    int n_pot = estimate_potential(v, nt, nl, np, nd);
                    if (nm + n_pot >= max_earnings) {
                        auto next_key = make_tuple(v, np, nd);
                        // Check dominancia antes de añadir al beam
                        if (!is_dominated(next_key, nt, nm)) {
                            vector<int> npath = curr.path_indices; npath.push_back(v);
                            next_beam.push_back({v, np, nd, nl, nt, nm, n_pot, npath});
                        }
                    }
                }
            }
        }

        if (next_beam.empty()) break;

        sort(next_beam.begin(), next_beam.end(), [](const State& a, const State& b) {
            int f1 = a.money + a.potential;
            int f2 = b.money + b.potential;
            if (f1 != f2) return f1 > f2;
            return a.time < b.time;
        });

        if (next_beam.size() > ADAPTIVE_BEAM) {
            next_beam.resize(ADAPTIVE_BEAM);
        }
        current_beam = next_beam;
    }

    if (max_earnings <= 0) { cout << "0\n0\n"; return 0; }

    vector<Point> fm;
    for (size_t i = 0; i < best_poi_sequence.size() - 1; ++i) {
        int u = best_poi_sequence[i], v = best_poi_sequence[i+1];
        fm.insert(fm.end(), path_segments[u][v].path.begin(), path_segments[u][v].path.end());
    }
    
    if (best_poi_sequence.back() != 0) {
        int last = best_poi_sequence.back();
        fm.insert(fm.end(), path_segments[last][0].path.begin(), path_segments[last][0].path.end());
    }

    cout << max_earnings << endl;
    cout << fm.size() << endl; 
    for(const auto& p : fm) cout << p.r << " " << p.c << "\n";

    return 0;
}
