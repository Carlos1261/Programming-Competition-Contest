#include "testlib.h"
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <string>
#include <set>
#include <map>

using namespace std;

// Convert linear index to (r, c) coordinates
pair<int, int> get_coord(int index, int C) {
    return {index / C, index % C};
}

int main(int argc, char* argv[]) {
    registerGen(argc, argv, 1);
    
    // Command line parameters
    int R = opt<int>(2); 
    int C = opt<int>(3); 
    int N = opt<int>(4); 
    int CAP = opt<int>(5); 
    double t_mult = opt<double>(6); 
    
    // Pickup zones: digits 1-9 (reused if N > 9)
    string safe_pickups = "123456789"; 
    // Dropoff zones: A-Z excluding C and T
    string safe_dropoffs = "ABDEFGHIJKLMNOPQRSUVWXYZ"; 
    
    // Constraint validation for N <= 12
    if (N > 12) {
        quitf(_fail, "N is limited to 12 for consistency with pickup/dropoff zones.");
    }
    
    // Ensure grid is large enough for start + dropoffs + 9 pickup stops
    if (R * C < (1 + N + 9)) {
        quitf(_fail, "Grid size is too small for %d passengers.", N);
    }
    
    // Taxi starting position
    int start_pos_idx = rnd.next(0, R * C - 1);
    pair<int, int> start = get_coord(start_pos_idx, C);
    
    vector<int> free_cells;
    for(int i = 0; i < R * C; ++i) {
        if (i != start_pos_idx) free_cells.push_back(i);
    }
    shuffle(free_cells.begin(), free_cells.end());
    
    vector<string> grid(R, string(C, '.'));
    vector<pair<int, int>> p_locs_actual(N);
    vector<pair<int, int>> d_locs(N);
    vector<char> p_chars(N), d_chars(N);
    
    // 1. Place the 9 physical pickup zones on the grid
    map<char, pair<int,int>> pickup_map;
    for (int i = 0; i < 9; i++) {
        int idx = free_cells.back(); 
        free_cells.pop_back();
        auto [r, c] = get_coord(idx, C);
        char zone_char = safe_pickups[i];
        grid[r][c] = zone_char;
        pickup_map[zone_char] = {r, c};
    }
    
    // 2. Assign passengers to zones (sharing if N > 9)
    for (int i = 0; i < N; i++) {
        p_chars[i] = safe_pickups[i % 9];
        p_locs_actual[i] = pickup_map[p_chars[i]];
        
        int idx_d = free_cells.back(); 
        free_cells.pop_back();
        auto [dr, dc] = get_coord(idx_d, C);
        d_chars[i] = safe_dropoffs[i]; 
        d_locs[i] = {dr, dc};
        grid[dr][dc] = d_chars[i];
    }
    
    // 3. Place Obstacles (C, T, #)
    for (int idx : free_cells) {
        auto [r, c] = get_coord(idx, C);
        int type = rnd.next(1, 100);
        
        if (type <= 10) grid[r][c] = '#';      // 10% Blocked
        else if (type <= 15) grid[r][c] = 'C'; // 5% Checkpoint (Cost 3)
        else if (type <= 25) grid[r][c] = 'T'; // 10% Traffic (Cost 5)
    }
    
    // 4. Calculate T_MAX based on Manhattan distances
    int max_dist_tour = 0;
    for (int i = 0; i < N; i++) {
        int to_p = abs(p_locs_actual[i].first - start.first) + abs(p_locs_actual[i].second - start.second);
        int to_d = abs(d_locs[i].first - p_locs_actual[i].first) + abs(d_locs[i].second - p_locs_actual[i].second);
        int back = abs(start.first - d_locs[i].first) + abs(start.second - d_locs[i].second);
        max_dist_tour = max(max_dist_tour, to_p + to_d + back);
    }
    
    int T_MAX = max(50, (int)(max_dist_tour * t_mult * 2.0));
    T_MAX = min(T_MAX, 3000); 

    // Output
    cout << R << " " << C << " " << CAP << " " << T_MAX << endl;
    cout << start.first << " " << start.second << endl;
    for (const auto& row : grid) cout << row << endl;
    
    cout << N << endl;
    for (int i = 0; i < N; i++) {
        int pay = (rnd.next(4, 200)) * 5; 
        int dist_p = abs(p_locs_actual[i].first - start.first) + abs(p_locs_actual[i].second - start.second);
        int min_patience = dist_p * 2 + 10; 
        int max_patience = max(min_patience + 20, (int)(T_MAX * 0.8));
        int patience = rnd.next(min_patience, max_patience);
        
        cout << p_chars[i] << " " << d_chars[i] << " " << pay << " " << patience << endl;
    }
    
    return 0;
}