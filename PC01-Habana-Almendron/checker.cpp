#include "testlib.h"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <map>

using namespace std;

struct Passenger {
    int id;
    int p_r, p_c, d_r, d_c; 
    int pay, patience;
    bool valid; 
};

int get_move_cost(char cell) {
    if (cell == '#') return -1;
    if (cell == 'C') return 3;
    if (cell == 'T') return 5;
    return 1;
}

void check_interactions(int r, int c, int current_time, int CAP, int& current_load, 
                        long long& money, vector<int>& p_state, const vector<Passenger>& passengers) {
    
    // PHASE 1: DROPOFF
    // Passengers get off first to free up space
    for(size_t i=0; i<passengers.size(); i++) {
        if (!passengers[i].valid) continue;
        // p_state 1 means "In Taxi"
        if (p_state[i] == 1 && passengers[i].d_r == r && passengers[i].d_c == c) {
            p_state[i] = 2; // Delivered
            current_load--; 
            money += passengers[i].pay; 
        }
    }

    // PHASE 2: PICKUP
    // Passengers get on if capacity allows and patience is respected
    for(size_t i=0; i<passengers.size(); i++) {
        if (!passengers[i].valid) continue;
        // p_state 0 means "Waiting"
        if (p_state[i] == 0 && passengers[i].p_r == r && passengers[i].p_c == c) {
            // Check capacity BEFORE picking up
            if (current_load < CAP) {
                if (current_time <= passengers[i].patience) {
                    p_state[i] = 1; // Picked up
                    current_load++; 
                }
            } 
        }
    }
    
    // CRITICAL SECURITY CHECK
    // Ensures solution didn't violate capacity constraints implicitly
    if (current_load > CAP) {
        quitf(_wa, "Capacity violation: Taxi has %d passengers (Max allowed: %d)", current_load, CAP);
    }
}

int main(int argc, char* argv[]) {
    registerTestlibCmd(argc, argv);

    // --- READING INPUT ---
    int R = inf.readInt();
    int C = inf.readInt();
    int CAP = inf.readInt();
    int T_MAX = inf.readInt();
    int start_r = inf.readInt();
    int start_c = inf.readInt();

    vector<string> grid(R);
    for (int i = 0; i < R; i++) grid[i] = inf.readToken();

    int N = inf.readInt();
    vector<Passenger> passengers(N);
    map<char, pair<int,int>> locs;
    
    // Map grid locations for pickups (1-9) and dropoffs (A-Z)
    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {
            char ch = grid[r][c];
            if (ch != '.' && ch != '#' && ch != 'C' && ch != 'T') locs[ch] = {r, c};
        }
    }

    for (int i = 0; i < N; i++) {
        passengers[i].id = i;
        passengers[i].valid = true;
        string p_token = inf.readToken(); 
        string d_token = inf.readToken(); 
        char p_char = p_token[0];
        char d_char = d_token[0];
        
        passengers[i].pay = inf.readInt();
        passengers[i].patience = inf.readInt();

        if (locs.count(p_char)) {
            passengers[i].p_r = locs[p_char].first;
            passengers[i].p_c = locs[p_char].second;
        } else passengers[i].valid = false;

        if (locs.count(d_char)) {
            passengers[i].d_r = locs[d_char].first;
            passengers[i].d_c = locs[d_char].second;
        } else passengers[i].valid = false;
    }

    // --- READING USER OUTPUT ---
    long long user_money = ouf.readLong(); 
    int M = ouf.readInt(); 

    // --- SIMULATION ---
    int curr_r = start_r;
    int curr_c = start_c;
    int current_time = 0;
    int current_load = 0;
    long long calculated_money = 0;
    vector<int> p_state(N, 0); // 0: Waiting, 1: In Taxi, 2: Delivered

    // Initial check at start position (t=0)
    check_interactions(curr_r, curr_c, current_time, CAP, current_load, calculated_money, p_state, passengers);

    for(int k=0; k<M; k++) {
        int next_r = ouf.readInt();
        int next_c = ouf.readInt();

        // 1. Validate Geometry
        int dist = abs(next_r - curr_r) + abs(next_c - curr_c);
        if (dist != 1) quitf(_wa, "Invalid move step %d: Jump from (%d,%d) to (%d,%d)", k+1, curr_r, curr_c, next_r, next_c);
        if (next_r < 0 || next_r >= R || next_c < 0 || next_c >= C) quitf(_wa, "Out of bounds at step %d: (%d,%d)", k+1, next_r, next_c);

        // 2. Validate Obstacles
        int cost = get_move_cost(grid[next_r][next_c]);
        if (cost == -1) quitf(_wa, "Hit obstacle at step %d (%d, %d)", k+1, next_r, next_c);

        // 3. Update State
        current_time += cost;
        if (current_time > T_MAX) quitf(_wa, "Time limit exceeded (%d > %d)", current_time, T_MAX);

        curr_r = next_r;
        curr_c = next_c;

        // 4. Process Passengers
        check_interactions(curr_r, curr_c, current_time, CAP, current_load, calculated_money, p_state, passengers);
    }

    // --- FINAL CHECKS ---
    if (curr_r != start_r || curr_c != start_c) quitf(_wa, "Did not return to start position (%d,%d)", start_r, start_c);

    if (calculated_money != user_money) {
        quitf(_wa, "Money mismatch. Output claims %lld, Simulation found %lld", user_money, calculated_money);
    }
    
    // Compare with Judge Answer (if available in .a files)
    if (!ans.seekEof()) {
        long long judge_money = ans.readLong();
        if (user_money < judge_money) quitf(_wa, "Suboptimal solution. User: %lld, Judge: %lld", user_money, judge_money);
    }

    quitf(_ok, "OK! Earnings: %lld, Moves: %d, Time: %d", user_money, M, current_time);
    return 0;
}