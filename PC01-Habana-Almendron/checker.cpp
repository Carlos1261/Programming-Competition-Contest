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
    
    // FASE 1: BAJAR (Dropoff)
    // Sin check de patience, solo ubicación.
    for(size_t i=0; i<passengers.size(); i++) {
        if (!passengers[i].valid) continue;
        if (p_state[i] == 1 && passengers[i].d_r == r && passengers[i].d_c == c) {
            p_state[i] = 2; // Entregado
            current_load--; // Libera asiento
            money += passengers[i].pay; // Cobra
        }
    }

    // FASE 2: SUBIR (Pickup)
    for(size_t i=0; i<passengers.size(); i++) {
        if (!passengers[i].valid) continue;
        if (current_load < CAP) {
            if (p_state[i] == 0 && passengers[i].p_r == r && passengers[i].p_c == c) {
                // REGLA: Patience solo aplica aquí
                if (current_time <= passengers[i].patience) {
                    p_state[i] = 1; // Subido
                    current_load++; // Ocupa asiento
                }
            }
        }
    }
}

int main(int argc, char* argv[]) {
    registerTestlibCmd(argc, argv);

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
    
    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {
            char ch = grid[r][c];
            if (ch != '.' && ch != '#' && ch != 'C' && ch != 'T') locs[ch] = {r, c};
        }
    }

    for (int i = 0; i < N; i++) {
        passengers[i].id = i;
        passengers[i].valid = true;
        char p_char = inf.readToken()[0];
        char d_char = inf.readToken()[0];
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

    long long user_money = ouf.readLong(); 
    int M = ouf.readInt(); 

    int curr_r = start_r;
    int curr_c = start_c;
    int current_time = 0;
    int current_load = 0;
    long long calculated_money = 0;
    vector<int> p_state(N, 0); 

    check_interactions(curr_r, curr_c, current_time, CAP, current_load, calculated_money, p_state, passengers);

    for(int k=0; k<M; k++) {
        int next_r = ouf.readInt();
        int next_c = ouf.readInt();

        int dist = abs(next_r - curr_r) + abs(next_c - curr_c);
        if (dist != 1) quitf(_wa, "Invalid move step %d", k+1);
        if (next_r < 0 || next_r >= R || next_c < 0 || next_c >= C) quitf(_wa, "Out of bounds");

        int cost = get_move_cost(grid[next_r][next_c]);
        if (cost == -1) quitf(_wa, "Hit obstacle");

        current_time += cost;
        if (current_time > T_MAX) quitf(_wa, "Time limit exceeded");

        curr_r = next_r;
        curr_c = next_c;

        check_interactions(curr_r, curr_c, current_time, CAP, current_load, calculated_money, p_state, passengers);
    }

    if (curr_r != start_r || curr_c != start_c) quitf(_wa, "Did not return to start");

    if (calculated_money != user_money) {
        quitf(_wa, "Money mismatch. Claimed: %lld, Real: %lld", user_money, calculated_money);
    }
    
    if (!ans.seekEof()) {
        long long judge_money = ans.readLong();
        if (user_money < judge_money) quitf(_wa, "Suboptimal. User: %lld, Judge: %lld", user_money, judge_money);
    }

    quitf(_ok, "OK! Earnings: %lld", user_money);
}