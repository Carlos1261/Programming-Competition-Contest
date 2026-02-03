#include "testlib.h"
#include <set>
#include <string>

using namespace std;

int main(int argc, char* argv[]) {
    registerValidation(argc, argv);

    // Line 1: R C CAP TMAX
    int R = inf.readInt(5, 99, "R");
    inf.readSpace();
    int C = inf.readInt(5, 99, "C");
    inf.readSpace();
    int CAP = inf.readInt(1, 9, "CAP");
    inf.readSpace();
    int TMAX = inf.readInt(10, 3000, "T_MAX");
    inf.readEoln();

    // Line 2: Start Position
    int start_r = inf.readInt(0, R - 1, "START_R");
    inf.readSpace();
    int start_c = inf.readInt(0, C - 1, "START_C");
    inf.readEoln();

    // Grid Validation
    set<char> valid_chars;
    valid_chars.insert('.');
    valid_chars.insert('#');
    valid_chars.insert('C');
    valid_chars.insert('T');
    // Digits 1-9
    for(char c='1'; c<='9'; c++) valid_chars.insert(c);
    // Letters A-Z (excluding C, T)
    for(char c='A'; c<='Z'; c++) if(c!='C' && c!='T') valid_chars.insert(c);

    for (int i = 0; i < R; i++) {
        string row = inf.readToken();
        ensuref((int)row.length() == C, "Row %d length mismatch", i);
        for(char c : row) {
            if (valid_chars.find(c) == valid_chars.end()) {
                quitf(_fail, "Invalid character '%c' in grid at row %d", c, i);
            }
        }
        inf.readEoln();
    }

    // Passengers
    int N = inf.readInt(1, 12, "N");
    inf.readEoln();

    for (int i = 0; i < N; i++) {
        // Format: Pick Drop Pay Patience
        // Pick: single char, digit 1-9
        string p_str = inf.readToken();
        ensuref(p_str.length() == 1, "Pickup zone must be 1 char");
        ensuref(isdigit(p_str[0]) && p_str[0] != '0', "Pickup must be 1-9");
        inf.readSpace();

        // Drop: single char, A-Z except C,T
        string d_str = inf.readToken();
        ensuref(d_str.length() == 1, "Dropoff zone must be 1 char");
        ensuref(isupper(d_str[0]) && d_str[0] != 'C' && d_str[0] != 'T', "Dropoff must be A-Z (no C,T)");
        inf.readSpace();

        int pay = inf.readInt(20, 1000, "Pay");
        inf.readSpace();
        int pat = inf.readInt(5, TMAX, "Patience");
        inf.readEoln();
    }

    inf.readEof();
    return 0;
}