#pragma once

#include <algorithm>

// Adaptive draft depth controller for MTP speculative decoding (draft-mtp-adaptive).
//
// Single-bucket state machine. Every verification adds (n_accepted + 1 - depth)
// to a bucket B: a full accept adds +1, a near miss (depth-1 accepted) adds 0,
// and a miss of k tokens subtracts k-1, so the depth hovers at the acceptance
// level instead of resetting on every miss. Truncated drafts are losses too,
// but gentler: every drafted token was accepted, the p_min gate just stopped
// the draft short of the depth, so their drop pressure is halved but always
// at least 1, so a high p_min cannot pin the depth above the draft length.
//
// When B reaches the cap T = drop_pressure(depth) + climb_threshold(depth) the
// depth climbs one step and B resets to drop_pressure(depth); when B falls to
// 0 the depth drops one step and B resets to drop_pressure(depth). From the
// reset point climbing costs climb_threshold(depth) net full accepts and
// dropping costs drop_pressure(depth) net misses, so the cost table keeps its
// shape: fast off the floor, a barrier at 3->4 where marginal content
// collapses, fast again at depth. At the floor B clamps at 0 instead of
// dropping the depth; at the ceiling B clamps at the cap instead of growing
// unbounded.
struct common_speculative_adaptive {
    int n_cur    = 0; // current adaptive draft depth N
    int n_bucket = 0; // accumulated bucket: net accepted surplus over the depth

    // consecutive full accepts needed to climb one step from depth N; low at the
    // floor and at depth, high in the middle where acceptance is marginal
    static int climb_threshold(int depth) {
        switch (depth) {
            case 1:  return 2;
            case 2:  return 5;
            case 3:  return 10; // hardened barrier: marginal content must really earn 3->4
            case 4:  return 5;
            case 5:  return 4;
            case 6:  return 3;
            default: return 2; // depth >= 7
        }
    }

    // accumulated (n_draft - n_accepted) needed to drop one step from depth N;
    // scaled by depth, with a floor so shallow depths do not collapse too fast
    static int drop_pressure(int depth) {
        return std::max(depth * 5, 20);
    }

    // bucket cap: drop pressure plus the climb cost, so climbing from the reset
    // point costs climb_threshold(depth) net full accepts
    static int bucket_cap(int depth) {
        return drop_pressure(depth) + climb_threshold(depth);
    }

    // reset to the floor max(1, n_min_adaptive), bounded by the ceiling n_max;
    // the controller climbs from there once acceptance feedback arrives
    void reset(int n_max, int n_min_adaptive) {
        const int cap   = std::max(1, n_max);
        const int floor = std::max(1, n_min_adaptive);

        n_cur    = std::min(floor, cap);
        n_bucket = drop_pressure(n_cur);
    }

    // feed one verification result: n_draft is the number of tokens this
    // implementation drafted, n_accepted the number the target accepted
    void update(int n_draft, int n_accepted, int n_max, int n_min_adaptive) {
        if (n_draft <= 0) {
            return;
        }

        const int cap   = std::max(1, n_max);
        const int floor = std::max(1, n_min_adaptive);

        int delta;

        if (n_accepted == n_draft && n_draft < n_cur) {
            // truncated drafts are gentle losses: the p_min gate stopped the
            // draft short, so halve the drop pressure but never below 1
            delta = (n_accepted - n_cur - 1) / 2;
        } else {
            delta = n_accepted + 1 - n_cur;
        }

        n_bucket += delta;

        // At the bucket boundaries adjust the current depth if possible. Otherwise
        // both n_cur and n_bucket always get clamped to their minimum/maximum values
        if (n_bucket >= bucket_cap(n_cur)) {
            // At or above the high water mark
            if (n_cur < cap) {
                n_cur++;
                n_bucket = drop_pressure(n_cur);
            } else {
                n_cur = cap;
                n_bucket = bucket_cap(n_cur);
            }
        } else if (n_bucket <= 0) {
            // At or below low water mark
            if (n_cur > floor) {
                n_cur--;
                n_bucket = drop_pressure(n_cur);
            } else {
                n_cur = floor;
                n_bucket = 0;
            }
        }
    }
};
