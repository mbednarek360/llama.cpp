#include "speculative-adaptive.h"

#include <cassert>
#include <cstdio>

#undef NDEBUG

static void test_reset(void) {
    common_speculative_adaptive ctrl;

    // cold start at the floor max(1, n_min_adaptive), bucket at the drop weight
    ctrl.reset(8, 1);
    assert(ctrl.n_cur == 1);
    assert(ctrl.n_bucket == 20);

    // the default adaptive floor of 3 starts the controller at depth 3
    ctrl.reset(8, 3);
    assert(ctrl.n_cur == 3);
    assert(ctrl.n_bucket == 20);

    // the ceiling clamps the cold start to n_max
    ctrl.reset(1, 3);
    assert(ctrl.n_cur == 1);
    assert(ctrl.n_bucket == 20);
}

static void test_climb(void) {
    common_speculative_adaptive ctrl;
    ctrl.reset(8, 1); // ceiling 8, cold start at the floor 1

    // a full accept at depth 1 adds +1; the cap is 22, so 2 full accepts climb
    ctrl.update(1, 1, 8, 1);
    assert(ctrl.n_cur == 1);
    assert(ctrl.n_bucket == 21);
    ctrl.update(1, 1, 8, 1);
    assert(ctrl.n_cur == 2);
    assert(ctrl.n_bucket == 20);

    // a near miss at depth 2 is neutral: it neither climbs nor resets anything
    ctrl.update(2, 1, 8, 1);
    assert(ctrl.n_cur == 2);
    assert(ctrl.n_bucket == 20);

    // depth 2 climbs after 5 net full accepts (cap 25)
    for (int i = 0; i < 4; ++i) {
        ctrl.update(2, 2, 8, 1);
        assert(ctrl.n_cur == 2);
    }
    ctrl.update(2, 2, 8, 1);
    assert(ctrl.n_cur == 3);
    assert(ctrl.n_bucket == 20);

    // depth 3 is the hardened barrier: 9 net full accepts to reach depth 4 (cap 29)
    for (int i = 0; i < 8; ++i) {
        ctrl.update(3, 3, 8, 1);
        assert(ctrl.n_cur == 3);
    }
    ctrl.update(3, 3, 8, 1);
    assert(ctrl.n_cur == 4);
    assert(ctrl.n_bucket == 20);

    // a draft truncated one token short of the depth is neutral, exactly like
    // a full-length draft that fell one token short
    ctrl.update(3, 3, 8, 1); // depth 4, only 3 tokens drafted, all accepted
    assert(ctrl.n_cur == 4);
    assert(ctrl.n_bucket == 20);

    // depth 4 needs 6 net full accepts (cap 26)
    for (int i = 0; i < 5; ++i) {
        ctrl.update(4, 4, 8, 1);
    }
    assert(ctrl.n_cur == 4);
    ctrl.update(4, 4, 8, 1);
    assert(ctrl.n_cur == 5);
    assert(ctrl.n_bucket == 25);

    // depth 5 needs 5 net full accepts (cap 30)
    for (int i = 0; i < 4; ++i) {
        ctrl.update(5, 5, 8, 1);
    }
    assert(ctrl.n_cur == 5);
    ctrl.update(5, 5, 8, 1);
    assert(ctrl.n_cur == 6);
    assert(ctrl.n_bucket == 30);

    // depth 6 needs 4 net full accepts (cap 34)
    for (int i = 0; i < 3; ++i) {
        ctrl.update(6, 6, 8, 1);
    }
    assert(ctrl.n_cur == 6);
    ctrl.update(6, 6, 8, 1);
    assert(ctrl.n_cur == 7);
    assert(ctrl.n_bucket == 35);

    // depth 7 needs 3 net full accepts (cap 38)
    for (int i = 0; i < 2; ++i) {
        ctrl.update(7, 7, 8, 1);
    }
    assert(ctrl.n_cur == 7);
    ctrl.update(7, 7, 8, 1);
    assert(ctrl.n_cur == 8);
    assert(ctrl.n_bucket == 40);

    // at the ceiling the bucket re-baselines instead of growing unbounded
    for (int i = 0; i < 8; ++i) {
        ctrl.update(8, 8, 8, 1);
    }
    assert(ctrl.n_cur == 8);
    assert(ctrl.n_bucket == 40);

    // no feedback for a zero-length draft
    ctrl.update(0, 0, 8, 1);
    assert(ctrl.n_cur == 8);
    assert(ctrl.n_bucket == 40);
}

static void test_drop(void) {
    common_speculative_adaptive ctrl;
    ctrl.reset(8, 1); // cold start at the floor

    // at the floor a total miss adds 0 (1 accepted + 1 - depth 1), free forever
    for (int i = 0; i < 100; ++i) {
        ctrl.update(1, 0, 8, 1);
    }
    assert(ctrl.n_cur == 1);
    assert(ctrl.n_bucket == 20);

    // climb to depth 3 (2 + 5 full accepts)
    for (int i = 0; i < 2; ++i) {
        ctrl.update(1, 1, 8, 1);
    }
    for (int i = 0; i < 5; ++i) {
        ctrl.update(2, 2, 8, 1);
    }
    assert(ctrl.n_cur == 3);

    // at depth 3 a total miss adds -2, so 10 misses drain the bucket of 20
    for (int i = 0; i < 9; ++i) {
        ctrl.update(3, 0, 8, 1);
        assert(ctrl.n_cur == 3);
    }
    assert(ctrl.n_bucket == 2);
    ctrl.update(3, 0, 8, 1);
    assert(ctrl.n_cur == 2);
    assert(ctrl.n_bucket == 20);

    // at depth 2 a near miss is exactly neutral: the depth hovers, it does
    // not decay on one-token shortfalls
    for (int i = 0; i < 100; ++i) {
        ctrl.update(2, 1, 8, 1);
    }
    assert(ctrl.n_cur == 2);
    assert(ctrl.n_bucket == 20);

    // at depth 2 a total miss adds -1, so 20 misses drop one step
    for (int i = 0; i < 19; ++i) {
        ctrl.update(2, 0, 8, 1);
        assert(ctrl.n_cur == 2);
    }
    assert(ctrl.n_bucket == 1);
    ctrl.update(2, 0, 8, 1);
    assert(ctrl.n_cur == 1);
    assert(ctrl.n_bucket == 20);

    // back at the floor, misses stay free
    for (int i = 0; i < 100; ++i) {
        ctrl.update(1, 0, 8, 1);
    }
    assert(ctrl.n_cur == 1);
    assert(ctrl.n_bucket == 20);

    // deep depths fall quickly: at depth 5 a total miss adds -4, so 7 misses
    // drain the bucket of 25
    ctrl.reset(8, 1);
    ctrl.n_cur    = 5; // simulate a controller that already climbed to 5
    ctrl.n_bucket = 25;
    for (int i = 0; i < 6; ++i) {
        ctrl.update(5, 0, 8, 1);
        assert(ctrl.n_cur == 5);
    }
    assert(ctrl.n_bucket == 1);
    ctrl.update(5, 0, 8, 1);
    assert(ctrl.n_cur == 4);
    assert(ctrl.n_bucket == 20);
}

static void test_momentum(void) {
    common_speculative_adaptive ctrl;
    ctrl.reset(8, 1);
    ctrl.n_cur    = 3;
    ctrl.n_bucket = 20;

    // a bad patch drains the bucket: 5 total misses at depth 3 add -2 each
    for (int i = 0; i < 5; ++i) {
        ctrl.update(3, 0, 8, 1); // total miss
    }
    assert(ctrl.n_bucket == 10);
    assert(ctrl.n_cur == 3);

    // full accepts do not wipe the damage: they recover the bucket one token
    // at a time, and 10 of them only bring it back to the reset point
    for (int i = 0; i < 10; ++i) {
        ctrl.update(3, 3, 8, 1);
    }
    assert(ctrl.n_bucket == 20);
    assert(ctrl.n_cur == 3); // no climb: the old controller would have climbed here

    // 9 more full accepts reach the hardened cap of 29 and climb
    for (int i = 0; i < 9; ++i) {
        ctrl.update(3, 3, 8, 1);
    }
    assert(ctrl.n_cur == 4);
    assert(ctrl.n_bucket == 20);
}

static void test_truncation(void) {
    common_speculative_adaptive ctrl;
    ctrl.reset(8, 1);
    ctrl.n_cur    = 5;
    ctrl.n_bucket = 25;

    // truncated one token short of the depth: neutral, like a near miss
    ctrl.update(4, 4, 8, 1);
    assert(ctrl.n_bucket == 25);

    // truncated two tokens short: -1
    ctrl.update(3, 3, 8, 1);
    assert(ctrl.n_bucket == 24);

    // truncated three tokens short: -2
    ctrl.update(2, 2, 8, 1);
    assert(ctrl.n_bucket == 22);

    // truncated and partly rejected drains more
    ctrl.update(2, 1, 8, 1);
    assert(ctrl.n_bucket == 19);

    // a full-length near miss is neutral, a full-length total miss is -4
    ctrl.update(5, 4, 8, 1);
    assert(ctrl.n_bucket == 19);
    ctrl.update(5, 0, 8, 1);
    assert(ctrl.n_bucket == 15);
}

static void test_floor(void) {
    common_speculative_adaptive ctrl;

    // with the floor at 2 the depth never drops below 2, no matter how bad
    // the content gets, and the bucket stays at the reset point (misses free)
    ctrl.reset(8, 2);
    for (int i = 0; i < 1000; ++i) {
        ctrl.update(2, 0, 8, 2);
    }
    assert(ctrl.n_cur == 2);
    assert(ctrl.n_bucket == 20);

    // climbs still work from the floor
    for (int i = 0; i < 5; ++i) {
        ctrl.update(2, 2, 8, 2);
    }
    assert(ctrl.n_cur == 3);
    assert(ctrl.n_bucket == 20);

    // drops stop at the floor, not below it
    for (int i = 0; i < 10; ++i) {
        ctrl.update(3, 0, 8, 2);
    }
    assert(ctrl.n_cur == 2);
    assert(ctrl.n_bucket == 20);
    for (int i = 0; i < 90; ++i) {
        ctrl.update(2, 0, 8, 2);
    }
    assert(ctrl.n_cur == 2);
    assert(ctrl.n_bucket == 20);
}

int main(void) {
    test_reset();
    test_climb();
    test_drop();
    test_momentum();
    test_truncation();
    test_floor();

    printf("test-speculative-adaptive: all tests OK\n\n");

    return 0;
}
