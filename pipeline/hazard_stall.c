#include "hazard.h"

/*
 * Naive interlock, no forwarding: the instruction in ID may only read the
 * register file once every older producer of its sources has written back.
 * (WB writes before ID reads in the same cycle.)
 */

/* Does the instruction in this slot write register r? */
static int writes(const HazardSlot *s, int r) {
    return s->valid && s->ctrl.dst != REG_NONE && s->ctrl.dst == r;
}

/* Is source r of the ID instruction still being produced in EX or MEM?
 * (A producer in WB is fine: it writes before ID reads this cycle.) */
static int pending(const HazardInput *in, int r) {
    if (r == REG_NONE) return 0;            /* unused operand or x0 */
    return writes(&in->ex, r) || writes(&in->mem, r);
}

static void decide_stall(const HazardInput *in, HazardDecision *out) {
    if (!in->id.valid) return;
    const Control *c = &in->id.ctrl;
    if (pending(in, c->src1) || pending(in, c->src2) || pending(in, c->src3))
        out->stall = 1;
}

const HazardDetector hazard_stall = {
    "stall",
    "stall in ID until producers write back; no forwarding",
    decide_stall,
};
