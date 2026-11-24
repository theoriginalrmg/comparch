#include "hazard.h"

/*
 * Full forwarding unit:
 *   EX/MEM -> EX   producer one instruction ahead (ALU result)
 *   MEM/WB -> EX   producer two ahead (ALU result or loaded value)
 *   (three ahead needs nothing: WB writes before ID reads)
 * plus a 1-cycle load-use stall. FP registers (unified numbers 32..63) are
 * forwarded exactly like integer ones.
 */

/* Does the instruction in this slot write register r? */
static int writes(const HazardSlot *s, int r) {
    return s->valid && s->ctrl.dst != REG_NONE && s->ctrl.dst == r;
}

/* Where should EX take source register r from? Newest producer wins. */
static ForwardSel pick(const HazardInput *in, int r) {
    if (r == REG_NONE) return FWD_NONE;                 /* unused operand or x0 */
    /* EX/MEM first: it holds the younger write. A load there has only its
     * address, not its value yet -- the load-use stall below makes sure
     * no consumer is ever in EX at that moment. */
    if (writes(&in->mem, r) && in->mem.ctrl.mem_op != MEMOP_LOAD) return FWD_EX_MEM;
    if (writes(&in->wb, r)) return FWD_MEM_WB;          /* ALU result or loaded value */
    return FWD_NONE;
}

static void decide_forward(const HazardInput *in, HazardDecision *out) {
    /* 1. Forwarding for the instruction now in EX. */
    if (in->ex.valid) {
        out->fwd1 = pick(in, in->ex.ctrl.src1);
        out->fwd2 = pick(in, in->ex.ctrl.src2);
        out->fwd3 = pick(in, in->ex.ctrl.src3);
    }

    /* 2. Load-use: a load in EX whose result the ID instruction needs.
     *    Its value only exists at the end of MEM, one cycle too late. */
    if (in->id.valid && in->ex.valid && in->ex.ctrl.mem_op == MEMOP_LOAD &&
        in->ex.ctrl.dst != REG_NONE) {
        int d = in->ex.ctrl.dst;
        const Control *c = &in->id.ctrl;
        if (c->src1 == d || c->src2 == d || c->src3 == d) out->stall = 1;
    }
}

const HazardDetector hazard_forward = {
    "forward",
    "EX->EX and MEM->EX forwarding, 1-cycle stall on load-use",
    decide_forward,
};
