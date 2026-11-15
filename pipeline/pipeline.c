#include "pipeline_state.h"
#include "control.h"
#include "../core/decoder.h"
#include "../ui/cli_display.h"

#include <stdio.h>
#include <string.h>

/* Opcodes the EX stage needs to tell apart (same values as isa/rv32i_base.c) */
#define OP_R      0x33
#define OP_IARITH 0x13
#define OP_LOAD   0x03
#define OP_STORE  0x23
#define OP_JALR   0x67
#define OP_JAL    0x6F
#define OP_LUI    0x37
#define OP_AUIPC  0x17

void pipeline_init(Pipeline *p) {
    memset(p, 0, sizeof(*p));
}

/* ------------------------------------------------------------------ */
/* ALU used by EX for everything that is not a branch or a jump.       */
/* a = rs1_val, b = rs2_val. I-type ops use imm instead of b.          */
/* ------------------------------------------------------------------ */
static uint32_t alu(const DecodedInstr *i, uint32_t pc, uint32_t a, uint32_t b) {
    uint32_t imm = (uint32_t)i->imm;

    switch (i->opcode) {
        case OP_LUI:   return imm;
        case OP_AUIPC: return pc + imm;
        case OP_LOAD:
        case OP_STORE: return a + imm;            /* effective address */

        case OP_R:
            switch (i->funct3) {
                case 0x0: return (i->funct7 == 0x20) ? a - b : a + b;   /* sub / add */
                case 0x1: return a << (b & 0x1F);                        /* sll */
                case 0x2: return ((int32_t)a < (int32_t)b) ? 1 : 0;      /* slt */
                case 0x3: return (a < b) ? 1 : 0;                        /* sltu */
                case 0x4: return a ^ b;                                  /* xor */
                case 0x5: return (i->funct7 == 0x20)                     /* sra / srl */
                              ? (uint32_t)((int32_t)a >> (b & 0x1F))
                              : a >> (b & 0x1F);
                case 0x6: return a | b;                                  /* or */
                case 0x7: return a & b;                                  /* and */
            }
            break;

        case OP_IARITH:
            switch (i->funct3) {
                case 0x0: return a + imm;                                /* addi */
                case 0x1: return a << (imm & 0x1F);                      /* slli */
                case 0x2: return ((int32_t)a < i->imm) ? 1 : 0;          /* slti */
                case 0x3: return (a < imm) ? 1 : 0;                      /* sltiu */
                case 0x4: return a ^ imm;                                /* xori */
                case 0x5: return (i->funct7 == 0x20)                     /* srai / srli */
                              ? (uint32_t)((int32_t)a >> (imm & 0x1F))
                              : a >> (imm & 0x1F);
                case 0x6: return a | imm;                                /* ori */
                case 0x7: return a & imm;                                /* andi */
            }
            break;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Task 4 hazard check (naive): stall if a source register of the     */
/* instruction in ID is the destination of ANY older instruction      */
/* still in ID/EX, EX/MEM or MEM/WB. No forwarding.                   */
/* ------------------------------------------------------------------ */
static int writes_reg(int valid, const ControlSignals *c, uint32_t rd, uint32_t r) {
    return valid && c->reg_write && rd != 0 && rd == r;
}

static int depends_on(const Pipeline *p, uint32_t r) {
    if (r == 0) return 0;  /* x0 never causes a hazard */
    return writes_reg(p->id_ex.valid,  &p->id_ex.ctrl,  p->id_ex.instr.rd,  r) ||
           writes_reg(p->ex_mem.valid, &p->ex_mem.ctrl, p->ex_mem.instr.rd, r) ||
           writes_reg(p->mem_wb.valid, &p->mem_wb.ctrl, p->mem_wb.instr.rd, r);
}

static int hazard_detected(const Pipeline *p, const DecodedInstr *d, const ControlSignals *c) {
    return (c->uses_rs1 && depends_on(p, d->rs1)) ||
           (c->uses_rs2 && depends_on(p, d->rs2));
}

/* ------------------------------------------------------------------ */
/* One clock tick. Every stage reads p (old latches), writes nx.       */
/* ------------------------------------------------------------------ */
CPUStatus pipeline_cycle(Pipeline *p, CPUState *cpu, Memory *mem) {
    if (cpu->status != CPU_RUNNING) return cpu->status;

    Pipeline nx = *p;
    nx.if_id.valid = nx.id_ex.valid = nx.ex_mem.valid = nx.mem_wb.valid = 0;

    int flush = 0, stall = 0;
    uint32_t redirect_pc = 0;

    nx.cycles++;
    cpu->cycle_count = nx.cycles;

    /* ============================ WB ============================ */
    if (p->mem_wb.valid) {
        const MEMWB *w = &p->mem_wb;

        if (w->fault != PF_NONE) {
            if (w->fault == PF_ILLEGAL) {
                cpu->status = CPU_ILLEGAL_INSTRUCTION;
                cpu->fault_addr = w->pc;
                snprintf(cpu->fault_msg, sizeof(cpu->fault_msg),
                         "unknown instruction 0x%08x at pc=0x%08x (no handler registered)",
                         w->instr.raw, w->pc);
            } else if (w->fault == PF_FETCH) {
                cpu->status = CPU_MEM_FAULT;
                cpu->fault_addr = w->pc;
                snprintf(cpu->fault_msg, sizeof(cpu->fault_msg),
                         "instruction fetch out of bounds at pc=0x%08x", w->pc);
            } else { /* PF_MEM */
                cpu->status = CPU_MEM_FAULT;
                cpu->fault_addr = w->fault_addr;
                snprintf(cpu->fault_msg, sizeof(cpu->fault_msg),
                         "%s out of bounds at address 0x%08x (pc=0x%08x)",
                         w->handler ? w->handler->mnemonic : "access", w->fault_addr, w->pc);
            }
            p->cycles = nx.cycles;
            return cpu->status;          /* machine stops here */
        }

        if (w->ctrl.reg_write) cpu_write_reg(cpu, (int)w->instr.rd, w->wb_val);

        if (w->ctrl.is_system) {
            /* All older instructions have written back, so the register
             * file is exact here. Reuse the registry's ecall handler. */
            uint32_t fetch_pc = cpu->pc;
            DecodedInstr tmp = w->instr;
            cpu->pc = w->pc;
            w->handler->execute(cpu, mem, &tmp);
            cpu->pc = fetch_pc;
        }

        nx.retired++;

        /* ecall exit: the machine halts on this clock edge. Younger
         * instructions must not touch memory or count a flush. */
        if (cpu->status != CPU_RUNNING) {
            p->cycles  = nx.cycles;
            p->retired = nx.retired;
            return cpu->status;
        }
    }

    /* ============================ MEM =========================== */
    if (p->ex_mem.valid) {
        const EXMEM *e = &p->ex_mem;
        MEMWB *o = &nx.mem_wb;

        o->valid      = 1;
        o->pc         = e->pc;
        o->fault      = e->fault;
        o->fault_addr = e->fault_addr;
        o->instr      = e->instr;
        o->handler    = e->handler;
        o->ctrl       = e->ctrl;
        o->wb_val     = e->alu_result;

        if (e->fault == PF_NONE) {
            uint32_t addr = e->alu_result;
            MemStatus ms = MEM_OK;

            if (e->ctrl.mem_read) {
                uint8_t b; uint16_t h; uint32_t w;
                switch (e->instr.funct3) {
                    case 0x0: ms = memory_read_byte(mem, addr, &b); o->wb_val = (uint32_t)(int32_t)(int8_t)b;  break; /* lb  */
                    case 0x1: ms = memory_read_half(mem, addr, &h); o->wb_val = (uint32_t)(int32_t)(int16_t)h; break; /* lh  */
                    case 0x2: ms = memory_read_word(mem, addr, &w); o->wb_val = w;                             break; /* lw  */
                    case 0x4: ms = memory_read_byte(mem, addr, &b); o->wb_val = (uint32_t)b;                   break; /* lbu */
                    case 0x5: ms = memory_read_half(mem, addr, &h); o->wb_val = (uint32_t)h;                   break; /* lhu */
                }
            } else if (e->ctrl.mem_write) {
                switch (e->instr.funct3) {
                    case 0x0: ms = memory_write_byte(mem, addr, (uint8_t)e->store_val);  break; /* sb */
                    case 0x1: ms = memory_write_half(mem, addr, (uint16_t)e->store_val); break; /* sh */
                    case 0x2: ms = memory_write_word(mem, addr, e->store_val);           break; /* sw */
                }
            }

            if (ms != MEM_OK) {
                o->fault = PF_MEM;
                o->fault_addr = addr;
            }
        }
    }

    /* ============================ EX ============================ */
    if (p->id_ex.valid) {
        const IDEX *d = &p->id_ex;
        EXMEM *o = &nx.ex_mem;

        o->valid      = 1;
        o->pc         = d->pc;
        o->fault      = d->fault;
        o->fault_addr = 0;
        o->instr      = d->instr;
        o->handler    = d->handler;
        o->ctrl       = d->ctrl;
        o->store_val  = d->rs2_val;
        o->alu_result = 0;

        if (d->fault == PF_NONE) {
            uint32_t a = d->rs1_val, b = d->rs2_val;
            uint32_t imm = (uint32_t)d->instr.imm;

            if (d->ctrl.is_branch) {
                int taken = 0;
                switch (d->instr.funct3) {
                    case 0x0: taken = (a == b); break;                         /* beq  */
                    case 0x1: taken = (a != b); break;                         /* bne  */
                    case 0x4: taken = ((int32_t)a <  (int32_t)b); break;       /* blt  */
                    case 0x5: taken = ((int32_t)a >= (int32_t)b); break;       /* bge  */
                    case 0x6: taken = (a <  b); break;                         /* bltu */
                    case 0x7: taken = (a >= b); break;                         /* bgeu */
                }
                if (taken) { flush = 1; redirect_pc = d->pc + imm; }
            } else if (d->ctrl.is_jump) {
                o->alu_result = d->pc + 4;                                     /* link value -> WB_PC4 */
                flush = 1;
                redirect_pc = (d->instr.opcode == OP_JALR) ? ((a + imm) & ~1u)
                                                           : d->pc + imm;      /* jal */
            } else {
                o->alu_result = alu(&d->instr, d->pc, a, b);
            }
        }
    }

    /* ============================ ID ============================ */
    if (p->if_id.valid && !flush) {   /* a taken branch/jump in EX kills this instruction */
        const IFID *f = &p->if_id;
        IDEX *o = &nx.id_ex;

        memset(o, 0, sizeof(*o));
        o->pc = f->pc;
        o->instr.raw = f->raw;

        if (f->fault != PF_NONE) {
            o->valid = 1;
            o->fault = f->fault;                         /* PF_FETCH carried down */
        } else {
            DecodedInstr d;
            const InstructionHandler *h = NULL;

            if (decode_instruction(f->raw, &d, &h) != DECODE_OK) {
                o->valid = 1;
                o->fault = PF_ILLEGAL;                   /* do not touch h */
            } else {
                ControlSignals c = control_decode(&d, h);

                if (hazard_detected(p, &d, &c)) {
                    stall = 1;                           /* nx.id_ex stays a bubble */
                } else {
                    o->valid   = 1;
                    o->fault   = PF_NONE;
                    o->instr   = d;
                    o->handler = h;
                    o->ctrl    = c;
                    o->rs1_val = cpu_read_reg(cpu, (int)d.rs1);
                    o->rs2_val = cpu_read_reg(cpu, (int)d.rs2);
                }
            }
        }
    }

    /* ============================ IF ============================ */
    if (stall) {
        nx.if_id = p->if_id;        /* freeze: same instruction retries ID next cycle */
        nx.stalls++;                /* cpu->pc not advanced */
    } else {
        uint32_t fetch_pc = flush ? redirect_pc : cpu->pc;
        IFID *o = &nx.if_id;
        o->valid = 1;
        o->pc    = fetch_pc;
        o->raw   = 0;
        o->fault = PF_NONE;
        if (memory_read_word(mem, fetch_pc, &o->raw) != MEM_OK) o->fault = PF_FETCH;
        cpu->pc = fetch_pc + 4;
    }
    if (flush) nx.flushes++;

#ifdef PIPE_TRACE
    fprintf(stderr, "c%-4llu IF:%08x ID:%c EX:%c MEM:%c WB:%c %s%s\n",
            (unsigned long long)nx.cycles, nx.if_id.pc,
            nx.id_ex.valid ? '*' : '.', nx.ex_mem.valid ? '*' : '.',
            nx.mem_wb.valid ? '*' : '.', p->mem_wb.valid ? '*' : '.',
            stall ? "STALL " : "", flush ? "FLUSH" : "");
#endif

    *p = nx;                        /* clock edge */
    return cpu->status;
}

void pipeline_run_batch(Pipeline *p, CPUState *cpu, Memory *mem, uint64_t max_cycles) {
    while (cpu->status == CPU_RUNNING && p->cycles < max_cycles) {
        pipeline_cycle(p, cpu, mem);
    }
    printf("\n");
    print_registers(cpu);
    print_status(cpu);
    if (cpu->status == CPU_RUNNING) {
        printf("  -> stopped: reached max cycle limit (%llu)\n", (unsigned long long)max_cycles);
    }
    double cpi = p->retired ? (double)p->cycles / (double)p->retired : 0.0;
    printf("pipeline: cycles=%llu retired=%llu stalls=%llu flushes=%llu CPI=%.3f\n",
           (unsigned long long)p->cycles, (unsigned long long)p->retired,
           (unsigned long long)p->stalls, (unsigned long long)p->flushes, cpi);
}
