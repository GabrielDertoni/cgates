#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>

typedef int PortId;
typedef int GateId;

typedef char Bit;
typedef uint8_t u8;
typedef uint64_t u64;

typedef struct {
    Bit value;
} Port;

// typedef struct {
//     Bit value0 : 1;
//     Bit value2 : 1;
//     Bit value3 : 1;
//     Bit value4 : 1;
//     Bit value5 : 1;
//     Bit value6 : 1;
//     Bit value7 : 1;
// } PortX8;

typedef unsigned char PortX8;

typedef enum {
    GateKind_In,
    GateKind_Out,

    GateKind_Wire,
    GateKind_Not,

    GateKind_And,
    GateKind_Or,
    GateKind_Nor,
} GateKind;

static const int GateKindIns_LUT[] = {
    [GateKind_In]           = 0,
    [GateKind_Out]          = 1,

    [GateKind_Wire]         = 1,
    [GateKind_Not ]         = 1,

    [GateKind_And ]         = 2,
    [GateKind_Or  ]         = 2,
    [GateKind_Nor ]         = 2,
};

static const int GateKindsOuts_LUT[] = {
    [GateKind_In]           = 1,
    [GateKind_Out]          = 0,

    [GateKind_Wire]         = 1,
    [GateKind_Not ]         = 1,

    [GateKind_And ]         = 1,
    [GateKind_Or  ]         = 1,
    [GateKind_Nor ]         = 1,
};

typedef struct {
    int n_inputs;
    int n_outputs;
    // int inputs_offset;
    // int outputs_offset;
} GateBundleData;

typedef struct {
    GateKind kind;
    union {
        PortId inputs[2];
        PortId input;
    };
    union {
        PortId outputs[2];
        PortId output;
    };
    // Specific to gates
    union {
        // GateKind_In
        Bit value;

        // GateKind_Out
        struct {
            void (*cb)(Bit, void*);
            void* cb_data;
        };
    };
} Gate;

typedef struct {
    u64* bits;
    // Determines the range of blocks that may contain 1s
    // In particular, it mantains the invariants `bits[block_len-1] != 0` and
    // `bits[block_len..block_cap] == 0`.
    int block_len;
    int block_cap;
} GateSet;

typedef struct {
    GateId curr;

    int block_idx;
    u64 block;
    const GateSet* set;
} GateSetIter;

#define FOR_EACH_GATE(set_, it) \
    for (gate_set_iter_init(&it, set_); it.block_idx < it.set->block_cap; gate_set_iter_step(&it))

typedef struct {
    PortId* inout_ids;
    int len_inout_ids;
    int cap_inout_ids;

    // port -> input relationship graph stored as an adjacency list
    GateSet* subs;

    PortX8* ports;
    int len_ports;
    int cap_ports;

    Gate* gates;
    int len_gates;
    int cap_gates;

    // Whitch gates are "dirty" (need to be updated)
    GateSet dirty;
} Circuit;

void gate_set_iter_init(GateSetIter* it, const GateSet* set) {
    it->set = set;
    it->block_idx = 0;
    if (it->block_idx < it->set->block_cap)
        it->block = it->set->bits[0];
}

void gate_set_iter_step(GateSetIter* it) {
    if (it->block_idx >= it->set->block_cap) return;
    int idx = __builtin_clz(it->block);
    it->curr = it->block_idx * 64 + idx;
    // Clear msb
    it->block &= ~(1 << (64 - idx - 1));
    while (it->block == 0) {
        it->block_idx++;
        if (it->block_idx >= it->set->block_cap) return;
        it->block = it->set->bits[it->block_idx];
    }
}

void circuit_gate_mark_dirty(Circuit* circuit, GateId gate);

Circuit* circuit_new() {
    Circuit* circuit = (Circuit*)calloc(1, sizeof(Circuit));
    return circuit;
}

void circuit_delete(Circuit* circuit) {
    if (!circuit) return;
    if (circuit->inout_ids) free(circuit->inout_ids);
    if (circuit->ports) free(circuit->ports);
    if (circuit->gates) free(circuit->gates);
    free(circuit);
}

PortId circuit_port_new(Circuit* circuit) {
    if (circuit->len_ports >= circuit->cap_ports) {
        circuit->cap_ports = circuit->cap_ports == 0 ? 8 : circuit->cap_ports * 2;
        int x8_cap = circuit->cap_ports >> 3;
        circuit->ports = (PortX8*)realloc(circuit->ports, x8_cap * sizeof(PortX8));
        int x8_len = circuit->len_ports >> 3;
        memset(circuit->ports + x8_len, 0, x8_cap - x8_len);

        circuit->subs = (GateSet*)realloc(circuit->subs, x8_cap * sizeof(GateSet));
        memset(circuit->subs + x8_len, 0, x8_cap - x8_len);
    }
    return circuit->len_ports++;
}

void circuit_port_notify(Circuit* circuit, PortId id) {
    GateSetIter it;
    FOR_EACH_GATE(&circuit->subs[id], it) {
        circuit_gate_mark_dirty(circuit, it.curr);
    }
}

Bit circuit_port_get(const Circuit* circuit, PortId id) {
    assert(id < circuit->len_ports);
    int hi = id >> 3;
    int lo = id & 0b111;
    return (circuit->ports[hi] >> lo) & 1;
}

void circuit_port_set(Circuit* circuit, PortId id, Bit value) {
    assert(id < circuit->len_ports);
    int hi = id >> 3;
    int lo = id & 0b111;
    circuit->ports[hi] &= ~(1 << lo); // reset
    circuit->ports[hi] |= (value & 1) << lo; // set to value
    circuit_port_notify(circuit, id);
}

void circuit_update_gate(Circuit* circuit, GateId gate) {
    assert(circuit && gate < circuit->len_gates);

    Gate g = circuit->gates[gate];
    switch (g.kind) {
        case GateKind_Out: {
            Bit value = circuit_port_get(circuit, g.input);
            g.cb(value, g.cb_data);
        } break;

        case GateKind_In: {
            circuit_port_set(circuit, g.output, g.value);
        } break;

        case GateKind_Wire: {
            Bit value = circuit_port_get(circuit, g.input);
            circuit_port_set(circuit, g.output, value);
        } break;

        case GateKind_Not: {
            Bit value = circuit_port_get(circuit, g.input);
            circuit_port_set(circuit, g.output, ~value);
        } break;

        case GateKind_And: {
            Bit left = circuit_port_get(circuit, g.inputs[0]);
            Bit right = circuit_port_get(circuit, g.inputs[1]);
            circuit_port_set(circuit, g.output, left && right);
        } break;

        case GateKind_Or: {
            Bit left = circuit_port_get(circuit, g.inputs[0]);
            Bit right = circuit_port_get(circuit, g.inputs[1]);
            circuit_port_set(circuit, g.output, left || right);
        } break;

        case GateKind_Nor: {
            Bit left = circuit_port_get(circuit, g.inputs[0]);
            Bit right = circuit_port_get(circuit, g.inputs[1]);
            circuit_port_set(circuit, g.output, !(left || right));
        } break;

        default: assert(0);
    }
}

GateId circuit_gate_new(Circuit* circuit, GateKind kind) {
    if (circuit->len_gates >= circuit->cap_gates) {
        circuit->cap_gates = circuit->cap_gates == 0 ? 8 : circuit->cap_gates * 2;
        circuit->gates = (Gate*)realloc(circuit->gates, circuit->cap_gates * sizeof(Gate));
    }
    GateId id = circuit->len_gates++;
    Gate* gate = &circuit->gates[id];
    gate->kind = kind;
    return id;
}

void gate_set_mark(GateSet* set, GateId id) {
    int block_idx = id >> 6;
    if (block_idx >= set->block_cap) {
        int oldcap = set->block_cap;
        set->block_cap = set->block_cap == 0 ? 8 : set->block_cap * 2;
        if (set->block_cap <= block_idx)
            set->block_cap = block_idx + 1;

        set->bits = (u64*)realloc(set->bits, set->block_cap * sizeof(u64));
        memset(set->bits, 0, set->block_cap - oldcap);
    }
    set->bits[block_idx] |= 1 << (id & 0x3f);
    if (block_idx >= set->block_len)
        set->block_len = block_idx + 1;
}

void gate_set_unmark(GateSet* set, GateId id) {
    int block_idx = id >> 6;
    if (block_idx >= set->block_len) return;
    set->bits[block_idx] &= ~(1 << id & 0x3f);

    if (set->block_len == block_idx + 1) {
        while (set->bits[set->block_len] == 0) {
            set->block_len--;
        }
    }
}

bool gate_set_empty(const GateSet* set) {
    return set->block_len <= 0;
}

GateId gate_set_pop(GateSet* set) {
    assert(!gate_set_empty(set));
    u64* block = &set->bits[set->block_len - 1];
    int idx = __builtin_ctz(*block);
    GateId id = set->block_len * 64 + (64 - idx - 1);
    *block &= ~(1 << idx);
    return id;
}

void circuit_gate_subscribe(Circuit* circuit, GateId gate, PortId port) {
    gate_set_mark(&circuit->subs[port], gate);
}

void circuit_gate_mark_dirty(Circuit* circuit, GateId gate) {
    gate_set_mark(&circuit->dirty, gate);
}

void circuit_update(Circuit* circuit) {
    while (!gate_set_empty(&circuit->dirty)) {
        GateId id = gate_set_pop(&circuit->dirty);
        circuit_update_gate(circuit, id);
    }
}

void on_output(Bit value, void* capture) {
    const char* name = (const char*)capture;
    printf("(%s) value = %d\n", name, value);
}

int main() {
    Circuit* c = circuit_new();

    PortId p1 = circuit_port_new(c);
    PortId p2 = circuit_port_new(c);
    PortId p3 = circuit_port_new(c);
    PortId p4 = circuit_port_new(c);
    PortId p5 = circuit_port_new(c);

    GateId in1 = circuit_gate_new(c, GateKind_In);
    c->gates[in1].output = p1;

    GateId in2 = circuit_gate_new(c, GateKind_In);
    c->gates[in2].output = p2;

    GateId r = circuit_gate_new(c, GateKind_Out);
    c->gates[r].input = p3;
    circuit_gate_subscribe(c, r, p3);
    c->gates[r].cb = on_output;
    c->gates[r].cb_data = (void*)"reset";

    GateId s = circuit_gate_new(c, GateKind_Out);
    c->gates[s].input = p4;
    circuit_gate_subscribe(c, s, p4);
    c->gates[s].cb = on_output;
    c->gates[s].cb_data = (void*)"set";

    GateId a = circuit_gate_new(c, GateKind_Nor);
    c->gates[a].inputs[0] = p1;
    circuit_gate_subscribe(c, a, p1);
    c->gates[a].inputs[1] = p4;
    circuit_gate_subscribe(c, a, p4);
    c->gates[a].output = p3;

    GateId b = circuit_gate_new(c, GateKind_Nor);
    c->gates[a].inputs[0] = p3;
    circuit_gate_subscribe(c, a, p3);
    c->gates[a].inputs[1] = p2;
    circuit_gate_subscribe(c, a, p2);
    c->gates[a].output = p4;

    c->gates[in1].value = 0;
    circuit_gate_mark_dirty(c, in1);
    c->gates[in2].value = 0;
    circuit_gate_mark_dirty(c, in2);

    circuit_update(c);

    circuit_delete(c);

    return 0;
}
