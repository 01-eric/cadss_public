#ifndef CSIM_H
#define CSIM_H

#include "trace.h"
#include "coherence.h"

typedef struct _cache_sim_args {
    int arg_count;
    char** arg_list;
    coher* coherComp;
} cache_sim_args;

// representation of a single cache line
typedef struct {
    short vbit;
    short dbit;
    int evict; // used as either LRU counter or RRPV, depending on the mode
    unsigned long tag;
} line;

typedef struct _csim {
    sim_interface si;
    void (*memoryRequest)(trace_op*, int, int64_t, void(*callback)(int, int64_t));
    line **cache;
} csim;

#endif
