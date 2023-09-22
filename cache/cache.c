#include "csim.h"
#include "trace.h"
#include <stdio.h>
#include <stdbool.h>
#include <getopt.h>
#include <stdlib.h>
#include <assert.h>

typedef struct _pendingRequest {
    int64_t tag;
    int8_t procNum;
    void (*memCallback)(int, int64_t);
} pendingRequest;

csim* self = NULL; // csim is the cache object, see csim.h for definition
int processorCount = 1;
int CADSS_VERBOSE = 0;
pendingRequest pending = {0};
int countDown = 0;
int E = -1; // associativity
int s = -1; // # of set bits, and S = 2^s gives number of sets
int b = -1; // # of block bits, and B = 2^b gives number of block bytes
int v = -1; // # of lines in victim cache
int k = -1; // # of bits in the RRPV, max value of RRPV is 2^k - 1

void memoryRequest(trace_op* op, int processorNum, int64_t tag,
                   void (*callback)(int, int64_t));

int pow2(int n) {
    if (n == 0)
        return 1;
    else
        return 2 * pow2(n - 1);
}

csim* init(cache_sim_args* csa)
{
    extern char *optarg;
    int op;

    // process arguments to fill in the parameters of the cache
    while ((op = getopt(csa->arg_count, csa->arg_list, "E:s:b:i:R:")) != -1)
    {
        switch (op)
        {
            case 'E':
                E = atoi(optarg);
                break;
            case 's':
                s = atoi(optarg);
                break;
            case 'b':
                b = atoi(optarg);
                break;
            case 'i':
                v = atoi(optarg);
                break;
            case 'R':
                k = atoi(optarg);
                break;
        }
    } printf("Input parameters: E = %d, s = %d, b = %d, v = %d, k = %d\n", E, s, b, v, k);

    // initialize the cache here
    self = malloc(sizeof(csim));
    self->memoryRequest = memoryRequest;
    self->si.tick = tick;
    self->si.finish = finish;
    self->si.destroy = destroy;
    int S = pow2(s);
    int B = pow2(b);
    line **cache = calloc(sizeof(line*), S);
    cache = calloc(sizeof(line*), S);
    for (int i = 0; i < S; i++) {
        cache[i] = calloc(sizeof(line), E);
        for (int j = 0; j < E; j++) {
            cache[i][j].vbit = 0;
            cache[i][j].dbit = 0;     // disregarded if vbit == 0
            cache[i][j].lastused = 0; // disregarded if vbit == 0
            cache[i][j].tag = 0;      // disregarded if vbit == 0
        }
    } self->cache = cache;
    printf("Initialized cache of %d x %d x %d\n\n", S, E, B);
    return self;
}

void handleHit(int set, int index, bool store) {
    line *l = &self->cache[set][index]; 
    l->lastused = 0;
    if (store && l->dbit == 0) l->dbit = 1;
    printf("Got cache hit in set %d at index %d\n", set, index);
    countDown = 1;
    // TODO: do additional check for hit in victim cache
}

void handleColdMiss(unsigned long tag, int set, int index, bool store) {
    line *l = &self->cache[set][index];
    l->lastused = 0;
    l->tag = tag;
    l->vbit = 1;
    if (store) l->dbit = 1;
    printf("Got cold cache miss, loaded into set %d at index %d\n", set, index);
    countDown = 100;
}

void handleConflictMiss(unsigned long tag, int set, int index, bool store) {
    line *l = &self->cache[set][index];
    l->lastused = 0;
    l->tag = tag;
    if (l->dbit == 1) {
        l->dbit = 0;
        countDown = 150;
    } else countDown = 100;
    if (store) l->dbit = 1;
    printf("Got conflict cache miss, evicted entry in set %d at index %d\n", set, index);
    // TODO: move evicted entry to victim cache
}

void memoryRequest(trace_op* op, int processorNum, int64_t tag,
                   void (*callback)(int, int64_t))
{
    printf("Received %s instruction for address 0x%lx\n", op->op == 1 ? "load" : "store", op->memAddress);
    assert(op != NULL);
    assert(callback != NULL);

    // Simple model to only have one outstanding memory operation
    if (countDown != 0)
    {
        assert(pending.memCallback != NULL);
        pending.memCallback(pending.procNum, pending.tag);
    }

    pending = (pendingRequest){
        .tag = tag, .procNum = processorNum, .memCallback = callback};

    // handle the cache operation here to determine countdown based on hit or miss
    unsigned long addr = op->memAddress;
    bool store = op->op == MEM_STORE;
    addr >>= b;
    unsigned long cacheTag = addr >> s;
    unsigned long set = s == 0 ? 0 : addr << (64 - s) >> (64 - s);
    printf("Operation type: %s, tag: 0x%lx, set: %lu, ", store ? "store" : "load", cacheTag, set);
    int emptyIndex = -1; // denotes cold miss if not -1
    int matchIndex = -1; // denotes cache hit if not -1
    int LRU = 0; // if neither cache hit or cold miss, then must be conflict miss
    int LRUIndex = -1; // use LRUIndex in case of conflict miss
    for (int j = 0; j < E; j++) {
        line *l = &self->cache[set][j];
        if (l->vbit == 0 && emptyIndex == -1) emptyIndex = j;
        else if (l->vbit == 1) {
            if (l->tag == cacheTag && matchIndex == -1) matchIndex = j;
            if (++l->lastused > LRU) {
                LRU = l->lastused;
                LRUIndex = j;
            }
        }
    } printf("match index: %d, empty index: %d, LRU value: %d, LRU index: %d\n", matchIndex, emptyIndex, LRU, LRUIndex);
    if (matchIndex != -1) handleHit(set, matchIndex, store);
    else if (emptyIndex != -1) handleColdMiss(cacheTag, set, emptyIndex, store);
    else handleConflictMiss(cacheTag, set, LRUIndex, store);
    printf("Setting countdown to %d\n\n", countDown);
}

int tick()
{
    if (countDown > 0)
    {
        countDown--;
        if (countDown == 0)
        {
            assert(pending.memCallback != NULL);
            pending.memCallback(pending.procNum, pending.tag);
        }
    }

    return 1;
}

int finish(int outFd)
{
    return 0;
}

int destroy(void)
{
    // free any internally allocated memory here
    return 0;
}
