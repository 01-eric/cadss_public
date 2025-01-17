#include "csim.h"
#include "trace.h"
#include <stdio.h>
#include <stdbool.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>

typedef struct _pendingRequest {
    int64_t tag;
    int8_t procNum;
    void (*memCallback)(int, int64_t);
} pendingRequest;

csim *self = NULL; // csim is the cache object, see csim.h for definition
line *vcache = NULL; // victim cache will be a 1D array of line structs
int processorCount = 1;
bool verbose = false; // set this to true if you want print logs
pendingRequest pending = {0};
int countDown = 0;
int E = -1; // associativity
int s = -1; // # of set bits, and S = 2^s gives number of sets
int b = -1; // # of block bits, and B = 2^b gives number of block bytes
int v = -1; // # of lines in victim cache
int k = -1; // # of bits in the RRPV, max value of RRPV is 2^k - 1

void printv(const char *format, ...) { // wrapper for printf, only prints when verbose is set to true
    va_list args;
    va_start(args, format);
    if (verbose) vprintf(format, args);
}

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
    } printv("Input parameters: E = %d, s = %d, b = %d, v = %d, k = %d\n", E, s, b, v, k);

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
    for (int i = 0; i < S; i++) cache[i] = calloc(sizeof(line), E);
    self->cache = cache;
    printv("Initialized cache of %d x %d x %d\n", S, E, B);
    if (v > 0) { // initialize the victim cache if applicable
        vcache = calloc(sizeof(line), v); 
        printv("Initialized victim cache of size 1 x %d x %d\n", v, B);
    } printv("\n");

    return self;
}

void handleHit(int set, int index, bool store) {
    line *l = &self->cache[set][index]; 
    l->evict = 0; // set to 0 in both LRU and RRPV case
    if (store && l->dbit == 0) l->dbit = 1;
    printv("Got cache hit in set %d at index %d\n", set, index);
}

void handleColdMiss(unsigned long tag, int set, int index, bool store, line *vcacheHit) {
    line *l = &self->cache[set][index];
    l->tag = tag;
    l->vbit = 1;
    if (vcacheHit) {
        l->evict = 0;
        l->dbit = vcacheHit->dbit;
        free(vcacheHit);
        printv("Got victim cache hit, loading into main cache set %d at index %d\n", set, index);
    } else {
        if (k != -1) l->evict = pow2(k) - 1; // RRPV case  
        else l->evict = 0; // LRU case
        countDown = 100;
        printv("Got cold cache miss, loaded into set %d at index %d\n", set, index);
    } if (store) l->dbit = 1;
}

void handleConflictMiss(unsigned long tag, int set, int index, bool store, line *vcacheHit) {
    line *l = &self->cache[set][index];

    // if victim cache exists, add to it first
    if (vcache) {
        int vEmptyIndex = -1; 
        int vEvictVal = -1; 
        int vEvictIndex = -1;
        line *vl;
        for (int i = 0; i < v; i++) {
            vl = &vcache[i];
            if (vl->vbit == 0 && vEmptyIndex == -1) vEmptyIndex = i;
            else if (vl->vbit == 1 && ++vl->evict > vEvictVal) {
                vEvictVal = vl->evict;
                vEvictIndex = i;
            }
        } if (vEmptyIndex != -1) vl = &vcache[vEmptyIndex];
        else {
            vl = &vcache[vEvictIndex];
            countDown = vl->dbit == 1 ? 150 : 100;
            printv("Evicting from victim cache...\n");
        } vl->vbit = 1;
        vl->dbit = l->dbit;
        vl->evict = 0;
        vl->tag = l->tag;
    }
        
    l->tag = tag;
    if (vcacheHit) {
        l->evict = 0;
        l->dbit = vcacheHit->dbit;
        free(vcacheHit);
        printv("Got victim cache hit, loading into main cache set %d at index %d\n", set, index);
    } else {
        if (k != -1) l->evict = pow2(k) - 1; // RRPV case  
        else l->evict = 0; // LRU case
        if (l->dbit == 1) {
            l->dbit = 0;
            if (!vcache) countDown = 150;
        } else if (!vcache) countDown = 100;
        printv("Got conflict cache miss, evicted entry in set %d at index %d\n", set, index);
    } if (store) l->dbit = 1;
}

void memoryRequest(trace_op* op, int processorNum, int64_t tag,
                   void (*callback)(int, int64_t))
{
    printv("Received %s instruction for address 0x%lx\n", op->op == 1 ? "load" : "store", op->memAddress);
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
    printv("Operation type: %s, tag: 0x%lx, set: %lu, ", store ? "store" : "load", cacheTag, set);
    int emptyIndex = -1; // denotes cold miss if not -1
    int matchIndex = -1; // denotes cache hit if not -1
    int evictVal = -1; // if neither cache hit or cold miss, then must be conflict miss
    int evictIndex = -1; // use LRUIndex in case of conflict miss

    // first pass through the main cache: checks to see if hit, cold miss, or conflict miss
    for (int j = 0; j < E; j++) {
        line *l = &self->cache[set][j];
        if (l->vbit == 0 && emptyIndex == -1) emptyIndex = j;
        else if (l->vbit == 1) {
            if (l->tag == cacheTag && matchIndex == -1) matchIndex = j;
            if (l->evict > evictVal) {
                evictVal = l->evict;
                evictIndex = j;
            }
        }
    } 
    
    // if first pass didn't result in a hit and victim cache is enabled: check victim cache
    // if victim cache finds a hit: remove the entry from the victim cache but copy its contents.
    // then have the victim cache entry be added via handleColdMiss or handleConflictMiss
    line *vcacheHit = NULL;
    if (vcache && matchIndex == -1) {
        int vMatchIndex = -1;
        for (int i = 0; i < v; i++) {
            line *l = &vcache[i];
            if (l->vbit == 1 && l->tag == cacheTag && vMatchIndex == -1) vMatchIndex = i;
        } if (vMatchIndex != -1) {
            line *l = &vcache[vMatchIndex];
            vcacheHit = malloc(sizeof(line)); // create a copy of the vcache entry
            vcacheHit->vbit = 1;
            vcacheHit->dbit = l->dbit;
            vcacheHit->tag = l->tag;
            l->vbit = 0; // clear entry
            l->dbit = 0;
            l->evict = 0;
            l->tag = 0;
        }
    }
    
    for (int j = 0; j < E; j++) {
        line *l = &self->cache[set][j];
        if (k == -1) ++l->evict; // LRU case: increment all LRU counters by 1
        else if (matchIndex == -1 && emptyIndex == -1) 
            l->evict += pow2(k) - 1 - evictVal; // RRPV case: increment all RRPVs until at least one reaches 2^k - 1
    }
    
    printv("match index: %d, empty index: %d, LRU/RRPV value: %d, evict index: %d\n", matchIndex, emptyIndex, evictVal, evictIndex);
    if (matchIndex != -1) handleHit(set, matchIndex, store);
    else if (emptyIndex != -1) handleColdMiss(cacheTag, set, emptyIndex, store, vcacheHit);
    else handleConflictMiss(cacheTag, set, evictIndex, store, vcacheHit);
    if (countDown == 0) countDown = 1;
    printv("Setting countdown to %d\n\n", countDown);
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
