#include <coherence.h>
#include <trace.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include "coher_internal.h"

#include "stree.h"

typedef void (*cacheCallbackFunc)(int, int, int64_t);

tree_t** coherStates = NULL;
int processorCount = 1;
bool verbose = true;
coherence_scheme cs = MI;
coher* self = NULL;
interconn* inter_sim = NULL;
cacheCallbackFunc cacheCallback = NULL;

uint8_t busReq(bus_req_type reqType, uint64_t addr, int processorNum);
uint8_t permReq(uint8_t is_read, uint64_t addr, int processorNum);
uint8_t invlReq(uint64_t addr, int processorNum);
void registerCacheInterface(void (*callback)(int, int, int64_t));

void printv(const char *format, ...) { // wrapper for printf, only prints when verbose is set to true
    va_list args;
    va_start(args, format);
    if (verbose) vprintf(format, args);
}

coher* init(coher_sim_args* csa)
{
    int op;

    while ((op = getopt(csa->arg_count, csa->arg_list, "s:")) != -1)
    {
        switch (op)
        {
            case 's':
                cs = atoi(optarg);
                break;
        }
    }

    if (processorCount < 1 || processorCount > 256)
    {
        fprintf(stderr,
                "Error: processorCount outside valid range - %d specified\n",
                processorCount);
        return NULL;
    }

    // for each processor, maintain a tree map which maps address -> state for fast lookup
    coherStates = malloc(sizeof(tree_t*) * processorCount);
    for (int i = 0; i < processorCount; i++)
    {
        coherStates[i] = tree_new();
    }

    inter_sim = csa->inter;

    self = malloc(sizeof(coher));
    self->si.tick = tick;
    self->si.finish = finish;
    self->si.destroy = destroy;
    self->permReq = permReq;
    self->busReq = busReq;
    self->invlReq = invlReq;
    self->registerCacheInterface = registerCacheInterface;

    inter_sim->registerCoher(self);

    return self;
}

void registerCacheInterface(void (*callback)(int, int, int64_t))
{
    cacheCallback = callback;
}

coherence_states getState(uint64_t addr, int processorNum) // given a memory address, looks up what state it's in
{
    coherence_states lookState
        = (coherence_states)tree_find(coherStates[processorNum], addr);
    if (lookState == UNDEF)
        return INVALID;

    return lookState;
}

void setState(uint64_t addr, int processorNum, coherence_states nextState) // given a memory address, set its state
{
    tree_insert(coherStates[processorNum], addr, (void*)nextState);
}

uint8_t busReq(bus_req_type reqType, uint64_t addr, int processorNum) // basically encapsulates receiving BusRd, BusWr, or another type of bus request
{
    printv("In mode %d; bus request with type %d, address %lx, processor %d\n", cs, reqType, addr, processorNum);
    if (processorNum < 0 || processorNum >= processorCount)
    {
        // ERROR
    }

    coherence_states currentState = getState(addr, processorNum);
    coherence_states nextState;
    cache_action ca;

    switch (cs) // THIS SWITCH STATEMENT IS TRIGGERED WHEN THERE IS A BUS REQUEST
    {
        case MI:
            nextState
                = snoopMI(reqType, &ca, currentState, addr, processorNum); // only moves M to I on BusWr
            break;
        case MSI:
            nextState
                = snoopMSI(reqType, &ca, currentState, addr, processorNum);
            break;
        case MESI:
            nextState
                = snoopMESI(reqType, &ca, currentState, addr, processorNum);
            break;
        case MOESI:
            // TODO: Implement this.
            break;
        case MESIF:
            // TODO: Implement this.
            break;
        default:
            fprintf(stderr, "Undefined coherence scheme - %d\n", cs);
            break;
    }

    switch (ca)
    {
        case DATA_RECV:
        case INVALIDATE:
        case NO_ACTION:
            cacheCallback(ca, processorNum, addr);
            break;

        default:
            assert(0);
    }

    // If the destination state is invalid, that is an implicit
    // state and does not need to be stored in the tree.
    if (nextState == INVALID)
    {
        if (currentState != INVALID)
        {
            tree_remove(coherStates[processorNum], addr);
        }
    }
    else
    {
        setState(addr, processorNum, nextState);
    }

    return 0;
}

uint8_t permReq(uint8_t is_read, uint64_t addr, int processorNum) // basically encapsulates PrRd and PrWr
{
    printv("In mode %d; perm request with type %d, address %lx, processor %d\n", cs, is_read, addr, processorNum);
    if (processorNum < 0 || processorNum >= processorCount)
    {
        // ERROR
    }

    coherence_states currentState = getState(addr, processorNum);
    coherence_states nextState;
    uint8_t permAvail = 0; // return value bool: whether permissions were granted or not

    switch (cs)
    {
        case MI:
            nextState = cacheMI(is_read, &permAvail, currentState, addr,
                                processorNum);
            break;

        case MSI:
            nextState = cacheMSI(is_read, &permAvail, currentState, addr, 
                                processorNum);
            break;

        case MESI:
            nextState = cacheMESI(is_read, &permAvail, currentState, addr, 
                                processorNum);
            break;

        case MOESI:
            // TODO: Implement this.
            break;

        case MESIF:
            // TODO: Implement this.
            break;

        default:
            fprintf(stderr, "Undefined coherence scheme - %d\n", cs);
            break;
    }

    setState(addr, processorNum, nextState);
    return permAvail;
}

uint8_t invlReq(uint64_t addr, int processorNum) // basically handles cache line invalidation
{
    printv("In mode %d; invalidation request with address %lx, processor %d\n", cs, addr, processorNum);
    coherence_states currentState, nextState = INVALID;
    cache_action ca;
    uint8_t flush;

    if (processorNum < 0 || processorNum >= processorCount)
    {
        // ERROR
    }

    currentState = getState(addr, processorNum);

    flush = 0; // returns whether or not data was actually flushed or not
    switch (cs)
    {
        case MI:
            nextState = INVALID;
            if (currentState != INVALID)
            {
                inter_sim->busReq(DATA, addr, processorNum); // DATA bus req type denotes that a processor flushed this cache entry
                flush = 1;
            }
            break;

        case MSI:
            nextState = INVALID;
            if (currentState == MODIFIED) {
                inter_sim->busReq(DATA, addr, processorNum);
                flush = 1; // not sure about this logic yet
            } break;
        case MESI:
            nextState = INVALID;
            if (currentState == MODIFIED) {
                inter_sim->busReq(DATA, addr, processorNum);
                flush = 1; // not sure about this logic yet
            } break;
            break;

        case MOESI:
            // TODO: Implement this.
            break;

        case MESIF:
            // TODO: Implement this.
            break;

        default:
            fprintf(stderr, "Undefined coherence scheme - %d\n", cs);
            break;
    }

    tree_remove(coherStates[processorNum], addr);

    // Notify about "permReqOnFlush".
    return flush;
}

int tick()
{
    return inter_sim->si.tick();
}

int finish(int outFd)
{
    return inter_sim->si.finish(outFd);
}

int destroy(void)
{
    // TODO

    return inter_sim->si.destroy();
}
