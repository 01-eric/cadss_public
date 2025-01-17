#ifndef COHER_INTERNAL_H
#define COHER_INTERNAL_H

#include <interconnect.h>
#include <stdio.h>

extern interconn* inter_sim;

typedef enum _coherence_states
{
    UNDEF = 0, // As tree find returns NULL, we need an unused for NULL
    MODIFIED,
    INVALID,
    INVALID_MODIFIED,
    SHARING,
    INVALID_SHARING, // serves dual purpose as intermediate between I -> E
    SHARING_MODIFIED,
    EXCLUSIVE_CLEAN,
    OWNED, // serves dual purpose as F state in MESIF (don't wanna create another state)
    OWNED_MODIFIED
} coherence_states;

typedef enum _coherence_scheme
{
    MI,
    MSI,
    MESI,
    MOESI,
    MESIF
} coherence_scheme;

coherence_states
cacheMI(uint8_t is_read, uint8_t* permAvail, coherence_states currentState,
        uint64_t addr, int procNum);
coherence_states
snoopMI(bus_req_type reqType, cache_action* ca, coherence_states currentState,
        uint64_t addr, int procNum);
coherence_states
cacheMSI(uint8_t is_read, uint8_t* permAvail, coherence_states currentState,
        uint64_t addr, int procNum);
coherence_states
snoopMSI(bus_req_type reqType, cache_action* ca, coherence_states currentState,
        uint64_t addr, int procNum);
coherence_states
cacheMESI(uint8_t is_read, uint8_t* permAvail, coherence_states currentState,
        uint64_t addr, int procNum);
coherence_states
snoopMESI(bus_req_type reqType, cache_action* ca, coherence_states currentState,
        uint64_t addr, int procNum);
coherence_states
cacheMOESI(uint8_t is_read, uint8_t* permAvail, coherence_states currentState,
        uint64_t addr, int procNum);
coherence_states
snoopMOESI(bus_req_type reqType, cache_action* ca, coherence_states currentState,
        uint64_t addr, int procNum);
coherence_states
cacheMESIF(uint8_t is_read, uint8_t* permAvail, coherence_states currentState,
        uint64_t addr, int procNum);
coherence_states
snoopMESIF(bus_req_type reqType, cache_action* ca, coherence_states currentState,
        uint64_t addr, int procNum);

#endif
