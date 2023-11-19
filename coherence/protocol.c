#include "coher_internal.h"

void sendBusRd(uint64_t addr, int procNum)
{
    inter_sim->busReq(BUSRD, addr, procNum);
}

void sendBusWr(uint64_t addr, int procNum)
{
    inter_sim->busReq(BUSWR, addr, procNum);
}

void sendData(uint64_t addr, int procNum)
{
    inter_sim->busReq(DATA, addr, procNum);
}

void indicateShared(uint64_t addr, int procNum)
{
    inter_sim->busReq(SHARED, addr, procNum);
}

coherence_states
cacheMI(uint8_t is_read, uint8_t* permAvail, coherence_states currentState,
        uint64_t addr, int procNum)
{
    switch (currentState)
    {
        case INVALID: // need to gain exclusive permission (in MI, this occurs on both PrRd and PrWr)
            *permAvail = 0; // set permission available to false
            sendBusWr(addr, procNum); // send BusWr to all other processors (since no differentiation between PrRd and PrWr)
            return INVALID_MODIFIED; // move to intermediate state waiting for permissions
        case MODIFIED:
            *permAvail = 1; // permissions already available
            return MODIFIED;
        case INVALID_MODIFIED: // intermediate state: need to wait for permissions
            fprintf(stderr, "IM state on %lx, but request %d\n", addr,
                    is_read);
            *permAvail = 0; // do nothing; snoopMI will snoop for other processors granting permission and move us to M state
            return INVALID_MODIFIED;
        default:
            fprintf(stderr, "State %d not supported, found on %lx\n",
                    currentState, addr);
            break;
    }

    return INVALID;
}

coherence_states
snoopMI(bus_req_type reqType, cache_action* ca, coherence_states currentState,
        uint64_t addr, int procNum)
{
    *ca = NO_ACTION;
    switch (currentState)
    {
        case INVALID:
            return INVALID;
        case MODIFIED: // note that this triggers on all req types, because M is always exclusive on both Rd and Wr
            sendData(addr, procNum); // send DATA bus request to other processors (indicating line was flushed)
            // indicateShared(addr, procNum); // Needed for E state
            *ca = INVALIDATE;
            // printf("Setting cache action to INVALIDATE\n");
            return INVALID;
        case INVALID_MODIFIED:
            if (reqType == DATA) // only need to wait for one response, since only one processor in M at a time
            {
                *ca = DATA_RECV;
                return MODIFIED;
            }

            return INVALID_MODIFIED;
        default:
            fprintf(stderr, "State %d not supported, found on %lx\n",
                    currentState, addr);
            break;
    }

    return INVALID;
}

coherence_states
cacheMSI(uint8_t is_read, uint8_t* permAvail, coherence_states currentState,
        uint64_t addr, int procNum)
{
    switch(currentState) {
        case INVALID:
            *permAvail = 0; // indicate permissions not available yet, need to go to intermediate state first
            if (is_read) {
                sendBusRd(addr, procNum);
                return INVALID_SHARING;
            } else {
                sendBusWr(addr, procNum);
                return INVALID_MODIFIED;
            }
        case MODIFIED:
            *permAvail = 1; // permissions already available
            return MODIFIED;
        case INVALID_MODIFIED:
            *permAvail = 0; // do nothing; snoopMI will move this state to modified eventually when DATA is received
            return INVALID_MODIFIED;
        case SHARING:
            if (is_read) {
                *permAvail = 1; // if we are reading, the permissions already available
                return SHARING;
            } else {
                *permAvail = 0;
                sendBusWr(addr, procNum); // same actions as moving from I -> M
                return INVALID_MODIFIED;
            }
        case INVALID_SHARING:
            *permAvail = 0; // do nothing; snoopMI will handle this case as well
            return INVALID_SHARING;
        default:
            break;
    } return INVALID;
}

coherence_states
snoopMSI(bus_req_type reqType, cache_action* ca, coherence_states currentState,
        uint64_t addr, int procNum)
{
    *ca = NO_ACTION;
    switch (currentState) {
        case INVALID:
            return INVALID;
        case MODIFIED:
            sendData(addr, procNum); // broadcast DATA busreq to let processors know data was evicted
            if (reqType == BUSRD) return SHARING; // move M -> S if BusRd, else move M -> I (how do we flush here???)
            else { // BUSWR
                *ca = INVALIDATE;
                return INVALID;
            }
        case INVALID_MODIFIED:
            if (reqType == DATA) {
                *ca = DATA_RECV;
                return MODIFIED;
            } return INVALID_MODIFIED;
        case SHARING:
            if (reqType == BUSWR) {
                *ca = INVALIDATE; // double check this is the way to use invalidate as well
                return INVALID;
            } return SHARING;
        case INVALID_SHARING:
            if (reqType == DATA) { // same logic as INVALID_MODIFIED
                *ca = DATA_RECV;
                return SHARING;
            } return INVALID_SHARING;
        default:
            break;
    } return INVALID;
}

coherence_states
cacheMESI(uint8_t is_read, uint8_t* permAvail, coherence_states currentState,
        uint64_t addr, int procNum)
{
    switch(currentState) {
        case INVALID:
            *permAvail = 0; // indicate permissions not available yet, need to go to intermediate state first
            if (is_read) {
                sendBusRd(addr, procNum); // send BusRd, wait and see if anyone asserts SHARED
                return INVALID_SHARING; // intermediate between I -> S and I -> E
            } else {
                sendBusWr(addr, procNum); 
                return INVALID_MODIFIED;
            }
        case MODIFIED:
            *permAvail = 1; // permissions already available
            return MODIFIED;
        case INVALID_MODIFIED:
            *permAvail = 0; // do nothing; snoopMI will move this state to modified eventually when DATA is received
            return INVALID_MODIFIED;
        case SHARING:
            if (is_read) {
                *permAvail = 1; // if we are reading, then permissions already available
                return SHARING;
            } else {
                *permAvail = 0;
                sendBusWr(addr, procNum); // same actions as moving from I -> M
                return INVALID_MODIFIED;
            }
        case INVALID_SHARING:
            *permAvail = 0; // do nothing; snoopMI will handle this case as well
            return INVALID_SHARING;
        case EXCLUSIVE_CLEAN:
            *permAvail = 1; // in E state, we already have RW permissions
            return is_read ? EXCLUSIVE_CLEAN : MODIFIED;
        default:
            break;
    } return INVALID;
}

coherence_states
snoopMESI(bus_req_type reqType, cache_action* ca, coherence_states currentState,
        uint64_t addr, int procNum)
{
    *ca = NO_ACTION;
    switch (currentState) {
        case INVALID:
            return INVALID;
        case MODIFIED:
            if (reqType == BUSRD) {
                indicateShared(addr, procNum); // IMPORTANT: since we are moving to S, need to indicate shared
                sendData(addr, procNum); // want DATA to be sent after SHARED bc we decide I -> S vs. I -> E based on SHARED
                return SHARING; // move M -> S if BusRd, else move M -> I (how do we flush here???)
            } else { // BUSWR
                sendData(addr, procNum);
                *ca = INVALIDATE;
                return INVALID;
            }
        case INVALID_MODIFIED:
            if (reqType == DATA || reqType == SHARED) {
                *ca = DATA_RECV;
                return MODIFIED;
            } return INVALID_MODIFIED;
        case SHARING:
            if (reqType == BUSRD) {
                indicateShared(addr, procNum); // indicate shared if another processor attempts to read (determines E state or not)
                return SHARING;
            } else {
                *ca = INVALIDATE;
                return INVALID;
            }
        case INVALID_SHARING:
            if (reqType == SHARED) return SHARING;
            else if (reqType == DATA) {
                *ca = DATA_RECV;
                return EXCLUSIVE_CLEAN; // move to E state if we receive DATA without receiving SHARED
            } return INVALID_SHARING;
        case EXCLUSIVE_CLEAN:
            if (reqType == BUSWR) { // rest is same logic as SHARING state
                *ca = INVALIDATE;
                return INVALID;
            } else {
                indicateShared(addr, procNum);
                return SHARING;
            }
        default:
            break;
    } return INVALID;
}

coherence_states
cacheMOESI(uint8_t is_read, uint8_t* permAvail, coherence_states currentState,
        uint64_t addr, int procNum)
{
    switch(currentState) {
        case INVALID:
            *permAvail = 0; // indicate permissions not available yet, need to go to intermediate state first
            if (is_read) {
                sendBusRd(addr, procNum); // send BusRd, wait and see if anyone asserts SHARED
                return INVALID_SHARING; // intermediate between I -> S and I -> E
            } else {
                sendBusWr(addr, procNum); 
                return INVALID_MODIFIED;
            }
        case MODIFIED:
            *permAvail = 1; // permissions already available
            return MODIFIED;
        case INVALID_MODIFIED:
            *permAvail = 0; // do nothing; snoopMI will move this state to modified eventually when DATA is received
            return INVALID_MODIFIED;
        case SHARING:
            if (is_read) {
                *permAvail = 1; // if we are reading, then permissions already available
                return SHARING;
            } else {
                *permAvail = 0;
                sendBusWr(addr, procNum); // same actions as moving from I -> M
                return INVALID_MODIFIED;
            }
        case INVALID_SHARING:
            *permAvail = 0; // do nothing; snoopMI will handle this case as well
            return INVALID_SHARING;
        case EXCLUSIVE_CLEAN:
            *permAvail = 1; // in E state, we already have RW permissions
            return is_read ? EXCLUSIVE_CLEAN : MODIFIED;
        case OWNED: // same logic as 'S' state?
            if (is_read) {
                *permAvail = 1;
                return OWNED;
            } else {
                *permAvail = 0;
                sendBusWr(addr, procNum);
                return INVALID_MODIFIED;
            }
        default:
            break;
    } return INVALID;
}

coherence_states
snoopMOESI(bus_req_type reqType, cache_action* ca, coherence_states currentState,
        uint64_t addr, int procNum)
{
    *ca = NO_ACTION;
    switch (currentState) {
        case INVALID:
            return INVALID;
        case MODIFIED:
            if (reqType == BUSRD) {
                // indicateShared(addr, procNum); // whether or not we still need this depends on whether OWNED needs to send SHARED
                sendData(addr, procNum); // want DATA to be sent after SHARED bc we decide I -> S vs. I -> E based on SHARED
                return OWNED; // move M -> S if BusRd, else move M -> I (how do we flush here???)
            } else if (reqType == BUSWR) {
                sendData(addr, procNum);
                *ca = INVALIDATE;
                return INVALID;
            }
        case INVALID_MODIFIED:
            if (reqType == DATA || reqType == SHARED) {
                *ca = DATA_RECV;
                return MODIFIED;
            } return INVALID_MODIFIED;
        case SHARING:
            if (reqType == BUSRD) {
                indicateShared(addr, procNum); // indicate shared if another processor attempts to read (determines E state or not)
                return SHARING;
            } else if (reqType == BUSWR) {
                *ca = INVALIDATE;
                return INVALID;
            }
        case INVALID_SHARING:
            if (reqType == SHARED) return SHARING;
            else if (reqType == DATA) {
                *ca = DATA_RECV;
                return EXCLUSIVE_CLEAN; // move to E state if we receive DATA without receiving SHARED
            } return INVALID_SHARING;
        case EXCLUSIVE_CLEAN:
            if (reqType == BUSWR) { // rest is same logic as SHARING state
                *ca = INVALIDATE;
                return INVALID;
            } else if (reqType == BUSRD) {
                indicateShared(addr, procNum);
                return SHARING;
            }
        case OWNED:
            if (reqType == BUSRD) {
                // indicateShared(addr, procNum) // need this???
                sendData(addr, procNum);
                return OWNED;
            } else if (reqType == BUSWR) {
                sendData(addr, procNum);
                *ca = INVALIDATE;
                return INVALID;
            }
        default:
            break;
    } return INVALID;
}
