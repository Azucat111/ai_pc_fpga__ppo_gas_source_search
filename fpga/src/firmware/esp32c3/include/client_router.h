#pragma once
#include <stdint.h>

// No Arduino dependencies: the same arbitration code is tested on the PC.
struct CarEndpoint {
    uint32_t ip=0;
    uint16_t port=0;
    bool operator==(const CarEndpoint &other) const { return ip==other.ip && port==other.port; }
};

class ClientRouter {
public:
    enum Decision { Forward, Busy, NotReady };
    struct Subscriber { CarEndpoint endpoint; uint32_t seen=0; bool used=false; };
    Subscriber subscribers[2];

    bool blocked(CarEndpoint endpoint) const { return owned && !(endpoint==owner); }

    bool subscribe(CarEndpoint endpoint,uint32_t now) {
        for(auto &s:subscribers) if(s.used && s.endpoint==endpoint) { s.seen=now;return true; }
        // New CLI invocations use a new UDP port. Reuse that computer's slot,
        // but do not redirect the active controller's telemetry mid-action.
        for(auto &s:subscribers) if(s.used && s.endpoint.ip==endpoint.ip) {
            if(owned && s.endpoint==owner) return false;
            s.endpoint=endpoint;s.seen=now;return true;
        }
        for(auto &s:subscribers) {
            if(!s.used || (uint32_t(now-s.seen)>=60000 && !(owned && s.endpoint==owner))) {
                s.endpoint=endpoint;s.seen=now;s.used=true;return true;
            }
        }
        return false;
    }
    bool active(const Subscriber &s,uint32_t now) const {
        return s.used && uint32_t(now-s.seen)<60000;
    }
    static uint32_t word(const uint8_t *p) {
        return uint32_t(p[0]) | (uint32_t(p[1])<<8) | (uint32_t(p[2])<<16) | (uint32_t(p[3])<<24);
    }
    static bool atLeast(uint32_t a,uint32_t b) { return int32_t(a-b)>=0; }

    // Called only for CRC-checked complete frames. Offsets follow protocol v1.
    void telemetry(const uint8_t *p,uint32_t now) {
        const uint8_t *w=p+8;
        uint32_t nextBoot=word(w),nextFrame=word(p+4);
        if(haveTelemetry && nextBoot==boot && !atLeast(nextFrame,frame+1)) return;
        if(!haveTelemetry || nextBoot!=boot) owned=false;
        boot=nextBoot;frame=nextFrame;boardMs=word(w+4);lastSequence=word(w+8);
        mode=word(w+16);telemetryAt=now;haveTelemetry=true;
        // Never release on a queued STOP snapshot from before the forwarded command.
        // Rejected commands also have a checked-sequence acknowledgement.
        if(owned && mode==0 && (atLeast(lastSequence,pendingSequence) || atLeast(word(w+42*4),pendingSequence))) owned=false;
    }

    Decision command(CarEndpoint endpoint,const uint8_t *p,uint32_t now) {
        uint8_t requested=p[16];
        // STOP from either station always reaches the PL, including with a stale boot.
        if(requested==0) {
            if(owned && word(p+8)==boot && atLeast(word(p+4),pendingSequence)) pendingSequence=word(p+4);
            return Forward;
        }
        if(blocked(endpoint)) return Busy;
        if(!haveTelemetry || uint32_t(now-telemetryAt)>150 || !boot) return NotReady;
        uint32_t seq=word(p+4),ttl=uint32_t(p[18])|(uint32_t(p[19])<<8);
        // Reject stale/wrong-boot requests without acquiring ownership.
        uint32_t age=boardMs+uint32_t(now-telemetryAt)-word(p+12);
        if(word(p+8)!=boot || ttl<100 || ttl>500 || age>ttl || requested>4 || p[17] || p[28] || p[29]) return NotReady;
        if(!owned) {
            // An orphan heartbeat cannot acquire control. Wait for a stopped PL.
            if(requested==4 || mode!=0 || !atLeast(seq,lastSequence+1)) return NotReady;
            owner=endpoint;owned=true;pendingSequence=seq;
        } else if(atLeast(seq,pendingSequence)) pendingSequence=seq;
        return Forward;
    }
private:
    CarEndpoint owner;
    bool owned=false,haveTelemetry=false;
    uint32_t boot=0,frame=0,boardMs=0,lastSequence=0,mode=0,telemetryAt=0,pendingSequence=0;
};
