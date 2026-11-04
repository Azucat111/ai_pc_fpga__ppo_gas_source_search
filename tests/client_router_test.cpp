#include "client_router.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void put(uint8_t *p,uint32_t n) { for(int i=0;i<4;++i) p[i]=uint8_t(n>>(8*i)); }
static uint8_t telemetry[192],packet[32];
static CarEndpoint a,b,c;
static void snapshot(ClientRouter &r,uint32_t frame,uint32_t mode,uint32_t accepted,uint32_t checked,uint32_t now=100,uint32_t boot=7) {
    memset(telemetry,0,sizeof(telemetry));put(telemetry+4,frame);put(telemetry+8,boot);
    put(telemetry+12,now);put(telemetry+16,accepted);put(telemetry+24,mode);
    put(telemetry+8+42*4,checked);r.telemetry(telemetry,now);
}
static ClientRouter::Decision send(ClientRouter &r,CarEndpoint who,uint32_t seq,uint8_t mode,uint32_t now=100,uint32_t boot=7) {
    memset(packet,0,sizeof(packet));put(packet+4,seq);put(packet+8,boot);put(packet+12,now);
    packet[16]=mode;packet[18]=0xf4;packet[19]=1;return r.command(who,packet,now);
}
int main() {
    a.ip=1;a.port=100;b.ip=2;b.port=200;c.ip=3;c.port=300;
    ClientRouter r;
    assert(r.subscribe(a,100) && r.subscribe(b,100));assert(!r.subscribe(c,100));
    assert(r.active(r.subscribers[0],100) && r.active(r.subscribers[1],100));
    assert(send(r,a,1,2)==ClientRouter::NotReady);
    snapshot(r,1,0,0,0);
    assert(send(r,a,1,4)==ClientRouter::NotReady); // orphan heartbeat
    assert(send(r,a,1,2)==ClientRouter::Forward);
    assert(send(r,b,1,3)==ClientRouter::Busy);
    assert(send(r,a,1,2)==ClientRouter::Forward); // retransmission is still PL checked
    snapshot(r,2,0,0,0); // pre-command STOP cannot surrender ownership
    assert(send(r,b,2,2)==ClientRouter::Busy);
    snapshot(r,3,2,1,1);
    assert(send(r,a,2,4)==ClientRouter::Forward);
    assert(send(r,b,3,4)==ClientRouter::Busy);
    assert(send(r,b,0,0,100,1)==ClientRouter::Forward); // stale STOP always forwarded
    snapshot(r,4,0,2,0); // STOP can carry a sequence older than the action
    assert(send(r,b,3,3)==ClientRouter::Forward);
    assert(send(r,a,4,2)==ClientRouter::Busy);
    snapshot(r,5,3,3,3);
    snapshot(r,6,0,3,3,650); // PL lease expiry / turn completion releases owner
    assert(send(r,a,4,2,650)==ClientRouter::Forward);
    assert(!r.subscribe(c,650));
    snapshot(r,7,0,3,4,650); // rejected action also releases control
    assert(send(r,b,4,3,650)==ClientRouter::Forward);
    snapshot(r,1,0,0,0,700,8); // PL reboot invalidates ownership
    assert(send(r,a,1,2,700,7)==ClientRouter::NotReady);
    assert(send(r,a,1,2,700,8)==ClientRouter::Forward);
    snapshot(r,2,2,1,1,700,8);
    assert(send(r,b,2,2,1400,8)==ClientRouter::Busy); // no ESP-only lease unlock
    snapshot(r,3,0,1,1,1400,8);
    assert(send(r,b,2,2,1600,8)==ClientRouter::NotReady); // stale telemetry
    // Two slots are per computer, not per invocation's ephemeral UDP port.
    ClientRouter slots;
    assert(slots.subscribe(a,100) && slots.subscribe(b,100));
    CarEndpoint next=a;next.port=101;
    assert(slots.subscribe(next,200));assert(slots.subscribers[0].endpoint==next);
    snapshot(slots,1,0,0,0,200);
    assert(send(slots,next,1,2,200)==ClientRouter::Forward);
    assert(!slots.subscribe(a,201)); // preserve current owner's return path
    assert(slots.subscribe(c,60200)); // expired observer can be replaced
    assert(slots.subscribers[0].endpoint==next); // active owner's slot cannot be evicted
    // Unsigned uptime rollover and sequence rollover remain supported.
    ClientRouter wrap;
    snapshot(wrap,0xfffffffe,0,0xfffffffe,0xfffffffe,0xfffffff0);
    assert(send(wrap,a,0xffffffff,2,0xfffffff0)==ClientRouter::Forward);
    snapshot(wrap,0xffffffff,2,0xffffffff,0xffffffff,0xfffffff5);
    assert(send(wrap,a,0,4,5)==ClientRouter::Forward);
    snapshot(wrap,0,0,0,0,10);
    assert(send(wrap,b,1,3,10)==ClientRouter::Forward);
    puts("PASS two-client router: fanout registration, exclusivity, STOP, handoff, reboot, stale links, slot reuse, rollover");
}
