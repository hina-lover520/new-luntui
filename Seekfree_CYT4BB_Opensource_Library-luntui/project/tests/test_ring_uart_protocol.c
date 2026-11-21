#include "ring_link.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do {if (!(x)) {fprintf(stderr,"FAIL protocol line %d\n",__LINE__);exit(1);}}while(0)
int main(void)
{
    int error, route, cap, degree;
    unsigned count=0;
    for(error=-90;error<=90;++error) for(route=-1;route<=1;++route) {
        int32_t word=ring_command_diagnostics(3,1,1,1,1,1,2,error,119,route);
        CHECK(ring_command_diag_fault(word)==3);
        CHECK(ring_command_diag_candidate(word)==1 && ring_command_diag_cross_veto(word)==1);
        CHECK(ring_command_diag_committed(word)==1 && ring_command_diag_path_valid(word)==1);
        CHECK(ring_command_diag_stop(word)==1 && ring_command_diag_raw_state(word)==2);
        CHECK(ring_command_diag_error(word)==error && ring_command_diag_near(word)==119);
        CHECK(ring_command_diag_route(word)==route); ++count;
    }
    CHECK(ring_command_diag_near(ring_command_diagnostics(0,0,0,0,0,0,1,0,-1,0))==-1);
    for(cap=-1;cap<=100;++cap) for(degree=-400;degree<=400;degree+=20) {
        int32_t word=ring_command_cap_turn(cap,degree);
        CHECK(ring_command_cap_value(word)==cap);
        CHECK(ring_command_turn_deg(word)==degree); ++count;
    }
    {
        int side, source, cls, score, cc;
        for(side=-1; side<=1; ++side) for(source=0; source<=6; ++source)
        for(cls=0; cls<=3; ++cls) for(score=0; score<=255; score+=51)
        for(cc=0; cc<=255; cc+=51) {
            int32_t flags=ring_diagnostic_flags(1,0,score,cc,side,source,1,cls);
            CHECK(ring_diagnostic_detected(flags)==1);
            CHECK(ring_diagnostic_allowed(flags)==0);
            CHECK(ring_diagnostic_branch(flags)==1);
            CHECK(ring_diagnostic_score(flags)==score);
            CHECK(ring_diagnostic_confirm_frames(flags)==cc);
            CHECK(ring_diagnostic_side(flags)==side);
            CHECK(ring_diagnostic_source(flags)==source);
            CHECK(ring_diagnostic_classification(flags)==cls);
            CHECK(((uint32_t)flags & 0xfc000000u)==0u);
            ++count;
        }
        CHECK(ring_diagnostic_detected(ring_diagnostic_flags(0,1,0,0,0,0,0,0))==0);
        CHECK(ring_diagnostic_allowed(ring_diagnostic_flags(0,1,0,0,0,0,0,0))==1);
        for(error=-1;error<=187;++error) for(route=-1;route<=187;route+=4) {
            int32_t points=ring_diagnostic_points(error,route);
            CHECK(ring_diagnostic_near_x(points)==error);
            CHECK(ring_diagnostic_look_x(points)==route);
            ++count;
        }
        CHECK(ring_diagnostic_near_x(ring_diagnostic_points(-32768,32767))==-32768);
        CHECK(ring_diagnostic_look_x(ring_diagnostic_points(-32768,32767))==32767);
        CHECK((uint32_t)ring_diagnostic_timing(65535u,65535u)==0xffffffffu);
        CHECK((uint16_t)(uint32_t)ring_diagnostic_timing(46u,0x1234u)==46u);
        CHECK(((uint32_t)ring_diagnostic_timing(46u,0x1234u)>>16)==0x1234u);
        CHECK(sizeof(ring_packet_t)==32u);
        CHECK(RING_COMMAND_ADDRESS==0x28001040 && RING_MOTION_ADDRESS==0x28001060);
        CHECK(RING_DIAGNOSTIC_ADDRESS==0x28001080);
        CHECK(RING_ENTRY_DIAGNOSTIC_ADDRESS==0x280010a0);
        CHECK(RING_GEOMETRY_DIAGNOSTIC_ADDRESS==0x280010c0);
        CHECK(RING_GEOMETRY_DIAGNOSTIC_MAGIC!=RING_LINK_MAGIC &&
              RING_GEOMETRY_DIAGNOSTIC_MAGIC!=RING_DIAGNOSTIC_MAGIC &&
              RING_GEOMETRY_DIAGNOSTIC_MAGIC!=RING_ENTRY_DIAGNOSTIC_MAGIC);
        CHECK(RING_ENTRY_DIAGNOSTIC_MAGIC!=RING_LINK_MAGIC &&
              RING_ENTRY_DIAGNOSTIC_MAGIC!=RING_DIAGNOSTIC_MAGIC);
        {
            static const int angles[]={-32768,-1800,-1,0,1,1800,32767};
            unsigned a,b,flags,reject,decision;
            for(a=0;a<sizeof(angles)/sizeof(angles[0]);++a)
            for(b=0;b<sizeof(angles)/sizeof(angles[0]);++b) {
                int32_t value=ring_entry_diagnostic_angles(angles[a],angles[b]);
                CHECK(ring_entry_diagnostic_hold_turn(value)==angles[a]);
                CHECK(ring_entry_diagnostic_entry_turn(value)==angles[b]);++count;
            }
            for(flags=0;flags<8192u;flags+=127u) for(reject=0;reject<64u;reject+=3u)
            for(decision=0;decision<=14u;++decision) {
                uint32_t value=(uint32_t)ring_entry_diagnostic_flags(flags,reject,decision);
                CHECK((value&65535u)==flags);
                CHECK(((value>>16)&255u)==reject);
                CHECK((value>>24)==decision);++count;
            }
            CHECK((uint32_t)ring_entry_diagnostic_age(65535u,255u)==0x00ffffffu);
        }
        {
            unsigned mode,rows,assoc,support,y;
            int x;
            for(mode=0;mode<=6u;++mode) for(rows=0;rows<=120u;rows+=10u)
            for(assoc=0;assoc<=120u;assoc+=10u) for(support=0;support<=7u;++support) {
                uint32_t value=(uint32_t)ring_geometry_diagnostic_flags(mode,rows,assoc,support);
                CHECK((value&255u)==mode && ((value>>8)&255u)==rows);
                CHECK(((value>>16)&255u)==assoc && (value>>24)==support);++count;
            }
            for(x=-1;x<188;++x) for(y=0;y<120u;y+=7u) {
                int32_t value=ring_geometry_diagnostic_guide(x,y);
                CHECK(ring_geometry_diagnostic_x(value)==x);
                CHECK(ring_geometry_diagnostic_y(value)==y);++count;
            }
            CHECK(ring_geometry_diagnostic_x(ring_geometry_diagnostic_guide(-1,255u))==-1);
            CHECK(ring_geometry_diagnostic_y(ring_geometry_diagnostic_guide(-1,255u))==255u);
        }
        CHECK(RING_LINK_MAGIC==0x524e4704 && RING_DIAGNOSTIC_MAGIC!=RING_LINK_MAGIC);
    }
    {
        int nx,ny,fx,fy,progress,first;
        unsigned gate,ref,nc,fc,gap;
        for(nx=-1;nx<188;nx+=13) for(ny=-1;ny<120;ny+=11)
        for(fx=-1;fx<188;fx+=17) for(fy=-1;fy<120;fy+=19) {
            int32_t value=ring_mouth_points(nx,ny,fx,fy);
            CHECK(ring_mouth_coordinate(value,0)==nx && ring_mouth_coordinate(value,8)==ny);
            CHECK(ring_mouth_coordinate(value,16)==fx && ring_mouth_coordinate(value,24)==fy);++count;
        }
        for(gate=0;gate<=32767u;gate+=113u) for(ref=0;ref<=1;ref++)
        for(nc=0;nc<=20u;nc+=3) for(fc=0;fc<=20u;fc+=4) for(gap=0;gap<=20u;gap+=5) {
            uint32_t value=(uint32_t)ring_mouth_gates(gate,ref,nc,fc,gap);
            CHECK(ring_mouth_gate_value((int32_t)value)==gate && ((value>>13)&1u)==ref);
            CHECK(((value>>14)&15u)==(nc>15u?15u:nc));
            CHECK(((value>>18)&15u)==(fc>15u?15u:fc));
            CHECK(((value>>22)&3u)==(gap>3u?3u:gap));++count;
        }
        for(first=-1;first<120;first+=7) for(progress=-200;progress<=200;progress+=9) {
            int32_t value=ring_mouth_progress_pair(first,progress,13,13);
            int expected=progress>127?127:(progress< -127?-127:progress);
            CHECK(ring_mouth_coordinate(value,0)==first && ring_mouth_progress(value)==expected);
            CHECK(((uint32_t)value>>16)==0x0d0du);++count;
        }
        CHECK(ring_command_version(ring_command_versioned_phase(2,64,13))==13u);
        CHECK(ring_command_boost(ring_command_versioned_phase(2,-64,13))==-64);
        CHECK(ring_motion_version(ring_motion_operator_version(RING_OPERATOR_VALID|RING_OPERATOR_RUN,13))==13u);
    }
    {
        unsigned missed,reason;
        for(missed=0;missed<=255u;++missed) for(reason=0;reason<=3u;++reason) {
            uint32_t original=(uint32_t)ring_mouth_gates(8191,1,15,15,15);
            uint32_t value=(uint32_t)ring_mouth_tracking((int32_t)original,missed,reason);
            CHECK((value&0x03ffffffu)==original);
            CHECK(((value>>26)&7u)==(missed>7u?7u:missed));
            CHECK(((value>>29)&3u)==reason && !(value&0x80000000u));++count;
        }
        CHECK(RING_PAIR_VERSION==36u);
        CHECK(ring_command_version(ring_command_versioned_phase(2,64,RING_PAIR_VERSION))==RING_PAIR_VERSION);
        CHECK(ring_motion_version(ring_motion_operator_version(RING_OPERATOR_VALID,RING_PAIR_VERSION))==RING_PAIR_VERSION);
    }
    {
        unsigned reject,candidate,latched,confirmed;
        static const int laps[]={-40000,-32768,-1,0,3500,3800,3950,32767,40000};
        unsigned i;
        CHECK(RING_EXIT_DIAGNOSTIC_ADDRESS==0x280011c0);
        CHECK(RING_MOUTH_DIAGNOSTIC_ADDRESS==0x280011a0);
        CHECK(RX_FORWARD_MREV==RING_LINK_WORDS-1 && sizeof(ring_packet_t)==32u);
        for(reject=0;reject<=127u;++reject) for(candidate=0;candidate<=1u;++candidate)
        for(latched=0;latched<=1u;++latched) for(confirmed=0;confirmed<=1u;++confirmed) {
            uint32_t status=(uint32_t)ring_exit_diagnostic_status(reject,candidate,latched,confirmed);
            CHECK((status&127u)==reject && ((status>>16)&1u)==candidate);
            CHECK(((status>>17)&1u)==latched && ((status>>18)&1u)==confirmed);
            CHECK(!(status&~0x0007007fu));++count;
        }
        for(i=0;i<sizeof(laps)/sizeof(laps[0]);++i) {
            int expected=laps[i]>32767?32767:(laps[i]< -32768?-32768:laps[i]);
            uint32_t packed=(uint32_t)ring_exit_diagnostic_counts(300u,256u,laps[i]);
            CHECK((packed&255u)==255u && ((packed>>8)&255u)==255u);
            CHECK(ring_exit_diagnostic_lap10((int32_t)packed)==expected);++count;
        }
    }
    printf("PASS: v4 control and prior diagnostic packing preserved; v24 paired version and independent F-window/ready/path diagnostics, %u cases\n",count);
    return 0;
}
