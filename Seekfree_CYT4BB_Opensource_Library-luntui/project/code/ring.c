#include "ring.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct { int16_t left, right; } run_t;
typedef struct {
    int valid, top, bottom, left, right, center;
    int growing, returning, bridge, clipped;
} island_t;
#define MAX_RUNS (RING_W / RING_MIN_RUN + 1)
ring_result_t ring_result;
static run_t row_runs[RING_H][MAX_RUNS];
static uint8_t row_count[RING_H], connected[RING_H][MAX_RUNS];
static int16_t observed_left[RING_H], observed_right[RING_H];
static int16_t gap_left[RING_H], gap_right[RING_H];
static int16_t lane_left[RING_H], lane_right[RING_H], inner[RING_H];
static int16_t previous_mid[RING_H], previous_inner[RING_H];
static int16_t strong_inner[RING_H];
static uint32_t phase_since, ring_since, last_path_ms, last_path_fault_ms, cooldown_since;
static uint32_t reentry_since;
static uint8_t reentry_latched;
static uint32_t tangent_force_since;
static unsigned maneuver_feedback, timed_started,lap_completed,visual_exit_count;
static int16_t maneuver_turn10;
static unsigned exit_turn_feedback;
static uint32_t last_frame_ms, observe_since, vision_ms;
static float entry_yaw, entry_revs, cooldown_revs, confirm_yaw, confirm_revs;
static float last_yaw, observe_travel, observe_last_revs;
static float exit_slope, exit_offset;
static int original_state, have_path, confirm_count, stable_count, exit_count;
static int cancel_count, lost_frames, candidate_x, candidate_anchor, confirm_route, observe_frames;
static int observed_growth, observed_return, observing;
static int16_t trusted_steer, trusted_near, trusted_look;
static int entry_origin_row, entry_last_row, entry_progress_row, entry_high_row, entry_changes, entry_measurements;
static int support_frames;
static float support_travel, support_last_revs;
static int seed_left, seed_right, seed_x, seed_inner;
static int seed_identity_lost;
static int assist_state, assist_side, assist_count, assist_candidate, assist_clock;
static float assist_yaw, assist_revs, assist_total_yaw, assist_total_revs;
static float tangent_slope, tangent_offset, tangent_far_revs;
static float mouth_entry_slope,mouth_entry_offset;
static int16_t mouth_entry_inner[RING_H];
static int mouth_entry_inner_count;
static float tangent_lap_revs;
static int tangent_near_close, tangent_gap;
static int tangent_left_count, tangent_return_count, tangent_return_seen;
typedef struct {
    int valid,x,y,first_y,count,progress,missed;
    float yaw,origin_yaw,first_forward,high_forward;
    uint32_t last_seen_ms;
} mouth_track_t;
static mouth_track_t mouth_near,mouth_far,mouth_near_closing;
static int16_t mouth_centre[RING_H],mouth_outside[RING_H];
static int16_t mouth_left[RING_H],mouth_right[RING_H];
static uint32_t mouth_signature,mouth_previous_signature;
static float mouth_previous_yaw,mouth_previous_revs;
static int mouth_corridor_seen;
static int mouth_ready;
static int mouth_open_seen;
static int mouth_near_cleared,mouth_clear_count;
static int mouth_corridor_rows,mouth_corridor_clipped,mouth_pass_reason;
static int16_t mouth_previous_lower[RING_H];
static int16_t mouth_bottom_anchor[10];
static int mouth_bottom_count;
static uint32_t mouth_bottom_since;
static int mouth_closure_count,mouth_closure_x[6],mouth_closure_y[6];
static int mouth_portal_row,mouth_portal_x,mouth_far_source;
static int16_t mouth_local_edge[RING_H];
static int mouth_local_valid,mouth_local_changes;
static uint32_t mouth_local_ms,mouth_local_curve_ms;
static float mouth_local_yaw,mouth_local_revs;
static uint16_t supplied_interval;
static uint8_t interval_supplied;
/* Exit observations never overwrite the ring path before a full lap.
 * A latched exit is tracked independently from the island boundary. */
typedef struct {
    int16_t mid[RING_H],left[RING_H],right[RING_H];
    int near_x,look_x,far_rows,near_rows;
    float yaw;
} exit_probe_t;
static exit_probe_t exit_candidate_path,exit_locked_path;
static unsigned exit_candidate_count;
static uint32_t exit_candidate_since,exit_candidate_last_ms,exit_start_ms;
static float exit_start_yaw,exit_start_revs;
static uint8_t exit_track_valid,exit_net_reached;
#ifdef RING_HOST_TEST
uint32_t ring_scan_pixels;
uint32_t ring_graph_checks;
#define PIXEL(row,x) (++ring_scan_pixels,(row)[x])
#define GRAPH_CHECK() (++ring_graph_checks)
#else
#define PIXEL(row,x) ((row)[x])
#define GRAPH_CHECK() ((void)0)
#endif
static uint16_t hold_rejections(const ring_motion_t *m);
static int assist_history_fresh(const ring_motion_t *m);
static int tangent_normal_path(void);
static void tangent_outer_reset(void);
static int tangent_outer_fresh(const ring_motion_t *m);

static int clamp(int x,int lo,int hi) { return x<lo?lo:(x>hi?hi:x); }
static int runs_at(const uint8_t *row,run_t *runs)
{
    int x=0,n=0;
    while(x<RING_W) {
        int l;
        while(x<RING_W && !PIXEL(row,x)) ++x;
        l=x;
        while(x<RING_W && PIXEL(row,x)) ++x;
        if(x-l>=RING_MIN_RUN) { runs[n].left=(int16_t)l; runs[n++].right=(int16_t)(x-1); }
    }
    return n;
}

/* One pixel scan, then a bottom-up run graph. Adjacency uses only two
 * pixels of segmentation tolerance, NOT the 18px steering step: that larger
 * value could join two roads across a real dark separator. No pixel rescan. */
static void scan_geometry(const uint8_t image[RING_H][RING_W],const ring_motion_t *m)
{
    int y,j,k,x=RING_W/2,seed=-1,best=10000;
    seed_identity_lost=0;
    memset(connected,0,sizeof(connected));
    for(y=0;y<RING_H;++y) {
        row_count[y]=(uint8_t)runs_at(image[y],row_runs[y]);
        observed_left[y]=observed_right[y]=gap_left[y]=gap_right[y]=-1;
    }
    /* Current geometry fingerprint costs no second image scan. Exact
     * repeated geometry is usable for control, never motion evidence. */
    mouth_signature=2166136261u;
    for(y=0;y<RING_H;++y) for(j=0;j<row_count[y];++j) {
        mouth_signature=(mouth_signature^(uint32_t)(y+1))*16777619u;
        mouth_signature=(mouth_signature^(uint32_t)(row_runs[y][j].left+1))*16777619u;
        mouth_signature=(mouth_signature^(uint32_t)(row_runs[y][j].right+1))*16777619u;
    }
    if(ring_owns_path() && have_path && seed_left>=0) {
        int fresh=(!hold_rejections(m) && support_frames<(int)RING_HOLD_FRAMES &&
            support_travel+fabsf(m->wheel_revs-support_last_revs)<=RING_HOLD_REVS) ||
            assist_history_fresh(m) || tangent_outer_fresh(m);
        /* Acquisition still starts at the vehicle centre. Once acquired,
         * the bottom run belongs to the measured branch, even when that
         * branch moves away from centre. A nearby unrelated road cannot
         * replace it. Reuse the original evidence bounds; no new grace. */
        for(j=0;j<row_count[117];++j) {
            run_t r=row_runs[117][j];
            int d=abs(r.left-seed_left)+abs(r.right-seed_right);
            int expected=fresh?seed_x:RING_W/2;
            int distance=expected<r.left?r.left-expected:(expected>r.right?expected-r.right:0);
            int new_inner=ring_result.route>0?r.left:r.right;
            if(r.left>seed_right || r.right<seed_left ||
               distance>RING_MAX_ROW_STEP ||
               (seed_inner>=0 && abs(new_inner-seed_inner)>RING_MAX_ROW_STEP)) continue;
            if(d<best) {best=d;seed=j;}
        }
        if(seed>=0) x=clamp(seed_x,row_runs[117][seed].left,row_runs[117][seed].right);
        else {
            /* The original centre component can still provide strong
             * ordinary/cross cancellation evidence. It is explicitly NOT
             * a replacement for the missing acquired steering branch. */
            best=10000;
            for(j=0;j<row_count[117];++j) {
                run_t r=row_runs[117][j];
                int d=x<r.left?r.left-x:(x>r.right?x-r.right:0);
                if(d<best) {best=d;seed=j;}
            }
            if(seed<0 || best>RING_MAX_ROW_STEP) return;
            /* An observed replacement is different from a blank/occluded
             * image: only the former disproves the seed identity. The
             * latter retains the existing bounded control/history hold. */
            seed_identity_lost=1;
        }
    } else {
        for(j=0;j<row_count[117];++j) {
            run_t r=row_runs[117][j];
            int d=x<r.left?r.left-x:(x>r.right?x-r.right:0);
            if(d<best) {best=d;seed=j;}
        }
        if(seed<0 || best>RING_MAX_ROW_STEP) return;
    }
    connected[117][seed]=1;
    for(y=116;y>=0;--y) {
        int base=0;
        /* Sorted interval sweep, not an all-pairs run adjacency matrix. */
        for(j=0;j<row_count[y];++j) {
            while(base<row_count[y+1] && row_runs[y+1][base].right<row_runs[y][j].left-RING_CONNECT_GAP) {
                GRAPH_CHECK();++base;
            }
            for(k=base;k<row_count[y+1] && row_runs[y+1][k].left<=row_runs[y][j].right+RING_CONNECT_GAP;++k) {
                GRAPH_CHECK();
                if(connected[y+1][k] &&
                   row_runs[y][j].left<=row_runs[y+1][k].right+RING_CONNECT_GAP &&
                   row_runs[y][j].right>=row_runs[y+1][k].left-RING_CONNECT_GAP) {
                    connected[y][j]=1; break;
                }
            }
        }
    }
    for(y=117;y>=0;--y) {
        int pick=-1,score=10000,gap_score=10000;
        for(j=0;j<row_count[y];++j) {
            int l=row_runs[y][j].left,r=row_runs[y][j].right;
            int d=x<l?l-x:(x>r?x-r:0);
            if(connected[y][j] && d<score) {pick=j;score=d;}
            if(j+1<row_count[y] && connected[y][j] && connected[y][j+1]) {
                int gl=r+1,gr=row_runs[y][j+1].left-1,w=gr-gl+1,c=(gl+gr)/2;
                int dist=abs(c-(observing?candidate_x:RING_W/2));
                if(w>=RING_MIN_ISLAND_WIDTH && w<=RING_MAX_ISLAND_WIDTH && dist<gap_score) {
                    gap_left[y]=(int16_t)gl;gap_right[y]=(int16_t)gr;gap_score=dist;
                }
            }
        }
        if(pick>=0 && score<=RING_MAX_ROW_STEP) {
            observed_left[y]=row_runs[y][pick].left;
            observed_right[y]=row_runs[y][pick].right;
            x=clamp(x,observed_left[y],observed_right[y]);
        }
    }
}

static int connected_span(int y,int l,int r)
{
    int j;
    if(y<0 || y>117) return 0;
    for(j=0;j<row_count[y];++j)
        if(connected[y][j] && row_runs[y][j].left<=l && row_runs[y][j].right>=r) return 1;
    return 0;
}

static int fit_edge(int side,int first,int last,float *slope,float *offset)
{
    int y,n=0,invalid=0,bad=0;
    float sy=0,sx=0,syy=0,syx=0,den;
    for(y=first;y<=last;++y) {
        int x=side>0?observed_right[y]:observed_left[y];
        if(x<=2 || x>=RING_W-3) {++invalid;continue;}
        ++n;sy+=y;sx+=x;syy+=y*y;syx+=y*x;
    }
    if(n<8 || invalid>(last-first+1)/6) return 0;
    den=n*syy-sy*sy;
    if(den<=0) return 0;
    *slope=(n*syx-sy*sx)/den; *offset=(sx-*slope*sy)/n;
    if(fabsf(*slope)>1.5f) return 0;
    for(y=first;y<=last;++y) {
        int x=side>0?observed_right[y]:observed_left[y];
        if(x>2 && x<RING_W-3 && fabsf(x-(*slope*y+*offset))>6.0f) ++bad;
    }
    return bad<=2;
}

/* A cross needs a connected and ALIGNED through-lane above both openings.
 * raw=2 alone is ambiguous. Ring + cross geometry is an explicit conflict. */
static int cross_evidence(void)
{
    int y,left=0,right=0,ly=0,ry=0,top=118,far=0,max_far=0;
    float ls,li,rs,ri;
    if(!fit_edge(-1,102,117,&ls,&li) || !fit_edge(1,102,117,&rs,&ri)) return 0;
    for(y=26;y<=100;++y) {
        int l=observed_left[y],r=observed_right[y];
        if(l<0 || row_count[y]!=1) continue;
        if(l<=2 && ls*y+li-l>ring_result.lane_width/3) {++left;ly+=y;if(y<top)top=y;}
        if(r>=RING_W-3 && r-(rs*y+ri)>ring_result.lane_width/3) {++right;ry+=y;if(y<top)top=y;}
    }
    if(left<6 || right<6 || abs(ly/left-ry/right)>30) return 0;
    for(y=top-1;y>=6;--y) {
        int j,through=0;
        float expected=((ls+rs)*y+li+ri)*0.5f;
        /* Count the aligned connected through-lane, not every white fleck
         * anywhere in the row. Detached side noise must not erase a cross. */
        for(j=0;j<row_count[y];++j) {
            int l=row_runs[y][j].left,r=row_runs[y][j].right,w=r-l+1;
            if(connected[y][j] && l>2 && r<RING_W-3 &&
               w>=ring_result.lane_width/2 && w<=ring_result.lane_width*4/3 &&
               fabsf((l+r)*0.5f-expected)<=8.0f) ++through;
        }
        if(through==1) {
            ++far;if(far>max_far)max_far=far;
        } else far=0;
    }
    return max_far>=4;
}

static int observed_ordinary(void)
{
    int y,near=0,through=0;
    float ls,li,rs,ri;
    if(!fit_edge(-1,94,117,&ls,&li) || !fit_edge(1,94,117,&rs,&ri)) return 0;
    for(y=50;y<=116;++y) {
        int l=observed_left[y],r=observed_right[y],w=r-l+1;
        if(row_count[y]==1 && l>2 && r<RING_W-3 &&
           w>=ring_result.lane_width*2/3 && w<=ring_result.lane_width*4/3 &&
           fabsf((l+r)*0.5f-((ls+rs)*y+li+ri)*0.5f)<=6.0f) {
            if(y>=94) ++near;
            ++through;
        }
    }
    return near>=18 && through>=54;
}

/* The dark separator must split TWO runs belonging to the incoming road,
 * merge below the tip, widen and turn back. A far white cap is not required:
 * the upper island can leave the camera. A fork/rectangle without a return
 * stays a candidate and cannot acquire steering. */
static island_t find_split_island(void)
{
    island_t out={0};
    int y,best_length=0;
    for(y=108;y>=24;--y) {
        int bottom=y,top=y,k,peak=0,peak_y=y,low,upper=RING_W,bridge=0;
        if(gap_left[y]<0) continue;
        while(top>2 && gap_left[top-1]>=0 &&
              abs(gap_left[top-1]-gap_left[top])<=8 && abs(gap_right[top-1]-gap_right[top])<=8) --top;
        for(k=top;k<=bottom;++k) {
            int w=gap_right[k]-gap_left[k]+1;
            if(w>peak) {peak=w;peak_y=k;}
        }
        low=gap_right[bottom]-gap_left[bottom]+1;
        for(k=top;k<=peak_y-3;++k) {
            int w=gap_right[k]-gap_left[k]+1;
            if(w<upper) upper=w;
        }
        for(k=bottom+1;k<=bottom+10 && k<=117;++k)
            if(connected_span(k,gap_left[bottom]-2,gap_right[bottom]+2)) {bridge=1;break;}
        if(bottom-top+1>=RING_SPLIT_ROWS && bottom-top+1>best_length && bridge) {
            out.valid=1;out.top=top;out.bottom=bottom;
            out.left=gap_left[bottom];out.right=gap_right[bottom];
            out.center=(out.left+out.right)/2;
            out.growing=peak-low>=8 && bottom-peak_y>=3;
            out.returning=upper<RING_W && peak-upper>=6 && peak_y-top>=3;
            out.bridge=bridge;best_length=bottom-top+1;
        }
        y=top;
    }
    return out;
}

/* A side-clipped island has no second white run. Require its curved inner
 * edge, a white merge BELOW its tip and a substantial inward bulge. A lone
 * lost boundary, rectangular cross corner or ordinary monotone bend does
 * not supply these together. Reuse row runs; never rescan the pixels. */
static island_t find_side_island(void)
{
    island_t best={0};
    int16_t edge[RING_H];
    int side,y,best_length=0;
    for(side=-1;side<=1;side+=2) {
        for(y=0;y<RING_H;++y) {
            int l=observed_left[y],r=observed_right[y];
            int e=side>0?r:l;
            edge[y]=-1;
            if(l<0 || r-l+1<ring_result.lane_width/2 || e<=2 || e>=RING_W-3) continue;
            if(side>0 && r!=row_runs[y][row_count[y]-1].right) continue;
            if(side<0 && l!=row_runs[y][0].left) continue;
            edge[y]=(int16_t)e;
        }
        for(y=108;y>=24;--y) {
            int bottom=y,top=y,k,peak=0,peak_y=y,bridge=0,low;
            float chord,bow;
            int middle;
            if(edge[y]<0) continue;
            while(top>2 && edge[top-1]>=0 && abs(edge[top-1]-edge[top])<=12) --top;
            if(bottom-top+1<RING_BRANCH_ROWS || bottom-top+1<=best_length) {y=top;continue;}
            for(k=top;k<=bottom;++k) {
                int p=side>0?RING_W-1-edge[k]:edge[k];
                if(p>peak) {peak=p;peak_y=k;}
            }
            low=side>0?RING_W-1-edge[bottom]:edge[bottom];
            for(k=bottom+1;k<=bottom+10 && k<=117;++k) {
                if(connected_span(k,side>0?edge[bottom]-2:0,
                                     side>0?RING_W-1:edge[bottom]+2)) ++bridge;
                else bridge=0;
                if(bridge>=3) break;
            }
            middle=(top+bottom)/2;
            chord=edge[top]+(float)(edge[bottom]-edge[top])*(middle-top)/(bottom-top);
            bow=side*(chord-edge[middle]);
            if(bridge>=3 && peak-low>=8 && bottom-peak_y>=3 && bow>=3.0f) {
                best.valid=best.growing=best.returning=best.bridge=1;
                best.clipped=side;best.top=top;best.bottom=bottom;
                best.left=side>0?edge[bottom]:0;
                best.right=side>0?RING_W-1:edge[bottom];
                best.center=(best.left+best.right)/2;best_length=bottom-top+1;
            }
            y=top;
        }
    }
    if(best.valid) {
        /* Reconstruct the selected identity from the cached road boundary. */
        for(y=0;y<RING_H;++y) {
            gap_left[y]=gap_right[y]=-1;
            if(y>=best.top && y<=best.bottom) {
                gap_left[y]=(int16_t)(best.clipped>0?observed_right[y]+1:0);
                gap_right[y]=(int16_t)(best.clipped>0?RING_W-1:observed_left[y]-1);
            }
        }
    }
    return best;
}

static island_t find_island(void)
{
    island_t split=find_split_island(),side;
    if(split.valid && split.growing && split.returning) return split;
    side=find_side_island();
    return side.valid?side:split;
}

static void change_phase(ring_phase_t phase,uint32_t now)
{ ring_result.phase=phase;phase_since=now;stable_count=exit_count=0; }
static void fail(int reason,uint32_t now)
{ring_result.fault=(uint8_t)reason;ring_result.stop=1;ring_result.decision=RING_DECISION_FAULT;change_phase(RING_FAULT,now);}
uint8_t ring_owns_path(void)
{return RING_ENABLE && ((ring_result.timed_entry && ring_result.phase==RING_APPROACH) ||
    (ring_result.phase>=(RING_TRACK_MODE?RING_ENTER:RING_APPROACH) && ring_result.phase<=RING_RECOVER &&
     !ring_result.circle_observe_only));}
uint8_t ring_blocks_cross(void)
{return RING_ENABLE && !ring_result.circle_observe_only &&
    ring_result.phase!=RING_NORMAL && ring_result.phase!=RING_COOLDOWN &&
    (!RING_TRACK_MODE || ring_result.phase!=RING_APPROACH || ring_result.timed_entry);}
int ring_speed_limit(void)
{
    if(!RING_ENABLE) return -1;
    if(ring_result.stop || ring_result.phase==RING_FAULT) return 0;
    if(ring_result.phase==RING_APPROACH) return ring_result.timed_entry?RING_APPROACH_RPM:(RING_TRACK_MODE?-1:RING_APPROACH_RPM);
    if(ring_result.circle_observe_only && ring_result.phase==RING_CIRCLE)return -1;
    if(ring_owns_path())
        return ring_result.phase>=RING_EXIT?RING_EXIT_RPM:RING_CIRCLE_RPM;
    return -1;
}
void ring_observe_track(int state) {original_state=state;}
void ring_observe_frame_interval(uint16_t dt_ms) {supplied_interval=dt_ms;interval_supplied=1;}
void ring_observe_exit_turn_feedback(unsigned stage)
{exit_turn_feedback=stage<=3u?stage:3u;}

void ring_observe_maneuver_feedback(unsigned stage,int16_t turn10)
{maneuver_feedback=stage<=4u?stage:4u;maneuver_turn10=turn10;}
void ring_init(void)
{
    reentry_since=0;reentry_latched=0;
    ring_reset_episode();
}
void ring_reset_episode(void)
{
    int y;
    memset(&ring_result,0,sizeof(ring_result));
    tangent_outer_reset();
    ring_result.lane_width=RING_DEFAULT_LANE_WIDTH;
    ring_result.island_near=ring_result.near_x=ring_result.look_x=-1;
    ring_result.visual_dt_ms=65535u;
    for(y=0;y<RING_H;++y) ring_result.mid[y]=previous_mid[y]=previous_inner[y]=strong_inner[y]=-1;
    phase_since=ring_since=last_path_ms=last_path_fault_ms=cooldown_since=last_frame_ms=observe_since=vision_ms=0;
    tangent_force_since=0;maneuver_feedback=timed_started=lap_completed=visual_exit_count=exit_turn_feedback=0;maneuver_turn10=0;
    entry_yaw=entry_revs=cooldown_revs=confirm_yaw=confirm_revs=last_yaw=0;
    observe_travel=observe_last_revs=exit_slope=exit_offset=0;
    have_path=confirm_count=stable_count=exit_count=cancel_count=lost_frames=0;
    observing=observe_frames=observed_growth=observed_return=0;
    candidate_x=candidate_anchor=confirm_route=0;original_state=1;interval_supplied=0;
    trusted_steer=0;trusted_near=trusted_look=-1;
    entry_origin_row=entry_last_row=entry_progress_row=entry_high_row=-1;entry_changes=entry_measurements=0;
    support_frames=0;support_travel=support_last_revs=0;
    seed_left=seed_right=seed_x=seed_inner=-1;
    seed_identity_lost=0;
    assist_state=assist_side=assist_count=assist_candidate=assist_clock=0;
    assist_yaw=assist_revs=assist_total_yaw=assist_total_revs=0;
    tangent_slope=mouth_entry_slope=0;tangent_offset=mouth_entry_offset=RING_W/2;
    tangent_far_revs=tangent_lap_revs=0;
    tangent_near_close=tangent_gap=tangent_left_count=tangent_return_count=0;
    tangent_return_seen=0;
    memset(&mouth_near,0,sizeof(mouth_near));memset(&mouth_far,0,sizeof(mouth_far));
    memset(&mouth_near_closing,0,sizeof(mouth_near_closing));
    mouth_corridor_seen=mouth_ready=mouth_open_seen=0;mouth_signature=mouth_previous_signature=0;
    mouth_near_cleared=mouth_clear_count=mouth_pass_reason=0;
    mouth_bottom_count=0;mouth_bottom_since=0;
    mouth_far_source=0;mouth_portal_row=mouth_portal_x=-1;
    mouth_local_valid=mouth_local_changes=0;mouth_local_ms=mouth_local_curve_ms=0;
    exit_candidate_count=0;exit_candidate_since=exit_candidate_last_ms=exit_start_ms=0;
    exit_start_yaw=exit_start_revs=0;exit_track_valid=exit_net_reached=0;
    memset(&exit_candidate_path,0,sizeof(exit_candidate_path));
    memset(&exit_locked_path,0,sizeof(exit_locked_path));
    for(y=0;y<RING_H;++y) mouth_previous_lower[y]=-1;
    mouth_corridor_rows=mouth_corridor_clipped=0;
    mouth_previous_yaw=mouth_previous_revs=0;
    ring_result.mouth_near_x=ring_result.mouth_near_y=-1;
    ring_result.mouth_far_x=ring_result.mouth_far_y=ring_result.mouth_first_y=-1;
    ring_result.near_row=ring_result.far_row=-1;
    ring_result.guide_y=255u;ring_result.guide_x=-1;
}
static void reset_observation(void)
{
    int y;
    observing=observe_frames=observed_growth=observed_return=confirm_count=0;
    observe_travel=0;ring_result.score=ring_result.confirm_frames=0;
    if(!have_path) for(y=0;y<RING_H;++y) previous_mid[y]=previous_inner[y]=-1;
}
static void cancel_uncommitted(const ring_motion_t *m)
{
    int y;
    ring_result.timed_entry=ring_result.circle_observe_only=0;
    ring_result.stop=ring_result.fault=ring_result.path_valid=ring_result.branch_ok=0;
    ring_result.path_source=ring_result.committed=ring_result.exit_turn_request=0;
    ring_result.route=ring_result.entry_side=0;ring_result.steer_error=0;
    ring_result.near_x=ring_result.look_x=-1;
    exit_track_valid=exit_net_reached=0;exit_candidate_count=0;
    ring_result.exit_candidate=ring_result.exit_latched=0;
    for(y=0;y<RING_H;++y) ring_result.mid[y]=-1;
    cancel_count=have_path=0;reset_observation();
    assist_state=2;assist_count=0;
    seed_left=seed_right=seed_x=seed_inner=-1;
    cooldown_since=m->now_ms;cooldown_revs=m->wheel_revs;
    change_phase(RING_COOLDOWN,m->now_ms); /* no exit event/beep */
}

void ring_release_control(uint8_t reason,const ring_motion_t *m)
{
    int16_t turn10=(int16_t)fmaxf(-32768.f,fminf(32767.f,ring_result.turn_deg*10.f));
    cancel_uncommitted(m);
    ring_result.force_entry=0;ring_result.release_reason=reason;
    ring_result.left_entry=ring_result.lap_valid=0;
    ring_result.approach_stage=RING_TANGENT_NONE;
    ring_result.decision=RING_DECISION_RELEASE;ring_result.entry_turn10=turn10;
}

/* Confirmation/beeping is not physical entry. If branch identity cannot be
 * maintained before entry, return ownership to a still-valid ORIGINAL
 * controller, including ordinary curves/wide mouths. Its own lost-line
 * state must never be turned into permission to resume forward motion. */
static int can_cancel_uncommitted(void)
{
    return !ring_result.committed && (original_state==1 || original_state==2);
}

/* Only a NEW, identity-associated inner edge can advance entry history.
 * Wheel/yaw feedback cannot move this landmark or create its observations. */
static void remember_entry_geometry(void)
{
    int y,nearest=-1;
    for(y=24;y<=117;++y) if(inner[y]>2 && inner[y]<RING_W-3) nearest=y;
    if(nearest<0) return;
    if(entry_origin_row<0 || nearest<entry_high_row-8) {
        entry_origin_row=entry_last_row=entry_progress_row=entry_high_row=nearest;
        entry_changes=entry_measurements=0;
    }
    else {
        /* Count cumulative pixel progress relative to the last credited
         * high-water mark. Updating that mark every frame loses genuine
         * 1px/frame motion; integrating signed deltas credits jitter twice. */
        if(nearest>=entry_progress_row+2) {++entry_changes;entry_progress_row=nearest;}
        if(nearest>entry_high_row) entry_high_row=nearest;
        entry_last_row=nearest;
    }
    ++entry_measurements;
}

static int entry_geometry_progressed(void)
{
    return entry_measurements>=3 && entry_changes>=2 &&
           entry_last_row-entry_origin_row>=8;
}

static uint16_t hold_rejections(const ring_motion_t *m)
{
    uint16_t flags=0;
    if(!have_path) flags|=RING_HOLD_NO_HISTORY;
    if(ring_result.visual_dt_ms==65535u) flags|=RING_HOLD_UNKNOWN_DT;
    if(vision_ms-last_path_ms>RING_PATH_HOLD_MS) flags|=RING_HOLD_TIME;
    if(support_frames>(int)RING_HOLD_FRAMES) flags|=RING_HOLD_FRAMES_REJECT;
    if(support_travel>RING_HOLD_REVS) flags|=RING_HOLD_TRAVEL;
    if(fabsf(m->yaw_deg-last_yaw)>RING_HOLD_YAW_DEG) flags|=RING_HOLD_YAW;
    return flags;
}

static int assist_history_fresh(const ring_motion_t *m)
{
    return assist_state==1 && ring_result.phase==RING_ENTER && !ring_result.committed &&
        ring_result.visual_dt_ms!=65535u && m->now_ms-phase_since<=RING_ASSIST_TOTAL_MS &&
        assist_total_yaw+fabsf(m->yaw_deg-assist_yaw)<=RING_ASSIST_ENTRY_YAW_DEG &&
        assist_total_revs+fabsf(m->wheel_revs-assist_revs)<=RING_ASSIST_ENTRY_REVS &&
        vision_ms-last_path_ms<=RING_ASSIST_HOLD_MS &&
        fabsf(m->yaw_deg-last_yaw)<=RING_ASSIST_HOLD_YAW_DEG &&
        support_frames<=(int)RING_HOLD_FRAMES &&
        support_travel+fabsf(m->wheel_revs-support_last_revs)<=RING_ASSIST_HOLD_REVS;
}

/* The previously accepted path must still lie inside one CURRENT connected
 * white corridor. Failed construction's new target/inner arrays are not an
 * identity source. No pixels are rescanned and a blank image cannot pass. */
static int assist_corridor(void)
{
    int y,j,last_l=-1,last_r=-1;
    for(y=117;y>=RING_LOOK_ROW;--y) {
        int pick=-1,x=previous_mid[y];
        if(x<0) return 0;
        for(j=0;j<row_count[y];++j) {
            run_t r=row_runs[y][j];
            if(connected[y][j] && x>=r.left+2 && x<=r.right-2 &&
               (last_l<0 || (r.left<=last_r+RING_CONNECT_GAP &&
                             r.right>=last_l-RING_CONNECT_GAP))) {pick=j;break;}
        }
        if(pick<0) return 0;
        last_l=row_runs[y][pick].left;last_r=row_runs[y][pick].right;
    }
    return 1;
}

/* This is control assistance, never new visual evidence. Only two actual
 * ENTER measurements choose its sign; route is the lap side, not its heading.
 * Once spent, this acquisition cannot re-arm even if strong pixels return. */
static int entry_assist(const ring_motion_t *m,int measured,int cross,int ordinary)
{
    int sign=0;
    if(ring_result.phase!=RING_ENTER) return 0;
    if(!assist_clock) {
        assist_clock=1;assist_yaw=entry_yaw;assist_revs=entry_revs;
    }
    assist_total_yaw+=fabsf(m->yaw_deg-assist_yaw);
    assist_total_revs+=fabsf(m->wheel_revs-assist_revs);
    assist_yaw=m->yaw_deg;assist_revs=m->wheel_revs;
    if(assist_state==2) return 0;
    if(ring_result.visual_dt_ms==65535u || m->now_ms-phase_since>RING_ASSIST_TOTAL_MS ||
       assist_total_yaw>RING_ASSIST_ENTRY_YAW_DEG || assist_total_revs>RING_ASSIST_ENTRY_REVS) {
        assist_state=2;assist_count=0;
        ring_result.decision=RING_DECISION_ASSIST_EXPIRED;return 0;
    }
    if(seed_identity_lost || cross || ordinary || (!measured && !assist_corridor())) {
        if(assist_state==1) ring_result.decision=RING_DECISION_ASSIST_BLOCKED;
        assist_state=2;assist_count=0;return 0;
    }
    if(ring_result.committed) {
        assist_state=2;assist_count=0;return 0;
    }
    if(measured==1) {
        int e=ring_result.steer_error;
        if(abs(e)>=RING_ASSIST_TARGET_ERROR && ring_result.guide_x>=0 &&
           abs(ring_result.guide_x-RING_W/2)>=RING_ASSIST_TARGET_ERROR &&
           e*(ring_result.guide_x-RING_W/2)>0) sign=e>0?1:-1;
        if(assist_state==1 && sign!=assist_side) {
            assist_state=2;assist_count=0;
            ring_result.decision=RING_DECISION_ASSIST_RELEASE;return 0;
        }
        if(assist_state==0) {
            assist_count=sign && sign==assist_candidate?assist_count+1:(sign?1:0);
            assist_candidate=sign;
            if(assist_count<RING_ASSIST_CONFIRM) return 0;
            assist_side=sign;assist_state=1;
        }
    } else {
        if(assist_state!=1) {assist_count=0;assist_candidate=0;return 0;}
        if(!assist_history_fresh(m)) {
            assist_state=2;ring_result.decision=RING_DECISION_ASSIST_EXPIRED;return 0;
        }
    }
    if(assist_state!=1) return 0;
    if(ring_result.steer_error*assist_side<RING_ASSIST_MIN_ERROR)
        ring_result.steer_error=(int16_t)(assist_side*RING_ASSIST_MIN_ERROR);
    ring_result.branch_mode=RING_BRANCH_ASSIST;
    ring_result.decision=RING_DECISION_ASSIST;
    ring_result.entry_flags|=(uint16_t)(RING_ENTRY_ASSIST |
        (assist_side<0?RING_ENTRY_ASSIST_LEFT:0));
    if(!measured) {
        ring_result.path_source=5;ring_result.path_valid=0;
        ring_result.branch_ok=0;ring_result.stop=0;
    }
    return 1;
}

/* Fit only cached measured edge points, never white-run midpoints. A short
 * local regression suppresses the 3px derivative spikes near an island tip. */
static int inner_fit(const int16_t *edge,int first,int last,float *slope,float *offset)
{
    int y,n=0;
    float sy=0,sx=0,syy=0,syx=0,den;
    first=clamp(first,24,117);last=clamp(last,24,117);
    for(y=first;y<=last;++y) if(edge[y]>2 && edge[y]<RING_W-3) {
        ++n;sy+=y;sx+=edge[y];syy+=y*y;syx+=y*edge[y];
    }
    den=n*syy-sy*sy;
    if(n<4 || den<=0) return 0;
    *slope=(n*syx-sy*sx)/den;*offset=(sx-*slope*sy)/n;
    return 1;
}

static int inner_support(int y,int radius)
{
    int k,n=0;
    for(k=y-radius;k<=y+radius;++k)
        if(k>=24 && k<=117 && inner[k]>2 && inner[k]<RING_W-3) ++n;
    return n>=4;
}

/* Partial views may update control for a few frames, but only while a
 * previous STRONG edge still authorises this same local measured segment.
 * They cannot renew their own time/frame/yaw/travel allowance. */
static int partial_inner(const island_t *island,int cross,int ordinary,const ring_motion_t *m)
{
    int y,start=-1,near_fit=0,guide_fit=0,segments=0;
    if(!have_path || cross || ordinary || hold_rejections(m) ||
       ring_result.phase<RING_APPROACH || ring_result.phase>RING_CIRCLE) return 0;
    for(y=24;y<=118;++y) {
        if(y<=117 && inner[y]>2 && inner[y]<RING_W-3 && strong_inner[y]>2 && strong_inner[y]<RING_W-3) {
            if(start<0) start=y;
        } else if(start>=0) {
            int end=y-1,k,n=end-start+1,bad=0;
            float residual=0,cs,ci,ps,pi;
            if(n>=12) {
                int middle=(start+end)/2;
                float t=(float)(middle-start)/(end-start);
                float chord=inner[start]*(1.0f-t)+inner[end]*t;
                /* Every control-supporting segment must preserve identity;
                 * checking only the longest near segment misses a switched
                 * or reverse-curving guide segment farther up the image. */
                if(ring_result.route*(inner[middle]-chord)<=-3.0f ||
                   !inner_fit(inner,start,end,&cs,&ci) ||
                   !inner_fit(strong_inner,start,end,&ps,&pi) || fabsf(cs-ps)>0.5f) return 0;
                for(k=start;k<=end;++k) {
                    int error=abs(inner[k]-strong_inner[k]);
                    residual+=error;if(error>RING_CORRIDOR_TOL/2) ++bad;
                    if(fabsf(inner[k]-(cs*k+ci))>4.0f) return 0;
                }
                if(residual/n>RING_CORRIDOR_TOL/2 || bad*4>n) return 0;
                ++segments;
                if(RING_NEAR_ROW>=start-4 && RING_NEAR_ROW<=end+4) near_fit=1;
                if(ring_result.guide_y>=start && ring_result.guide_y<=end) guide_fit=1;
            }
            start=-1;
        }
    }
    if(!segments || !guide_fit) return 0;
    /* Both control samples must be supported by this CURRENT edge or a
     * fitted continuation of at most four image rows. */
    if(!near_fit &&
       !(island->valid && island->bridge && island->bottom<RING_NEAR_ROW)) return 0;
    if(!inner_support(ring_result.guide_y,4)) return 0;
    return 1;
}

/* Curvature is measured on the identity-selected INNER edge, not any white
 * run midpoint. Straight road can be path_ok but cannot refresh branch_ok. */
static int curved_inner(void)
{
    int y,start=-1,end=-1,best_start=-1,best_end=-1;
    for(y=24;y<=117;++y) {
        if(inner[y]>2 && inner[y]<RING_W-3) {
            if(start<0) start=y;
            end=y;
            if(best_start<0 || end-start>best_end-best_start) {best_start=start;best_end=end;}
        } else start=-1;
    }
    if(best_start<0 || best_end-best_start<24) return 0;
    /* A cropped arc need not have its greatest bow at the middle of the
     * longest span. Test local chords of the SAME selected inner edge. */
    for(start=best_start;start+24<=best_end;start+=4)
        for(end=start+24;end<=best_end;end+=4) {
            int middle=(start+end)/2;
            float t=(float)(middle-start)/(end-start);
            float straight=inner[start]*(1.0f-t)+inner[end]*t;
            if(ring_result.route*(inner[middle]-straight)>=3.0f) return 1;
        }
    return 0;
}

static int build_path(const island_t *island,int structure,int exiting,
                      int cross,int ordinary,const ring_motion_t *m)
{
    int y,last_x=RING_W/2,last_y=117,rows=0,near=-1,look=-1;
    int associated=0,residual=0,bad_association=0,split_rows=0;
    int anchor=have_path || confirm_count>0;
    int look_associated=0,look_bad=0,near_associated=0,near_bad=0;
    int route=ring_result.route;
    int guide_y=RING_LOOK_ROW,guide=-1,edge_rows=0,strong=1,near_ref=RING_W/2;
    int island_anchor=island->valid && (structure ||
        (have_path && entry_geometry_progressed() && island->bridge &&
         (island->growing || island->returning)));
    int tangent_seed=RING_TRACK_MODE && ring_result.phase==RING_ENTER &&
        !ring_result.committed && (ring_result.approach_stage==RING_TANGENT_FAR_READY ||
                                  ring_result.approach_stage==RING_TANGENT_LEFT);
    for(y=0;y<RING_H;++y) {
        ring_result.mid[y]=lane_left[y]=lane_right[y]=inner[y]=-1;
    }
    for(y=117;y>=24;--y) {
        int j,pick=-1,score=10000,x=last_x;
        int expected=exiting?clamp((int)(exit_slope*y+exit_offset),0,RING_W-1):
            (!tangent_seed && previous_mid[y]>=0?previous_mid[y]:last_x);
        for(j=0;j<row_count[y];++j) {
            int l=row_runs[y][j].left,r=row_runs[y][j].right,d;
            if(!connected[y][j]) continue;
            if(!exiting && island_anchor && gap_left[y]>=0 && y>=island->top && y<=island->bottom) {
                if((route>0 && l<gap_right[y]) || (route<0 && r>gap_left[y])) continue;
            }
            d=expected<l?l-expected:(expected>r?expected-r:0);
            if(!exiting && !tangent_seed && anchor && previous_inner[y]>2 && previous_inner[y]<RING_W-3) {
                int e=route>0?l:r;
                /* A temporarily clipped edge is unknown, not a different
                 * branch. Keep scanning the connected white corridor; the
                 * measured support/partial-lifetime checks decide below. */
                if(e>2 && e<RING_W-3) d+=abs(e-previous_inner[y]);
            }
            if(d<score) {score=d;pick=j;}
        }
        if(pick<0 || (rows && score>RING_MAX_ROW_STEP*(last_y-y+1))) break;
        {
            int l=row_runs[y][pick].left,r=row_runs[y][pick].right;
            int w=r-l+1,offset=clamp(w/2,3,ring_result.lane_width/2);
            if(rows && (l>lane_right[last_y]+RING_CONNECT_GAP ||
                        r<lane_left[last_y]-RING_CONNECT_GAP)) break;
            lane_left[y]=(int16_t)l;lane_right[y]=(int16_t)r;
            inner[y]=(int16_t)(route>0?l:r);
            if(island_anchor) {
                if(gap_left[y]>=0 && y>=island->top && y<=island->bottom) ++split_rows;
                else inner[y]=-1; /* inlet/upper cap is NOT the island edge */
            }
            if(previous_inner[y]>2 && previous_inner[y]<RING_W-3 &&
               inner[y]>2 && inner[y]<RING_W-3) {
                int error=abs(inner[y]-previous_inner[y]);
                residual+=error;++associated;
                if(error>RING_CORRIDOR_TOL) ++bad_association;
                if(y>=RING_LOOK_ROW-6 && y<=RING_LOOK_ROW+6) {
                    ++look_associated;if(error>RING_CORRIDOR_TOL) ++look_bad;
                }
                if(y>=RING_NEAR_ROW-6 && y<=RING_NEAR_ROW+6) {
                    ++near_associated;if(error>RING_CORRIDOR_TOL) ++near_bad;
                }
            }
            /* Provisional points only guide the bottom-up run selection.
             * Final targets below use the complete measured edge fit. */
            if(!exiting && inner[y]>=0 && y+3<RING_H && inner[y+3]>=0) {
                float slope=(inner[y]-inner[y+3])/3.0f;
                offset=clamp((int)(ring_result.lane_width*0.5f*sqrtf(1.0f+slope*slope)+0.5f),3,w-3);
            }
            x=route>0?l+offset:r-offset;
            if(exiting) x=clamp(expected,l+2,r-2);
            else if(island->valid && y>island->bottom) {
                int target=route>0?island->right+1+ring_result.lane_width/2:
                                    island->left-1-ring_result.lane_width/2;
                if(y>=RING_NEAR_ROW || island->bottom>=RING_NEAR_ROW) x=(l+r)/2;
                else {
                    float t=(float)(RING_NEAR_ROW-y)/(RING_NEAR_ROW-island->bottom);
                    t=t*t*(3.0f-2.0f*t);x=near_ref+(int)((target-near_ref)*t);
                }
            }
            x=clamp(x,l+2,r-2);
            ring_result.mid[y]=(int16_t)x;
            if(y==RING_NEAR_ROW) near_ref=x;
            last_x=x;last_y=y;++rows;
        }
    }
    if(!exiting && island_anchor && island->bottom-4<RING_LOOK_ROW)
        guide_y=clamp(island->bottom-4,24,RING_LOOK_ROW);
    if(!exiting && island_anchor && guide_y<island->top+4)
        guide_y=clamp(island->top+4,24,RING_NEAR_ROW-4);
    /* A measured normal, not a derivative of noisy three-pixel steps. */
    for(y=117;y>=24;--y) if(lane_left[y]>=0) {
        int l=lane_left[y],r=lane_right[y],w=r-l+1;
        int offset=clamp(w/2,3,ring_result.lane_width/2),x;
        float slope,intercept;
        if(inner[y]>2 && inner[y]<RING_W-3) {
            ++edge_rows;
            if(inner_fit(inner,y-4,y+4,&slope,&intercept))
                offset=clamp((int)(ring_result.lane_width*0.5f*sqrtf(1.0f+slope*slope)+0.5f),3,w-3);
        }
        x=route>0?l+offset:r-offset;
        if(exiting) x=clamp((int)(exit_slope*y+exit_offset),l+2,r-2);
        else if(island_anchor && y>island->bottom && y>=RING_NEAR_ROW) x=(l+r)/2;
        if(!exiting && !tangent_seed && have_path && inner[y]>2 && inner[y]<RING_W-3 &&
           previous_inner[y]>2 && previous_inner[y]<RING_W-3 && previous_mid[y]>=0) {
            int predicted=previous_mid[y]+inner[y]-previous_inner[y];
            /* Bound a normal-offset jump after compensating its measured
             * edge movement. The result stays on this same white run. */
            x=clamp(x,predicted-RING_MAX_ROW_STEP,predicted+RING_MAX_ROW_STEP);
        }
        ring_result.mid[y]=(int16_t)clamp(x,l+2,r-2);
    }
    /* Below a still-visible island tip there is no inner boundary. Join
     * the near incoming road to an ACTUALLY measured branch guide, rather
     * than treating the tip as a vertical edge or choosing a straight lane. */
    if(!exiting && island_anchor && island->bottom<RING_NEAR_ROW &&
       ring_result.mid[guide_y]>=0 && lane_left[RING_NEAR_ROW]>=0) {
        int start=(lane_left[RING_NEAR_ROW]+lane_right[RING_NEAR_ROW])/2;
        int target=ring_result.mid[guide_y];
        for(y=RING_NEAR_ROW;y>island->bottom;--y) if(lane_left[y]>=0) {
            float t=(float)(RING_NEAR_ROW-y)/(RING_NEAR_ROW-guide_y);
            int x=start+(int)((target-start)*t);
            ring_result.mid[y]=(int16_t)clamp(x,lane_left[y]+2,lane_right[y]-2);
        }
    }
    near=ring_result.mid[RING_NEAR_ROW];look=ring_result.mid[RING_LOOK_ROW];
    guide=ring_result.mid[guide_y];
    ring_result.inner_rows=(uint8_t)edge_rows;
    ring_result.associated_rows=(uint8_t)associated;
    ring_result.guide_y=(uint8_t)guide_y;ring_result.guide_x=(int16_t)guide;
    ring_result.target_support=(uint8_t)((inner_support(RING_NEAR_ROW,4)?1:0) |
        (inner_support(RING_LOOK_ROW,4)?2:0) | (inner_support(guide_y,4)?4:0));
    ring_result.path_valid=(uint8_t)(rows>=35 && near>=0 && look>=0 &&
        guide>=0 && guide_y<RING_NEAR_ROW);
    ring_result.near_x=(int16_t)near;ring_result.look_x=(int16_t)look;
    if(!ring_result.path_valid) {ring_result.guide_x=-1;ring_result.guide_y=255u;return 0;}
    /* Acquisition and maintenance use the SAME minimum visible anchor.
     * A distant 8..19-row island stays a candidate; it must not beep/take
     * control and immediately reject itself on the next calculation. */
    if(!exiting && island_anchor && split_rows<RING_BRANCH_ROWS) {
        ring_result.reject|=RING_REJ_SHORT;strong=0;
    }
    if(!exiting && !(island_anchor && split_rows>=RING_BRANCH_ROWS)) {
        if((!have_path || associated<RING_BRANCH_ROWS) && !tangent_seed) {
            ring_result.reject|=RING_REJ_ASSOC;strong=0;
        }
    }
    /* Acquisition still needs a curved island. An established entry may
     * show a locally tangent part only while that SAME island is measured
     * now and its edge remains associated; a plain white lane cannot use it. */
    if(!exiting && !curved_inner() &&
       !(have_path && entry_geometry_progressed() && island_anchor &&
         split_rows>=RING_BRANCH_ROWS && associated>=RING_BRANCH_ROWS)) {
        ring_result.reject|=RING_REJ_CURVATURE;strong=0;
    }
    /* An already acquired branch must also remain in its corridor. */
    if(!exiting && anchor && !tangent_seed) {
        if(associated<RING_BRANCH_ROWS && !tangent_seed) ring_result.reject|=RING_REJ_ASSOC;
        else if(associated>=RING_BRANCH_ROWS &&
                (residual/associated>RING_CORRIDOR_TOL || bad_association*4>associated))
            ring_result.reject|=RING_REJ_RESIDUAL;
        if(look_associated>=4 && look_bad*2>look_associated) ring_result.reject|=RING_REJ_LOOK;
        if(near_associated>=4 && near_bad*2>near_associated) ring_result.reject|=RING_REJ_LOOK;
        /* A large target change needs the measured inner edge at that
         * target as support. Normal-distance offsets can move faster than
         * pixels on a real arc, so do not reject them by target delta alone. */
        if(have_path && !island_anchor &&
           ((trusted_near>=0 && abs(near-trusted_near)>RING_CORRIDOR_TOL && near_associated<4) ||
            (trusted_look>=0 && abs(look-trusted_look)>RING_CORRIDOR_TOL && look_associated<4)))
            ring_result.reject|=RING_REJ_LOOK;
        if(ring_result.reject&(RING_REJ_ASSOC|RING_REJ_RESIDUAL|RING_REJ_LOOK)) {
            ring_result.reject|=RING_REJ_CORRIDOR;strong=0;
        }
    }
    if(!exiting && !(ring_result.target_support&4u)) strong=0;
    if(!strong && !partial_inner(island,cross,ordinary,m)) {
        ring_result.guide_x=-1;ring_result.guide_y=255u;return 0;
    }
    ring_result.branch_mode=(uint8_t)(exiting?RING_BRANCH_EXIT:(strong?RING_BRANCH_STRONG:RING_BRANCH_PARTIAL));
    /* Normalise the actual guide's depth to the original 30-row horizon.
     * This makes an early island visible to steering while row78 is still
     * on the broad incoming bridge. No route-dependent steering is added. */
    ring_result.steer_error=(int16_t)clamp((near-RING_W/2-1)+
        (guide-near)*(RING_NEAR_ROW-RING_LOOK_ROW)*2/(3*(RING_NEAR_ROW-guide_y)),-90,90);
    return strong?1:2;
}

int16_t ring_blend_error(int16_t normal_error)
{
    if(ring_result.timed_entry) {
        if(ring_result.phase==RING_APPROACH)
            return ring_result.path_valid?ring_result.steer_error:normal_error;
        return 0; /* Only finite entry turn owns the gyro interval. */
    }
    if(!ring_owns_path()) return normal_error;
    if(ring_result.phase==RING_RECOVER)
        return (int16_t)((ring_result.steer_error*(RING_RECOVER_FRAMES-stable_count)+normal_error*stable_count)/RING_RECOVER_FRAMES);
    return ring_result.steer_error;
}
int ring_turn_boost(void)
{
    int limit,boost;
    if(ring_result.timed_entry || ring_result.exit_turn_request)return 0;
    if(!ring_owns_path() || ring_result.stop) return 0;
    limit=ring_result.phase<=RING_ENTER?RING_ENTRY_BOOST_RPM:RING_CIRCLE_BOOST_RPM;
    boost=clamp(ring_result.steer_error*3/5,-limit,limit);
    if(ring_result.phase==RING_RECOVER) boost=boost*(RING_RECOVER_FRAMES-stable_count)/RING_RECOVER_FRAMES;
    return boost;
}
static int exit_opening(void)
{
    int y;
    /* Locate the opening first, then inspect its neighbouring lane segments.
     * This does not require the mouth to land in exactly rows 55..90, nor a
     * 1.5x lane-width threshold. Lap + displacement gates still apply below. */
    for (y = 32; y <= 102; ++y) {
        int top = y, bottom, far = 0, near = 0, k;
        while (y <= 104) {
            int l = lane_left[y], r = lane_right[y];
            if (l < 0 || r - l + 1 <= ring_result.lane_width * 5 / 4 ||
                (ring_result.route > 0 ? l <= 2 : r >= RING_W - 3)) break;
            ++y;
        }
        bottom = y - 1;
        if (bottom - top + 1 < 6) continue;
        for (k = top - 1; k >= 24 && k >= top - 24; --k) {
            int l = lane_left[k], r = lane_right[k], w = r - l + 1;
            if (l > 1 && r < RING_W - 2 && w >= ring_result.lane_width / 2 &&
                w <= ring_result.lane_width * 4 / 3) ++far;
        }
        for (k = bottom + 1; k <= 117 && k <= bottom + 24; ++k) {
            int l = lane_left[k], r = lane_right[k], w = r - l + 1;
            if (l > 1 && r < RING_W - 2 && w >= ring_result.lane_width * 2 / 3 &&
                w <= ring_result.lane_width * 4 / 3) ++near;
        }
        if (far >= 6 && near >= 6) return 1;
    }
    return 0;
}

static int ordinary_lane(void)
{
    int y, count = 0;
    for (y = 60; y <= RING_NEAR_ROW; ++y) {
        int l = lane_left[y], r = lane_right[y], w = r - l + 1;
        if (l > 1 && r < RING_W - 2 && w >= ring_result.lane_width * 2 / 3 &&
            w <= ring_result.lane_width * 4 / 3) ++count;
    }
    return count >= 38;
}

/* Follow the CURRENT opposite boundary row by row. A near straight fit
 * cannot be extrapolated through a curved/camera-distorted whole image.
 * A clipped opposite boundary is missing evidence, not an invented line. */
static int tangent_reference(void)
{
    int y,j,last_l=-1,last_r=-1,last_edge=-1,rows=0,near_rows=0;
    for(y=0;y<RING_H;++y)
        mouth_centre[y]=mouth_outside[y]=mouth_left[y]=mouth_right[y]=-1;
    for(y=117;y>=8;--y) {
        int pick=-1,best=10000;
        for(j=0;j<row_count[y];++j) {
            run_t r=row_runs[y][j];
            int edge=ring_result.ring_side>0?r.left:r.right;
            int score;
            if(!connected[y][j] || edge<=2 || edge>=RING_W-3 ||
               r.right-r.left+1<ring_result.lane_width/2) continue;
            if(last_edge<0) {
                if(RING_W/2<r.left || RING_W/2>r.right) continue;
                score=abs(edge-(RING_W/2-ring_result.ring_side*ring_result.lane_width/2));
            } else {
                if(abs(edge-last_edge)>RING_MAX_ROW_STEP ||
                   r.left>last_r+RING_CONNECT_GAP || r.right<last_l-RING_CONNECT_GAP) continue;
                score=abs(edge-last_edge);
            }
            if(score<best) {best=score;pick=j;}
        }
        if(pick<0) break;
        mouth_left[y]=row_runs[y][pick].left;mouth_right[y]=row_runs[y][pick].right;
        mouth_outside[y]=(int16_t)(ring_result.ring_side>0?mouth_left[y]:mouth_right[y]);
        last_edge=mouth_outside[y];last_l=mouth_left[y];last_r=mouth_right[y];
        ++rows;if(y>=98) ++near_rows;
    }
    for(y=8;y<=117;++y) if(mouth_outside[y]>=0) {
        float slope=0,offset;
        if(inner_fit(mouth_outside,y-4,y+4,&slope,&offset) && fabsf(slope)>1.5f)
            slope=slope>0?1.5f:-1.5f;
        mouth_centre[y]=(int16_t)(mouth_outside[y]+ring_result.ring_side*
            ring_result.lane_width*0.5f*sqrtf(1.0f+slope*slope));
    }
    ring_result.mouth_ref=(uint8_t)(rows>=35 && near_rows>=12 &&
        mouth_centre[RING_NEAR_ROW]>=0 && mouth_centre[RING_LOOK_ROW]>=0);
    if(ring_result.mouth_ref) {
        float slope=0,offset=RING_W/2;
        if(inner_fit(mouth_centre,98,117,&slope,&offset)) {
            tangent_slope=slope;tangent_offset=offset;
        }
    }
    return ring_result.mouth_ref;
}
static int tangent_same_island(const island_t *island)
{
    return island->valid && island->growing && island->returning &&
        ring_result.ring_side*(island->center-(tangent_slope*island->bottom+tangent_offset))>0;
}
static int tangent_near_open(void)
{
    int y,n=0;
    for(y=94;y<=117;++y) {
        int edge=ring_result.ring_side>0?observed_right[y]:observed_left[y];
        float centre=mouth_centre[y];
        if(centre<0) continue;
        if(edge>=0 && ring_result.ring_side*(edge-centre)>ring_result.lane_width*5/6) ++n;
    }
    return n>=6;
}

/* F is a closing SIDE MOUTH: expanded current through-run below, an
 * aligned narrow through-run above. This is not a second island, nor a
 * dark-gap minimum-width proxy. All observations reuse the cached graph. */
static int tangent_far_closure(int *x)
{
    int y,state[RING_H],expanded=0;
    int selected=-1;
    mouth_closure_count=0;
    for(y=0;y<RING_H;++y) state[y]=0;
    for(y=8;y<=117;++y) {
        int l=mouth_left[y],r=mouth_right[y],centre=mouth_centre[y];
        int half=ring_result.ring_side*(centre-mouth_outside[y]);
        int width=r-l+1;
        if(l<0 || centre<l || centre>r || half<3) continue;
        if(width>half*2+6) state[y]=2;
        else if(l>2 && r<RING_W-3 && width>=half*4/3 && width<=half*2+6) state[y]=1;
    }
    for(y=117;y>=12;--y) {
        if(state[y]==2) ++expanded;
        else {
            if(expanded>=8 && state[y]==1 && state[y-1]==1 && state[y-2]==1 && state[y-3]==1)
            {
                int edge=ring_result.ring_side>0?mouth_right[y]:mouth_left[y];
                if(mouth_closure_count<6) {
                    mouth_closure_x[mouth_closure_count]=edge;
                    mouth_closure_y[mouth_closure_count++]=y+1;
                }
                /* Retain all distinct closings. The farthest is a
                 * pending observation even while N occupies nearer rows. */
                *x=edge;selected=y+1;
            }
            expanded=0;
        }
    }
    return selected;
}
#ifdef RING_HOST_TEST
void ring_test_observe_mouth(const uint8_t image[RING_H][RING_W],int side,int lane)
{
    ring_motion_t motion={0,0,0,1};
    int x=-1,y;
    ring_init();ring_result.ring_side=(int8_t)side;
    ring_result.lane_width=(int16_t)lane;scan_geometry(image,&motion);
    tangent_reference();y=tangent_far_closure(&x);
    ring_result.mouth_far_x=(int16_t)x;ring_result.mouth_far_y=(int16_t)y;
}
#endif

/* Each target is inside a CURRENT connected white run. During waiting the
 * caller keeps ordinary control; at F only the target shifts towards the
 * measured side opening. This never supplies inner-island branch credit. */
static int tangent_line_path(int entering)
{
    int y,j,last_l=-1,last_r=-1,rows=0,guide=RING_LOOK_ROW;
    if(entering && mouth_far.valid && mouth_far.missed==0)
        guide=clamp(mouth_far.y+3,RING_LOOK_ROW,RING_NEAR_ROW-6);
    for(y=0;y<RING_H;++y) ring_result.mid[y]=lane_left[y]=lane_right[y]=inner[y]=-1;
    for(y=117;y>=24;--y) {
        int pick=-1,best=10000;
        float centre=mouth_centre[y];
        int target=(int)(centre+0.5f);
        if(centre<0) break;
        if(entering) {
            float t=(float)(117-y)/(117-guide);
            if(t>1.0f)t=1.0f;
            target+=(int)(ring_result.ring_side*ring_result.lane_width*0.5f*t);
        }
        for(j=0;j<row_count[y];++j) {
            run_t r=row_runs[y][j];
            int d=target<r.left?r.left-target:(target>r.right?target-r.right:0);
            if(!connected[y][j] || (last_l>=0 &&
               (r.left>last_r+RING_CONNECT_GAP || r.right<last_l-RING_CONNECT_GAP))) continue;
            if(d<best) {best=d;pick=j;}
        }
        if(pick<0 || best>RING_MAX_ROW_STEP) break;
        lane_left[y]=row_runs[y][pick].left;lane_right[y]=row_runs[y][pick].right;
        ring_result.mid[y]=(int16_t)clamp(target,lane_left[y]+2,lane_right[y]-2);
        last_l=lane_left[y];last_r=lane_right[y];++rows;
    }
    ring_result.near_x=ring_result.mid[RING_NEAR_ROW];
    ring_result.look_x=ring_result.mid[RING_LOOK_ROW];
    ring_result.guide_x=ring_result.mid[guide];ring_result.guide_y=(uint8_t)guide;
    ring_result.path_valid=(uint8_t)(rows>=35 && ring_result.near_x>=0 && ring_result.guide_x>=0);
    if(!ring_result.path_valid) {ring_result.guide_x=-1;ring_result.guide_y=255;return 0;}
    ring_result.steer_error=(int16_t)clamp((ring_result.near_x-RING_W/2-1)+
        (ring_result.guide_x-ring_result.near_x)*(RING_NEAR_ROW-RING_LOOK_ROW)*2/
        (3*(RING_NEAR_ROW-guide)),-90,90);
    if(entering && ring_result.ring_side*ring_result.steer_error<RING_ASSIST_MIN_ERROR)
        ring_result.steer_error=(int16_t)(ring_result.ring_side*RING_ASSIST_MIN_ERROR);
    return 1;
}
/* The old F width-closing is only a longitudinal observation. In a
 * foreshortened/wide current corridor its row alone cannot time a turn.
 * Require the SAME connected side edge to return outwards, then merge
 * into white above the island. Offset its last measured tangent in TWO
 * dimensions; a horizontal tangent must not make a row-wise x offset
 * diverge. The guide and every intervening target must be current white
 * in a connected run. No clamp across dark pixels, no branch credit.
 *
 * This is a bounded entry preview/assist, never a measured inner path.
 * N-before-F, independent F association/progress and lap gates still run.
 * A tapered ordinary corridor has no outward merge and cannot supply it. */
static int tangent_portal_path(void)
{
    int y,j,tip=-1,edge=-1,side=ring_result.ring_side;
    int gx,gy,goal_y,dy,last_x,last_l,last_r,rows=0,wide=0,width=0;
    float slope=0,offset=0,norm;
    int16_t boundary[RING_H],path[RING_H];
    mouth_portal_row=mouth_portal_x=-1;
    for(y=0;y<RING_H;++y) {
        boundary[y]=(int16_t)(side>0?mouth_right[y]:mouth_left[y]);
        path[y]=-1;
    }
    for(y=108;y<=117;++y) if(mouth_left[y]>2 && mouth_right[y]<RING_W-3) {
        width+=mouth_right[y]-mouth_left[y]+1;++wide;
    }
    wide=wide>=8 && width/wide>=ring_result.lane_width*7/4;
    for(y=RING_LOOK_ROW;y>=28;--y) {
        int e=boundary[y],above=boundary[y-1],k,min_edge=e,return_rows=0;
        int jump=side*(above-e);
        int clipped_above=side>0?above>=RING_W-3:above>=0 && above<=2;
        /* A shoulder that merges through the image side has only its
         * remaining visible pixels as a jump. Its following signed bow
         * and returning tangent still have to prove island geometry. */
        if(e<=2 || e>=RING_W-3 || above<0 || mouth_outside[y]<0 ||
           jump<(clipped_above && wide?3:ring_result.lane_width/3)) continue;
        /* Track back down this very edge. A lone white flash, fork or
         * abrupt unrelated boundary cannot replace the island shoulder. */
        for(k=y;k<=y+24 && k<=117;++k) {
            if(boundary[k]<=2 || boundary[k]>=RING_W-3 ||
               (k>y && abs(boundary[k]-boundary[k-1])>8)) break;
            if(side*(boundary[k]-min_edge)<0) min_edge=boundary[k];
            ++return_rows;
        }
        if(return_rows<16 || side*(e-min_edge)<ring_result.lane_width/6 ||
           !inner_fit(boundary,y,y+6,&slope,&offset) || side*slope>=-.25f) continue;
        tip=y;edge=e;break;
    }
    if(tip<0) return 0;
    /* Preserve the CURRENT feature even when no safe target fits yet.
     * Corner identity/progress must not depend on preview-path success. */
    mouth_portal_row=tip;mouth_portal_x=edge;
    norm=sqrtf(1.f+slope*slope);
    gx=(int)(edge-side*ring_result.lane_width*.5f/norm+.5f);
    gy=(int)(tip+side*ring_result.lane_width*.5f*slope/norm+.5f);
    if(gy<24 || gy>RING_LOOK_ROW || gx<3 || gx>RING_W-4 ||
       mouth_centre[gy]<0 || side*(gx-mouth_centre[gy])<ring_result.lane_width/4)
        return 0;
    /* A local preview starts at the actual vehicle, not at a stale learned
     * width's displaced through centre. Reject an obstacle instead of
     * silently projecting the requested portal back onto the straight. */
    goal_y=gy;
    for(dy=0;dy<=3;++dy) {
        gy=goal_y+dy;last_x=RING_W/2;last_l=last_r=-1;rows=0;
        for(y=0;y<RING_H;++y) path[y]=-1;
        for(y=117;y>=gy;--y) {
            int x=RING_W/2+(gx-RING_W/2)*(117-y)/(117-gy),pick=-1;
            for(j=0;j<row_count[y];++j) {
                run_t r=row_runs[y][j];
                if(connected[y][j] && x>=r.left+2 && x<=r.right-2 &&
                   (last_l<0 || (r.left<=last_r+RING_CONNECT_GAP &&
                                  r.right>=last_l-RING_CONNECT_GAP))) {pick=j;break;}
            }
            if(pick<0 || abs(x-last_x)>RING_MAX_ROW_STEP) break;
            path[y]=(int16_t)x;last_x=x;
            last_l=row_runs[y][pick].left;last_r=row_runs[y][pick].right;++rows;
        }
        if(y<gy) break;
    }
    if(dy>3 || rows<35 || path[RING_NEAR_ROW]<0) return 0;
    mouth_portal_row=tip;mouth_portal_x=edge;
    memcpy(ring_result.mid,path,sizeof(path));
    ring_result.near_x=path[RING_NEAR_ROW];ring_result.look_x=path[RING_LOOK_ROW];
    ring_result.guide_x=(int16_t)gx;ring_result.guide_y=(uint8_t)gy;
    ring_result.path_valid=1;
    ring_result.steer_error=(int16_t)clamp((ring_result.near_x-RING_W/2-1)+
        (gx-ring_result.near_x)*(RING_NEAR_ROW-RING_LOOK_ROW)*2/
        (3*(RING_NEAR_ROW-gy)),-90,90);
    if(side*ring_result.steer_error<RING_ASSIST_MIN_ERROR)
        ring_result.steer_error=(int16_t)(side*RING_ASSIST_MIN_ERROR);
    return 1;
}

/* Current near corridor scale can differ from the historical learned
 * width after camera attitude changes. Only fully observed near edges
 * may widen the F horizon, and only together with a measured returning
 * side portal. The ordinary 1:1 world retains its original F region. */
static int tangent_far_window(int fy,int *path_ok)
{
    int y,n=0,width=0,portal=0,depth=119-RING_TANGENT_FAR_MIN_ROW;
    *path_ok=0;
    if(fy<0 || fy>RING_TANGENT_FAR_MAX_ROW) return 0;
    for(y=108;y<=117;++y) {
        int l=mouth_left[y],r=mouth_right[y];
        if(l>2 && r<RING_W-3) {width+=r-l+1;++n;}
    }
    if(n>=8 && width/n>=ring_result.lane_width*3/2) {
        portal=tangent_portal_path();
        if(portal) {
            depth=clamp((width/n)*3/4,depth,85);
            /* The width closing can be a perspective neck above the
             * actual side portal. Time this exceptional handover from
             * the CURRENT returning shoulder used by the validated path,
             * keeping closing association/progress as independent gates. */
            fy=mouth_portal_row;
        }
    }
    if(!portal && fy>=RING_TANGENT_FAR_MIN_ROW) {
        int guide=clamp(fy+3,RING_LOOK_ROW,RING_NEAR_ROW-6);
        int e=ring_result.ring_side>0?mouth_right[guide]:mouth_left[guide];
        int c=mouth_centre[guide];
        /* A successful white path by itself does not prove a side turn. */
        if(c>=0 && e>=0 && ring_result.ring_side*(e-c)>=ring_result.lane_width*3/4 &&
           tangent_line_path(1) && ring_result.ring_side*(ring_result.guide_x-c)>=ring_result.lane_width/4)
            *path_ok=1;
    } else if(portal) *path_ok=1;
    /* On the second N->F traversal this is an EXIT observation. Restore
     * its through guide immediately; an entry preview must never leak
     * into shared-straight control while waiting for the exit gate. */
    if(ring_result.approach_stage==RING_TANGENT_SHARED) tangent_line_path(0);
    return 119-fy<=depth;
}

/* A valid white path is not necessarily the newly selected F branch.
 * Before departure, compare the CURRENT measured guide with the original
 * through corridor rotated by measured yaw. A through-lane fit must not
 * overwrite the side-opening target merely because build_path succeeded. */
/* Signed distance from the rotated incoming line. The old tan(yaw)
 * expression flipped at90deg and omitted rotation of the line offset.
 * No pixel/metre assumption: this is yaw-only geometry in the IPM plane. */
static float tangent_entry_distance(const ring_motion_t *m)
{
    float yaw=(m->yaw_deg-ring_result.lap_origin_yaw)*0.01745329252f;
    float a=mouth_entry_slope,b=a*119.f+mouth_entry_offset-RING_W/2;
    float x=ring_result.guide_x-RING_W/2,forward=119-ring_result.guide_y;
    return ring_result.ring_side*((cosf(yaw)-a*sinf(yaw))*x+
        (sinf(yaw)+a*cosf(yaw))*forward-b)/sqrtf(1.f+a*a);
}
static int tangent_entry_branch(const ring_motion_t *m)
{
    /* At the beginning of a confirmed F turn the current inner-edge
     * offset can still describe the incoming straight. Its displaced
     * learned through reference alone must not grant branch ownership
     * to a target pointing back out of this turn. After actual heading
     * progress, zero/corrective error is legitimate and is not rejected.
     * This is an entry-control consistency check, never lap evidence. */
    if(!ring_result.left_entry && ring_result.lap_deg<20.f &&
       ring_result.ring_side*ring_result.steer_error<RING_ASSIST_TARGET_ERROR) return 0;
    return ring_result.guide_x>=0 && ring_result.guide_y<RING_NEAR_ROW &&
        tangent_entry_distance(m)>=ring_result.lane_width/4;
}

/* During the F bend the far row can cross the opening and see the OUTER
 * road edge, while a nearer current segment still measures the island.
 * Never join those two identities into one inner fit. Acquire the curved
 * near island segment on two associated observations, then use its actual
 * nearer guide. A plain straight/cross edge has no signed island bow. */
static int tangent_entry_inner_path(void)
{
    int16_t edge[RING_H];
    int y,first=117,associated=0,moved=0,curved=0,guide;
    if(ring_result.phase!=RING_ENTER || ring_result.committed) return 0;
    for(y=0;y<RING_H;++y) edge[y]=-1;
    for(y=117;y>=24;--y) {
        int e=ring_result.route>0?observed_left[y]:observed_right[y];
        if(e<=2 || e>=RING_W-3 || observed_left[y]<0 ||
           (y<117 && abs(e-edge[y+1])>8)) break;
        edge[y]=(int16_t)e;first=y;
    }
    if(118-first<24 || first>RING_NEAR_ROW-4) {mouth_entry_inner_count=0;return 0;}
    for(y=first;y+24<=117;++y) {
        int mid=(y+117)/2;
        float t=(float)(mid-y)/(117-y);
        float chord=edge[y]*(1-t)+edge[117]*t;
        if(ring_result.route*(edge[mid]-chord)>=3.f) {curved=1;break;}
    }
    if(!curved) {mouth_entry_inner_count=0;return 0;}
    for(y=first;y<=117;++y) if(mouth_entry_inner_count && mouth_entry_inner[y]>2 &&
                              mouth_entry_inner[y]<RING_W-3) {
        int d=abs(edge[y]-mouth_entry_inner[y]);
        if(d>RING_MAX_ROW_STEP) {mouth_entry_inner_count=0;break;}
        ++associated;if(d)++moved;
    }
    if(!mouth_entry_inner_count || associated<20) mouth_entry_inner_count=1;
    else if(moved && mouth_entry_inner_count<255) ++mouth_entry_inner_count;
    memcpy(mouth_entry_inner,edge,sizeof(edge));
    if(mouth_entry_inner_count<2) return 0;
    guide=first+4;
    if(guide>RING_NEAR_ROW-4) return 0;
    for(y=0;y<RING_H;++y) ring_result.mid[y]=lane_left[y]=lane_right[y]=inner[y]=-1;
    for(y=117;y>=first;--y) {
        float slope,offset;
        int l=observed_left[y],r=observed_right[y],x;
        if(!inner_fit(edge,y-4<first?first:y-4,y+4,&slope,&offset)) return 0;
        lane_left[y]=(int16_t)l;lane_right[y]=(int16_t)r;inner[y]=edge[y];
        x=edge[y]+(int)(ring_result.route*ring_result.lane_width*.5f*sqrtf(1+slope*slope));
        ring_result.mid[y]=(int16_t)clamp(x,l+2,r-2);
    }
    ring_result.near_x=ring_result.mid[RING_NEAR_ROW];ring_result.look_x=ring_result.mid[RING_LOOK_ROW];
    ring_result.guide_y=(uint8_t)guide;ring_result.guide_x=ring_result.mid[guide];
    ring_result.inner_rows=(uint8_t)(118-first);ring_result.associated_rows=(uint8_t)associated;
    ring_result.target_support=5;ring_result.path_valid=1;
    ring_result.steer_error=(int16_t)clamp((ring_result.near_x-RING_W/2-1)+
        (ring_result.guide_x-ring_result.near_x)*(RING_NEAR_ROW-RING_LOOK_ROW)*2/
        (3*(RING_NEAR_ROW-guide)),-90,90);
    return 1;
}

/* A cropped F bend can show a locally tangent island edge while the
 * upper image sees a DIFFERENT outside/background edge. Use only the
 * continuous near edge, associated to the measured F identity under yaw.
 * Tangent support is bounded from its last signed curve; it cannot renew
 * itself forever and NEVER advances entry commitment or CIRCLE.
 * Return1=current signed curve; return2=associated current tangent. */
static int tangent_local_path(const ring_motion_t *m)
{
    int16_t edge[RING_H],predicted[RING_H];
    int y,first=117,n=0,associated=0,bad=0,moved=0,curved=0,normal;
    float angle=(m->yaw_deg-mouth_local_yaw)*.01745329252f;
    if(!mouth_local_valid ||
       !(ring_result.phase==RING_ENTER || (ring_result.phase==RING_CIRCLE &&
         ring_result.committed && ring_result.approach_stage!=RING_TANGENT_SHARED)) ||
       vision_ms-mouth_local_ms>RING_OBSERVE_MS ||
       vision_ms-mouth_local_curve_ms>RING_ASSIST_TOTAL_MS ||
       fabsf(m->wheel_revs-mouth_local_revs)>RING_ASSIST_ENTRY_REVS ||
       fabsf(m->yaw_deg-mouth_local_yaw)>12.f) return 0;
    for(y=0;y<RING_H;++y) edge[y]=predicted[y]=-1;
    for(y=117;y>=48;--y) {
        int e=ring_result.route>0?observed_left[y]:observed_right[y];
        if(e<=2 || e>=RING_W-3 || observed_left[y]<0 ||
           (y<117 && abs(e-edge[y+1])>8)) break;
        edge[y]=(int16_t)e;first=y;++n;
    }
    if(n<24 || first>94) return 0;
    /* Reproject previous CURRENT edge samples, not their target midpoint.
     * Up to two rows of rasterisation may be interpolated. Translation
     * has no calibrated px/metre, so a bounded residual remains required. */
    for(y=24;y<=117;++y) if(mouth_local_edge[y]>2) {
        float x=mouth_local_edge[y]-RING_W/2,f=119-y;
        int yy=(int)(119-(sinf(angle)*x+cosf(angle)*f)+.5f);
        int xx=(int)(RING_W/2+cosf(angle)*x-sinf(angle)*f+.5f);
        if(yy>=24 && yy<=117 && xx>2 && xx<RING_W-3) predicted[yy]=(int16_t)xx;
    }
    for(y=first;y<=117;++y) {
        int k,p=-1;
        for(k=y-2;k<=y+2;++k) if(k>=24 && k<=117 && predicted[k]>=0 &&
            (p<0 || abs(k-y)<abs(p-y))) p=k;
        if(p>=0) {
            int d=abs(edge[y]-predicted[p]);++associated;
            if(d>RING_CORRIDOR_TOL/2) ++bad;
            if(mouth_local_edge[y]>=0 && edge[y]!=mouth_local_edge[y]) ++moved;
        }
    }
    /* A continuous far extension can belong to the outside background
     * above the opening. Keep the longest associated CURRENT near inner
     * segment, discarding only that incompatible far prefix. The same
     * minimum length, residual, movement and history gates still apply. */
    while(bad*4>associated && first<94 && 118-first>24) {
        int k,p=-1;
        for(k=first-2;k<=first+2;++k) if(k>=24 && k<=117 && predicted[k]>=0 &&
            (p<0 || abs(k-first)<abs(p-first))) p=k;
        if(p>=0) {
            --associated;
            if(abs(edge[first]-predicted[p])>RING_CORRIDOR_TOL/2) --bad;
            if(mouth_local_edge[first]>=0 && edge[first]!=mouth_local_edge[first]) --moved;
        }
        edge[first++]=-1;
    }
    if(associated<20 || bad*4>associated || moved<4) return 0;
    n=118-first;
    for(y=first;y+24<=117;++y) {
        int mid=(y+117)/2;
        float chord=edge[y]+(edge[117]-edge[y])*(float)(mid-y)/(117-y);
        if(ring_result.route*(edge[mid]-chord)>=3.f) {curved=1;break;}
    }
    /* A perfectly straight, normal-width replacement cannot inherit F
     * from fabricated yaw. Both current boundaries must still describe
     * the selected wider bend, or this edge must have real signed bow. */
    if(!curved) {
        int wide=0,turn_tip=0,k;
        for(y=94;y<=117;++y) if(observed_left[y]>2 && observed_right[y]<RING_W-3 &&
            observed_right[y]-observed_left[y]+1>ring_result.lane_width*4/3) ++wide;
        /* A current associated edge can end at the selected side of the
         * image while the actual road has normal width. That clipped bend
         * is not an unrelated ordinary straight: require its outward tip,
         * actual measured lateral sweep and the SAME temporal edge above.
         * It remains return2, never curvature/commitment credit. */
        for(k=first-1;k>=first-3 && k>=48;--k) {
            int e=ring_result.route>0?observed_left[k]:observed_right[k];
            if(e<0 || -ring_result.route*(e-edge[first])<0) break;
            if((ring_result.route>0?e<=2:e>=RING_W-3) &&
               -ring_result.route*(edge[first]-edge[117])>=ring_result.lane_width/4)
                turn_tip=1;
        }
        if(wide<16 && !turn_tip) return 0;
    }
    /* Construct current edge identity before attempting a normal target.
     * The old row-wise offset could reject this very measured bend before
     * its geometrically correct two-dimensional target was considered. */
    for(y=0;y<RING_H;++y) inner[y]=-1;
    for(y=first;y<=117;++y) inner[y]=edge[y];
    normal=tangent_normal_path();
    if(normal && (normal==1 || (have_path && !hold_rejections(m)))) {
        for(y=0;y<RING_H;++y) {
            lane_left[y]=edge[y]>=0?observed_left[y]:-1;
            lane_right[y]=edge[y]>=0?observed_right[y]:-1;
        }
        ring_result.inner_rows=(uint8_t)n;ring_result.associated_rows=(uint8_t)associated;
        memcpy(mouth_local_edge,edge,sizeof(edge));mouth_local_ms=vision_ms;mouth_local_yaw=m->yaw_deg;
        if(curved && normal==1) mouth_local_curve_ms=vision_ms;
        ++mouth_local_changes;
        return curved && normal==1?1:2;
    }
    return 0;
}

/* At returning N the shared white road hides the distant inner edge. A
 * still-MEASURED, associated near inner edge can guide this known junction
 * at a nearer horizon; a straight/blank image cannot renew this source.
 * This is a current edge measurement, never a prolonged stale-path hold. */
static int tangent_return_path(void)
{
    int y,j,first=117,associated=0,opening=0,rows=0,last_l=-1,last_r=-1;
    int16_t edge[RING_H];
    float slope,offset;
    if(!ring_result.left_entry || !ring_result.committed ||
       ring_result.lap_deg<250.0f || ring_result.lap_deg>375.0f) return 0;
    for(y=0;y<RING_H;++y) edge[y]=-1;
    for(y=117;y>=24;--y) {
        int l=observed_left[y],r=observed_right[y];
        int e=ring_result.route>0?l:r;
        if(l>=0 && e>2 && e<RING_W-3) edge[y]=(int16_t)e;
        if(y>=50 && y<=RING_LOOK_ROW && l>=0 &&
           r-l+1>ring_result.lane_width*3/2) ++opening;
    }
    if((opening<6 && !tangent_return_seen) || edge[RING_NEAR_ROW]<0 || edge[117]<0) return 0;
    while(first>88 && edge[first-1]>=0 && abs(edge[first-1]-edge[first])<=8) --first;
    /* Use the longest CURRENT local segment that is actually straight
     * enough to fit, shortening the horizon as the junction masks it. */
    for(;first<=RING_NEAR_ROW-4;++first) {
        int bad=0;
        if(!inner_fit(edge,first,117,&slope,&offset)) continue;
        for(y=first;y<=117;++y)
            if(fabsf(edge[y]-(slope*y+offset))>4.0f) {bad=1;break;}
        if(!bad) break;
    }
    if(first>RING_NEAR_ROW-4 || 118-first<12) return 0;
    for(y=first;y<=117;++y) {
        if(fabsf(edge[y]-(slope*y+offset))>4.0f) return 0;
        if(previous_inner[y]>2 && previous_inner[y]<RING_W-3) {
            if(abs(edge[y]-previous_inner[y])>RING_MAX_ROW_STEP) return 0;
            ++associated;
        }
    }
    if(associated<8) return 0;
    tangent_return_seen=1;
    for(y=0;y<RING_H;++y) ring_result.mid[y]=lane_left[y]=lane_right[y]=inner[y]=-1;
    for(y=117;y>=first;--y) {
        int pick=-1,e=edge[y];
        for(j=0;j<row_count[y];++j) {
            run_t r=row_runs[y][j];
            if(connected[y][j] && (ring_result.route>0?r.left:r.right)==e &&
               (last_l<0 || (r.left<=last_r+RING_CONNECT_GAP && r.right>=last_l-RING_CONNECT_GAP))) {
                pick=j;break;
            }
        }
        if(pick<0) return 0;
        lane_left[y]=row_runs[y][pick].left;lane_right[y]=row_runs[y][pick].right;inner[y]=(int16_t)e;
        ring_result.mid[y]=(int16_t)clamp(e+(int)(ring_result.route*ring_result.lane_width*0.5f*
            sqrtf(1.0f+slope*slope)),lane_left[y]+2,lane_right[y]-2);
        last_l=lane_left[y];last_r=lane_right[y];++rows;
    }
    ring_result.near_x=ring_result.mid[RING_NEAR_ROW];ring_result.look_x=-1;
    ring_result.guide_y=(uint8_t)first;ring_result.guide_x=ring_result.mid[first];
    ring_result.inner_rows=(uint8_t)rows;ring_result.associated_rows=(uint8_t)associated;
    ring_result.target_support=5;ring_result.path_valid=1;
    ring_result.steer_error=(int16_t)clamp((ring_result.near_x-RING_W/2-1)+
        (ring_result.guide_x-ring_result.near_x)*(RING_NEAR_ROW-RING_LOOK_ROW)*2/
        (3*(RING_NEAR_ROW-first)),-90,90);
    return 1;
}
/* Row-wise lane/2*sqrt(1+slope^2) diverges near an island shoulder.
 * Offset measured INNER points in both coordinates instead, then sample
 * that centre curve. Only the already selected current branch may call
 * this; white pixels alone never acquire a ring or grant commitment.
 * Read row runs, not the image. A clipped edge is not an inner boundary. */
static int tangent_normal_path(void)
{
    int16_t path[RING_H];
    int y,j,start=-1,end=-1,previous_y=-1,previous_x=-1,top=118,bottom=-1;
    int guide=-1,want=119-clamp(ring_result.lane_width*3/4,41,83);
    int side=ring_result.route,near_measured;
    for(y=0;y<RING_H;++y) path[y]=-1;
    /* Trace from the near branch until its first clipping/discontinuity;
     * never resume on a distant background edge above the opening. */
    for(y=117;y>=24;--y) {
        if(inner[y]<=2 || inner[y]>=RING_W-3 ||
           (start>=0 && abs(inner[y]-inner[y+1])>8)) {
            if(start>=0) break;
            continue;
        }
        if(start<0) start=y;
        end=y;
    }
    if(start<0 || end<0 || start-end<24) return 0;
    for(y=start;y>=end;--y) {
        float slope,offset,norm;
        int xx,yy,k;
        if(!inner_fit(inner,y-4<end?end:y-4,y+4>start?start:y+4,&slope,&offset)) continue;
        norm=sqrtf(1+slope*slope);
        xx=(int)floorf(inner[y]+side*ring_result.lane_width*.5f/norm+.5f);
        yy=(int)floorf(y-side*ring_result.lane_width*.5f*slope/norm+.5f);
        if(yy<24 || yy>117 || xx<3 || xx>RING_W-4) continue;
        if(previous_y>=0 && (yy>=previous_y || previous_y-yy>12)) continue;
        /* Check every interpolated target in the CURRENT component.
         * A failed normal is rejected, never clamped onto another road. */
        for(k=yy;k<=(previous_y<0?yy:previous_y);++k) {
            int x=previous_y<0?xx:xx+(previous_x-xx)*(k-yy)/(previous_y-yy);
            int white=0;
            for(j=0;j<row_count[k];++j) if(connected[k][j] &&
                x>=row_runs[k][j].left+2 && x<=row_runs[k][j].right-2) {white=1;break;}
            if(!white) break;
        }
        if(k<=(previous_y<0?yy:previous_y)) continue;
        for(k=yy;k<=(previous_y<0?yy:previous_y);++k)
            path[k]=(int16_t)(previous_y<0?xx:xx+(previous_x-xx)*(k-yy)/(previous_y-yy));
        previous_y=yy;previous_x=xx;
    }
    /* Foreshortened normals can rasterise several source points onto the
     * same target row. Count actual contiguous measured CURVE support,
     * before adding any vehicle tail, rather than the number of accepted
     * raster samples. It must span at least our normal near/look baseline. */
    for(y=24;y<=117;++y) if(path[y]>=0) {if(top==118)top=y;bottom=y;}
    if(bottom<0 || bottom-top+1<RING_NEAR_ROW-RING_LOOK_ROW) return 0;
    for(y=top;y<=bottom;++y) if(path[y]<0) return 0;
    near_measured=path[RING_NEAR_ROW]>=0;
    /* A normal can project ahead of its measured inner edge. A missing
     * near target is usable only when that source edge itself reaches the
     * near row; a far-only fragment cannot invent approach support. The
     * checked tail is weak, never new strong or commitment evidence. */
    if(!near_measured && start<RING_NEAR_ROW) return 0;
    for(y=24;y<=RING_NEAR_ROW-8;++y) if(path[y]>=0 &&
        (guide<0 || abs(y-want)<abs(guide-want))) guide=y;
    if(guide<0 || bottom<RING_LOOK_ROW) return 0;
    if(bottom<117) {
        int k;
        /* Extend only the near tail of that measured curve to the car.
         * Check every current-white point, even if row108 was already
         * measured. This join adds no guide or branch-evidence support. */
        for(k=117;k>bottom;--k) {
            int x=RING_W/2+(path[bottom]-RING_W/2)*(117-k)/(117-bottom),white=0;
            for(j=0;j<row_count[k];++j) if(connected[k][j] &&
                x>=row_runs[k][j].left+2 && x<=row_runs[k][j].right-2) {white=1;break;}
            if(!white) return 0;
            path[k]=(int16_t)x;
        }
    }
    memcpy(ring_result.mid,path,sizeof(path));
    ring_result.near_x=path[RING_NEAR_ROW];
    ring_result.look_x=path[RING_LOOK_ROW]>=0?path[RING_LOOK_ROW]:path[guide];
    ring_result.guide_x=path[guide];ring_result.guide_y=(uint8_t)guide;
    /* Bearing to the measured centre curve from the vehicle origin.
     * The old near-position bias could reverse an otherwise rightward
     * bend when the camera's learned lane width exceeded its lookahead. */
    ring_result.steer_error=(int16_t)clamp((path[guide]-RING_W/2)*60/(119-guide),-90,90);
    ring_result.path_valid=1;ring_result.target_support=5;
    return near_measured?1:2;
}

/* A checked short tail can maintain control within the SAME existing
 * strong-observation budget. It cannot renew that budget or acquire a
 * branch on its own. */
static int tangent_normal_measurement(const ring_motion_t *m)
{
    int measured=tangent_normal_path();
    if(measured==2 && (!have_path || hold_rejections(m))) return 0;
    return measured;
}

static void tangent_store_path(const ring_motion_t *m,int strong)
{
    trusted_steer=ring_result.steer_error;trusted_near=ring_result.near_x;trusted_look=ring_result.look_x;
    memcpy(previous_mid,ring_result.mid,sizeof(previous_mid));
    if(strong) {
        memcpy(previous_inner,inner,sizeof(previous_inner));memcpy(strong_inner,inner,sizeof(strong_inner));
        have_path=1;last_path_ms=vision_ms;last_path_fault_ms=m->now_ms;last_yaw=m->yaw_deg;
        support_frames=0;support_travel=0;support_last_revs=m->wheel_revs;lost_frames=0;
    }
    if(lane_left[117]>=0 && ring_result.mid[117]>=0) {
        seed_left=lane_left[117];seed_right=lane_right[117];seed_x=ring_result.mid[117];
        seed_inner=inner[117]>2 && inner[117]<RING_W-3?inner[117]:-1;
    }
}
static void tangent_lap(const ring_motion_t *m)
{
    if(ring_result.lap_valid)
        ring_result.lap_deg=(m->yaw_deg-ring_result.lap_origin_yaw)*ring_result.route*RING_YAW_SIGN;
    if(ring_result.lap_valid && ring_result.lap_deg>=RING_EXIT_MIN_DEG)lap_completed=1;
    if(ring_result.lap_valid && timed_started && maneuver_feedback==3u &&
       ring_result.lap_deg>=RING_EXIT_NET_MIN_DEG)exit_net_reached=1;
    ring_result.turn_deg=ring_result.lap_deg;
}
static void tangent_start_entry(const ring_motion_t *m)
{
    int y;
    tangent_force_since=m->now_ms;
    ring_result.approach_stage=RING_TANGENT_FAR_READY;
    ring_result.entry_side=ring_result.ring_side;
    ring_result.lap_valid=1;ring_result.lap_origin_yaw=m->yaw_deg;ring_result.lap_deg=0;
    mouth_entry_slope=tangent_slope;mouth_entry_offset=tangent_offset;
    mouth_entry_inner_count=0;mouth_local_valid=mouth_local_changes=0;
    mouth_local_ms=mouth_local_curve_ms=vision_ms;mouth_local_yaw=m->yaw_deg;mouth_local_revs=m->wheel_revs;
    for(y=0;y<RING_H;++y) {
        int e=ring_result.route>0?observed_left[y]:observed_right[y];
        mouth_entry_inner[y]=-1;
        mouth_local_edge[y]=(int16_t)(y>=24 && e>2 && e<RING_W-3?e:-1);
    }
    /* Only a current returning-shoulder source may initialise tangent
     * support. An old width-neck/white midpoint grants no edge identity. */
    if(mouth_far_source==2) mouth_local_valid=1;
    tangent_lap_revs=m->wheel_revs;entry_yaw=confirm_yaw=m->yaw_deg;
    entry_revs=confirm_revs=m->wheel_revs;
    assist_total_yaw=assist_total_revs=0;assist_yaw=m->yaw_deg;assist_revs=m->wheel_revs;
    last_path_ms=vision_ms;last_path_fault_ms=m->now_ms;last_yaw=m->yaw_deg;
    support_frames=0;support_travel=0;support_last_revs=m->wheel_revs;
    tangent_left_count=0;tangent_return_seen=0;
    for(y=0;y<RING_H;++y) previous_inner[y]=strong_inner[y]=-1;
    change_phase(RING_ENTER,m->now_ms);
}

static int mouth_associate(mouth_track_t *track,int x,int y,float yaw,int changed)
{
    float angle=(yaw-track->yaw)*0.01745329252f;
    float lateral=track->x-RING_W/2,forward=119-track->y;
    float predicted_x=RING_W/2+cosf(angle)*lateral-sinf(angle)*forward;
    float predicted_y=119-(sinf(angle)*lateral+cosf(angle)*forward);
    float dx=x-predicted_x,dy=y-predicted_y;
    int tolerance=track==&mouth_near?RING_CORRIDOR_TOL:RING_MAX_ROW_STEP;
    if(!track->valid || vision_ms-track->last_seen_ms>RING_OBSERVE_MS) return 0;
    /* A short dropout may cross five visual frames. Recover the SAME
     * yaw-corrected corner within a bounded corridor; no motion credit is
     * added merely because the window is wider. Never revive expired N. */
    if(track->missed>5) tolerance+=clamp(track->missed-5,0,RING_MAX_ROW_STEP);
    if(fabsf(dx)>tolerance || dy<-8.0f || dy>tolerance) return 0;
    if(changed && dy>0.5f && (abs(x-track->x)>=1 || abs(y-track->y)>=1)) {
        float origin_angle=(yaw-track->origin_yaw)*0.01745329252f;
        float distance=119-y;
        float fixed_forward=cosf(origin_angle)*distance-sinf(origin_angle)*(x-RING_W/2);
        float progress=track->first_forward-fixed_forward;
        /* Net high water, never sums only positive increments: an image
         * corner oscillating at two adjacent pixels cannot earn 8px. */
        if(progress>track->high_forward) track->high_forward=progress;
        track->progress=(int)fmaxf(0,progress+0.5f);
        if(track->count<255) ++track->count;
    }
    track->x=x;track->y=y;track->yaw=yaw;track->missed=0;track->last_seen_ms=vision_ms;
    return 1;
}
static void mouth_seed(mouth_track_t *track,int x,int y,float yaw)
{
    memset(track,0,sizeof(*track));track->valid=1;track->x=x;track->y=track->first_y=y;
    track->yaw=track->origin_yaw=yaw;track->count=1;track->first_forward=(float)(119-y);
    track->last_seen_ms=vision_ms;
}
static int mouth_shared_corridor(void)
{
    int y,n=0;
    mouth_corridor_clipped=0;
    for(y=94;y<=117;++y) {
        int l=mouth_left[y],r=mouth_right[y],w=r-l+1;
        if(l>=0 && (l<=2 || r>=RING_W-3)) ++mouth_corridor_clipped;
        if(l>2 && r<RING_W-3 && w>=ring_result.lane_width*2/3 &&
           w<=ring_result.lane_width*3/2) ++n;
    }
    mouth_corridor_rows=n;
    return mouth_corridor_rows>=18;
}

/* A wide side mouth may never show two complete narrow boundaries. N
 * can still be seen clearing the side of the image: its measured tip has
 * progressed into the near band, then CURRENT connected white replaces
 * its previous inward edge above that tip. Missing pixels, an unchanged
 * island or only gyro/wheel motion cannot supply this visual event. */
static int mouth_near_clearance(int same,int reference,const ring_motion_t *m)
{
    int y,white=0,closed=0,side=ring_result.ring_side;
    float angle=(m->yaw_deg-mouth_near.yaw)*0.01745329252f;
    float lateral=mouth_near.x-RING_W/2,forward=119-mouth_near.y;
    int px=(int)(RING_W/2+cosf(angle)*lateral-sinf(angle)*forward);
    int py=(int)(119-(sinf(angle)*lateral+cosf(angle)*forward));
    int probe=px+side*3;
    if(same || !reference || !mouth_near.valid || !tangent_near_close ||
       !mouth_open_seen || vision_ms-mouth_near.last_seen_ms>RING_OBSERVE_MS ||
       mouth_near.y<RING_NEAR_ROW-16 ||
       (side>0?mouth_near.x<RING_W-1-RING_MAX_ROW_STEP:mouth_near.x>RING_MAX_ROW_STEP)) return 0;
    for(y=clamp(py-8,24,110);y<clamp(py-1,25,118);++y) {
        int l=mouth_left[y],r=mouth_right[y];
        if(probe>=3 && probe<=RING_W-4 && l>=0 && probe>=l+2 && probe<=r-2 &&
           mouth_centre[y]>=l && mouth_centre[y]<=r)
            ++white;
    }
    /* If N went below the near field, the bridge BELOW its tip becomes
     * the continuing inner edge. It moves inward through the last rows;
     * this is the opposite visual event to clearing the SIDE boundary. */
    if(mouth_near.y>=110) for(y=110;y<=117;++y) {
        int l=mouth_left[y],r=mouth_right[y],e=side>0?r:l;
        int inward=side*(px-e);
        if(l>2 && r<RING_W-3 && inward>=4 && inward<=ring_result.lane_width &&
           mouth_centre[y]>=l+2 && mouth_centre[y]<=r-2) ++closed;
    }
    return white>=6 || closed>=6;
}

/* The measured N tip can leave BELOW the image with its x still inside
 * the image. Its own current inner edge then replaces the former white
 * bridge at the bottom. Observe this LOCAL transition on the connected
 * through run; neither loss of a point nor unrelated top-image flicker is
 * passage. Never require an already clipped x or an invisible lower cap. */
static int mouth_bottom_contact(int reference,const ring_motion_t *m)
{
    int y,n=0,inward=0,moved=0,side=ring_result.ring_side,last=-1;
    float angle=(m->yaw_deg-mouth_near.yaw)*0.01745329252f;
    float px=RING_W/2+cosf(angle)*(mouth_near.x-RING_W/2)-
        sinf(angle)*(119-mouth_near.y);
    if(!reference || !mouth_near.valid || !tangent_near_close ||
       mouth_near.y<100 || vision_ms-mouth_near.last_seen_ms>RING_OBSERVE_MS)
        return 0;
    if(!mouth_bottom_since) {
        for(y=108;y<=117;++y) mouth_bottom_anchor[y-108]=mouth_previous_lower[y];
        mouth_bottom_since=vision_ms;
    }
    if(vision_ms-mouth_bottom_since>RING_OBSERVE_MS) {
        mouth_bottom_count=0;mouth_bottom_since=0;return 0;
    }
    for(y=117;y>=108;--y) {
        int l=mouth_left[y],r=mouth_right[y],e=side>0?r:l,old=mouth_previous_lower[y];
        if(l<0 || e<=2 || e>=RING_W-3 || fabsf(e-px)>RING_CORRIDOR_TOL ||
           (last>=0 && abs(e-last)>8) || mouth_centre[y]<l+2 || mouth_centre[y]>r-2)
            break;
        ++n;last=e;
        if(mouth_bottom_anchor[y-108]>=0 && side*(mouth_bottom_anchor[y-108]-e)>=3) ++inward;
        if(old>=0 && old!=e) ++moved;
    }
    if(n<6 || moved<1) return 0;
    if(!mouth_bottom_count) {
        if(inward<4) return 0;
        mouth_bottom_count=1;
    } else {
        if(mouth_bottom_count<255) ++mouth_bottom_count;
    }
    /* The bottom contact is an edge observation, not an imaginary tip.
     * Refresh N identity only within its original association corridor. */
    if(!mouth_associate(&mouth_near,side>0?mouth_right[117]:mouth_left[117],117,m->yaw_deg,1))
        return 0;
    return mouth_bottom_count>=RING_TANGENT_CONFIRM;
}

/* N and F are observed concurrently, with different identities. N must
 * pass before handover, but F history must not be erased at that event.
 * A whole frame can support control without proving visual motion. */
static int tangent_mouth_sequence(const island_t *island,const ring_motion_t *m,int reference,int cross)
{
    int same=0,reacquired=0,bottom=0,entry_path=0,source=1,changed=mouth_signature!=mouth_previous_signature;
    int y,nx=-1,ny=-1,fx=-1,fy=-1,known=ring_result.visual_dt_ms!=65535u;
    float yaw_step=fabsf(m->yaw_deg-mouth_previous_yaw);
    float rev_step=fabsf(m->wheel_revs-mouth_previous_revs);
    int motion_ok=yaw_step<=12.0f && rev_step<=0.4f;
    int corridor=reference && mouth_shared_corridor();
    uint16_t reject=0;
    ring_result.mouth_near_x=ring_result.mouth_near_y=-1;
    ring_result.mouth_far_x=ring_result.mouth_far_y=-1;
    if(!reference) reject|=RING_MOUTH_REF;
    if(!known) reject|=RING_MOUTH_CLOCK;
    if(cross) reject|=RING_MOUTH_CROSS;
    if(!motion_ok) reject|=RING_MOUTH_MOTION;
    if(!changed) reject|=RING_MOUTH_VISUAL_PROGRESS;
    if(!corridor && !mouth_corridor_seen) reject|=RING_MOUTH_CORRIDOR;
    if(ring_result.approach_stage==RING_TANGENT_SHARED && known && motion_ok && !cross && changed && corridor)
        mouth_corridor_seen=1;
    if(reference) {
        int k,n=0,width=0,mid_n=0,mid_width=0;
        fy=tangent_far_closure(&fx);
        for(k=108;k<=117;++k) if(mouth_left[k]>2 && mouth_right[k]<RING_W-3) {
            width+=mouth_right[k]-mouth_left[k]+1;++n;
        }
        for(k=82;k<=93;++k) if(mouth_left[k]>2 && mouth_right[k]<RING_W-3) {
            mid_width+=mouth_right[k]-mouth_left[k]+1;++mid_n;
        }
        /* A returning shoulder is the actual side portal, not a learned
         * width crossing. Observe it concurrently with N, retaining its
         * own identity. Only the wide, fully observed current corridor
         * may use this extra source; the normal fixed-world F is intact. */
        if(n>=8 && width/n>=ring_result.lane_width*3/2) {
            int portal_ok=tangent_portal_path();
            int wide_view=width/n>=ring_result.lane_width*7/4 && mid_n>=8 &&
                mid_width/mid_n>=ring_result.lane_width*3/2;
            if(portal_ok || (wide_view && mouth_portal_row>=0)) {
                fx=mouth_portal_x;fy=mouth_portal_row;source=2;
            }
        }
    }
    /* Missing shoulder observations cannot lock out every later neck
     * forever. After the same finite observation lifetime, a CURRENT
     * different source starts from zero count/progress, never inherits it. */
    if(source==1 && mouth_far_source==2 && mouth_far.valid &&
       vision_ms-mouth_far.last_seen_ms<=RING_OBSERVE_MS) {source=2;fx=fy=-1;}
    if(source!=mouth_far_source) {memset(&mouth_far,0,sizeof(mouth_far));mouth_ready=0;}
    mouth_far_source=source;ring_result.mouth_source=(uint8_t)source;
    if(island->valid && island->bridge) {
        ny=island->bottom;
        nx=ring_result.ring_side>0?island->left:island->right;
    }
        /* The acquisition detector intentionally stops at row108 to
         * require a bridge beneath the island. Continue the ALREADY
         * identified N tip into rows109..116 using its current inner edge
         * and white merge below it; do not freeze N at that detector cap. */
        if(reference && mouth_near.valid && tangent_near_close &&
           ring_result.approach_stage==RING_TANGENT_NEAR_SEEN) {
            int y;
            for(y=116;y>ny && y>=94;--y) {
                int e=ring_result.ring_side>0?mouth_right[y]:mouth_left[y];
                int b=y<116?y+2:y+1;
                int below=ring_result.ring_side>0?mouth_right[b]:mouth_left[b];
                int above=ring_result.ring_side>0?mouth_right[y-3]:mouth_left[y-3];
                if(e>2 && e<RING_W-3 && below>=0 && above>=0 &&
                   (ring_result.ring_side>0?below>=RING_W-3:below<=2) &&
                   ring_result.ring_side*(below-e)>=3 &&
                   ring_result.ring_side*(e-above)>=0) {nx=e;ny=y;break;}
            }
        }
    if(ny>=0) {
        if(mouth_near.valid) {
            int was_missing=mouth_near.missed>5;
            if(known && motion_ok && !cross) same=mouth_associate(&mouth_near,nx,ny,m->yaw_deg,changed);
            if(same && was_missing) reacquired=1;
            if(!same && known && motion_ok && !cross && reference &&
               vision_ms-mouth_near.last_seen_ms>RING_OBSERVE_MS &&
               ring_result.approach_stage==RING_TANGENT_NEAR_SEEN &&
               !mouth_near_cleared && tangent_same_island(island)) {
                /* Expiry starts a NEW observation, never retains the old
                 * progress/closing credit under a permanently valid bit. */
                mouth_seed(&mouth_near,nx,ny,m->yaw_deg);same=1;
                tangent_near_close=tangent_gap=mouth_open_seen=mouth_clear_count=0;
                mouth_bottom_count=0;mouth_bottom_since=0;
                memset(&mouth_near_closing,0,sizeof(mouth_near_closing));
                memset(&mouth_far,0,sizeof(mouth_far));mouth_ready=0;
                reacquired=1;
            }
        } else if(tangent_same_island(island)) {
            mouth_seed(&mouth_near,nx,ny,m->yaw_deg);same=1;
        }
    }
    if(same) {
        ring_result.mouth_near_x=(int16_t)nx;ring_result.mouth_near_y=(int16_t)ny;
        if(ny>ring_result.near_row) ring_result.near_row=(int16_t)ny;
    }
    /* Prefer a CURRENT one-sided opening/closing corner associated to N
     * after growing/returning island shape has left the frame. */
    if(!same && reference && mouth_near.valid && known && motion_ok && !cross) {
        /* A wide incoming lane can already exceed 4/3 learned width on
         * BOTH sides of N. Track the same returning side tip using its
         * local outward merge, not a crossing of that absolute width. */
        for(y=114;y>=28;--y) {
            int e=ring_result.ring_side>0?mouth_right[y]:mouth_left[y];
            int before=ring_result.ring_side>0?mouth_right[y-3]:mouth_left[y-3];
            int after=ring_result.ring_side>0?mouth_right[y+3]:mouth_left[y+3];
            int opposite=mouth_outside[y],k,stable=1;
            if(e<=2 || e>=RING_W-3 || before<0 || after<0 || opposite<0 ||
               ring_result.ring_side*(e-before)<0 ||
               ring_result.ring_side*(after-e)<ring_result.lane_width/4) continue;
            for(k=y-3;k<=y+3;++k)
                if(mouth_outside[k]<0 || abs(mouth_outside[k]-opposite)>RING_MAX_ROW_STEP) stable=0;
            if(stable && mouth_associate(&mouth_near,e,y,m->yaw_deg,changed)) {
                same=1;nx=e;ny=y;ring_result.mouth_near_x=(int16_t)e;
                ring_result.mouth_near_y=(int16_t)y;
                if(y>ring_result.near_row) ring_result.near_row=(int16_t)y;
                break;
            }
        }
    }
    if(!same && reference && mouth_near.valid && known && motion_ok && !cross) {
        int y;
        for(y=116;y>=24;--y) {
            int w=mouth_right[y]-mouth_left[y]+1;
            int prev=mouth_right[y-1]-mouth_left[y-1]+1;
            if(mouth_left[y]>=0 && mouth_left[y-1]>=0 &&
               ((w>ring_result.lane_width*4/3 && prev<=ring_result.lane_width*4/3) ||
                (prev>ring_result.lane_width*4/3 && w<=ring_result.lane_width*4/3))) {
                int x=ring_result.ring_side>0?mouth_right[y]:mouth_left[y];
                if(mouth_associate(&mouth_near,x,y,m->yaw_deg,changed)) {
                    same=1;nx=x;ny=y;ring_result.mouth_near_x=(int16_t)x;
                    ring_result.mouth_near_y=(int16_t)y;
                    if(y>ring_result.near_row) ring_result.near_row=(int16_t)y;
                    break;
                }
            }
        }
    }
    if(!same) {
        if(mouth_near.missed<255) ++mouth_near.missed;
        reject|=RING_MOUTH_NEAR;
    }
    if(fy>=0) {
        int distinct=1;
        int closing;
        for(closing=0;closing<mouth_closure_count;++closing)
            if(same && island->valid && mouth_closure_y[closing]>=island->top-4 &&
               mouth_closure_y[closing]<=island->bottom+4) {
                mouth_seed(&mouth_near_closing,mouth_closure_x[closing],mouth_closure_y[closing],m->yaw_deg);
                break;
            }
        ring_result.mouth_far_x=(int16_t)fx;ring_result.mouth_far_y=(int16_t)fy;
        ring_result.far_row=(int16_t)fy;
        if(ring_result.approach_stage==RING_TANGENT_NEAR_SEEN && mouth_near.valid &&
           fy>=mouth_near.y-12) distinct=0;
        /* The closing inside the tracked N island's vertical support is
         * N's own side boundary, not a second mouth. Keep its identity
         * after N passage so the same corner cannot be renamed as F. */
        if(source==2 && mouth_near.valid && fy>=mouth_near.y-24) distinct=0;
        if(source==1 && same && island->valid && fy>=island->top-4 && fy<=island->bottom+4) {
            mouth_seed(&mouth_near_closing,fx,fy,m->yaw_deg);distinct=0;
        } else if(source==1 && mouth_near_closing.valid && known && motion_ok && !cross &&
                  mouth_associate(&mouth_near_closing,fx,fy,m->yaw_deg,changed)) distinct=0;
        else ++mouth_near_closing.missed;
        if(!distinct) {
            reject|=RING_MOUTH_DISTINCT;
            if(mouth_far.valid && abs(mouth_far.y-fy)<=RING_MAX_ROW_STEP && abs(mouth_far.x-fx)<=RING_MAX_ROW_STEP)
                memset(&mouth_far,0,sizeof(mouth_far));
        }
        else if(known && motion_ok && !cross) {
            if(!mouth_associate(&mouth_far,fx,fy,m->yaw_deg,changed)) {
                /* A new F can begin inside a short current view. It still
                 * needs measured progress and N/corridor identity gates. */
                mouth_seed(&mouth_far,fx,fy,m->yaw_deg);
            }
        }
    } else {++mouth_far.missed;++mouth_near_closing.missed;reject|=RING_MOUTH_FAR;}
    if(!known || !motion_ok || cross) {
        tangent_gap=0;mouth_far.count=0;mouth_bottom_count=0;mouth_bottom_since=0;
    }
    if(ring_result.approach_stage==RING_TANGENT_NEAR_SEEN) {
        /* The observed N must progress towards the bottom; its threshold
         * is relative to its own first view, not an absolute row104. */
        if(mouth_near.count>=3 && mouth_near.progress>=8) tangent_near_close=1;
        if(reference && known && motion_ok && !cross && changed && tangent_near_open()) mouth_open_seen=1;
        if(known && motion_ok && !cross && changed)
            bottom=mouth_bottom_contact(reference,m);
        if(bottom) {
            mouth_near_cleared=1;
            ring_result.mouth_near_x=(int16_t)mouth_near.x;
            ring_result.mouth_near_y=(int16_t)mouth_near.y;
            ring_result.near_row=(int16_t)mouth_near.y;
        }
        if(known && motion_ok && !cross && changed &&
           mouth_near_clearance(same,reference,m)) ++mouth_clear_count;
        else mouth_clear_count=0;
        if(mouth_clear_count>=RING_TANGENT_CONFIRM) mouth_near_cleared=1;
        if(tangent_near_close && known && motion_ok && !cross && changed &&
           ((corridor && (mouth_open_seen || !same || ny<mouth_near.first_y-8)) ||
            (mouth_near_cleared && reference && (bottom || tangent_near_open())))) ++tangent_gap;
        else tangent_gap=0;
        if(tangent_gap>=RING_TANGENT_CONFIRM) {
            ring_result.approach_stage=RING_TANGENT_NEAR_PASSED;
            mouth_corridor_seen=1;tangent_far_revs=m->wheel_revs;
            mouth_pass_reason=corridor?1:2;
        }
    }
    if(!tangent_near_close) reject|=RING_MOUTH_NEAR_PROGRESS;
    if(ring_result.approach_stage!=RING_TANGENT_NEAR_PASSED &&
       ring_result.approach_stage!=RING_TANGENT_SHARED) reject|=RING_MOUTH_NEAR_PASSED;
    if(!mouth_far.valid || mouth_far.count<RING_TANGENT_CONFIRM) reject|=RING_MOUTH_ASSOC;
    if(mouth_far.progress<8) reject|=RING_MOUTH_VISUAL_PROGRESS;
    if(!tangent_far_window(fy,&entry_path)) reject|=RING_MOUTH_FAR_WINDOW;
    if(!entry_path) reject|=RING_MOUTH_ENTRY_PATH;
    if(fabsf(m->wheel_revs-tangent_far_revs)<0.05f) reject|=RING_MOUTH_MOTION;
    if(!mouth_corridor_seen) reject|=RING_MOUTH_CORRIDOR;
    if(ring_result.approach_stage>=RING_TANGENT_NEAR_PASSED) reject&=(uint16_t)~(RING_MOUTH_NEAR|RING_MOUTH_NEAR_PROGRESS);
    if(!reject && changed) ++mouth_ready;else mouth_ready=0;
    if(mouth_ready<RING_TANGENT_CONFIRM) reject|=RING_MOUTH_READY;
    ring_result.mouth_first_y=(int16_t)(mouth_far.valid?mouth_far.first_y:-1);
    ring_result.mouth_progress=(int16_t)mouth_far.progress;
    ring_result.mouth_near_count=(uint8_t)clamp(mouth_near.count,0,255);
    ring_result.mouth_far_count=(uint8_t)clamp(mouth_ready,0,255);
    ring_result.mouth_gap=(uint8_t)clamp(tangent_gap,0,255);
    ring_result.mouth_near_missed=(uint8_t)clamp(mouth_near.missed,0,255);
    ring_result.mouth_pass_reason=(uint8_t)(mouth_pass_reason?mouth_pass_reason:(reacquired?3:0));
    /* N is no longer required as a current visible point after passage. */
    if(ring_result.approach_stage>=RING_TANGENT_NEAR_PASSED) reject&=(uint16_t)~(RING_MOUTH_NEAR|RING_MOUTH_NEAR_PROGRESS);
    ring_result.mouth_gate=reject;
    for(y=0;y<RING_H;++y) mouth_previous_lower[y]=(int16_t)
        (reference && known && motion_ok && !cross?
            (ring_result.ring_side>0?mouth_right[y]:mouth_left[y]):-1);
    mouth_previous_signature=mouth_signature;mouth_previous_yaw=m->yaw_deg;mouth_previous_revs=m->wheel_revs;
    return reject==0;
}

/* Accumulate measured branch departure DURING the finite turn as well
 * as after it. Gyro progress by itself never earns visual commitment. */
static void tangent_entry_evidence(const ring_motion_t *m,int measured,int changed)
{
    if(ring_result.phase!=RING_ENTER || ring_result.approach_stage==RING_TANGENT_SHARED)return;
    if(ring_result.timed_entry && tangent_left_count>=(int)RING_BRANCH_CONFIRM)return;
    if(changed && measured==1 && ring_result.lap_deg>=20.f &&
       ring_result.guide_x>=0 && ring_result.guide_y<RING_NEAR_ROW &&
       tangent_entry_distance(m)>=ring_result.lane_width/6) {
        ring_result.left_entry=1;ring_result.approach_stage=RING_TANGENT_LEFT;
    }
    if(measured==1 && changed && ring_result.left_entry && ring_result.lap_deg>=35.f &&
       fabsf(m->wheel_revs-tangent_lap_revs)>=.1f) {
        if(tangent_left_count<(int)RING_BRANCH_CONFIRM)++tangent_left_count;
    } else tangent_left_count=0;
    if(tangent_left_count>=(int)RING_BRANCH_CONFIRM && !ring_result.timed_entry) {
        ring_result.committed=1;
        ring_result.decision=RING_DECISION_COMMIT;change_phase(RING_CIRCLE,m->now_ms);
    }
}

/* Only the finite75deg entry is scheduled. Hand steering to the actual
 * ring path afterwards, including its ordinary PID. Gyro thereafter only
 * measures lap progress; it never overrides the path or locks a heading. */
static int tangent_timed_entry(const island_t *island,const ring_motion_t *m,int cross)
{
    int measured=0;
    if(!ring_result.timed_entry)return 0;
    ring_result.force_entry=0;
    if(maneuver_feedback==4u ||
       (!maneuver_feedback && m->now_ms-phase_since>300u)) {
        ring_release_control(maneuver_feedback==4u?1:4,m);return 1;
    }
    if(ring_result.phase==RING_APPROACH && maneuver_feedback<2u) {
        int reference=tangent_reference();
        int portal=tangent_mouth_sequence(island,m,reference,cross);
        /* v30: recognition has already confirmed structure and side.
         * Keep N/F observations for diagnostics/exit; they must not veto
         * the scheduled entry a second time. Runtime still enforces the
         * finite delay, forward-speed readiness and fresh RUN command. */
        (void)portal;
        ring_result.force_entry=1;
        ring_result.path_valid=(uint8_t)(reference && tangent_line_path(0));
        ring_result.branch_ok=0;ring_result.path_source=0;
        ring_result.decision=RING_DECISION_ASSIST;
        if(m->now_ms-phase_since>RING_APPROACH_TIMEOUT_MS) {
            ring_result.decision=RING_DECISION_CANCEL_APPROACH;cancel_uncommitted(m);
        }
        return 1;
    }
    (void)tangent_reference();
    if(maneuver_feedback>=2u && ring_result.phase==RING_APPROACH){
        tangent_start_entry(m);timed_started=1;
        /* Reconstruct the actual control-side turn origin. The configured
         * straight heading correction is never credited as lap angle. */
        ring_result.lap_origin_yaw=m->yaw_deg-ring_result.ring_side*maneuver_turn10*.1f;
        entry_yaw=confirm_yaw=ring_result.lap_origin_yaw;
    }
    if(maneuver_feedback==3u) {
        ring_result.timed_entry=0;
        if(tangent_left_count>=(int)RING_BRANCH_CONFIRM && ring_result.left_entry && have_path &&
           !seed_identity_lost && m->now_ms-last_path_fault_ms<=RING_PATH_HOLD_MS) {
            ring_result.committed=1;change_phase(RING_CIRCLE,m->now_ms);
        }
        /* Critical: otherwise ring_blend_error() supplies ZERO to the
         * original PID forever, even when a real branch reports e=61. */
        return 0;
    }
    tangent_lap(m);
    if(ring_result.phase==RING_ENTER && !seed_identity_lost && ring_result.visual_dt_ms!=65535u){
        measured=build_path(island,island->valid&&island->growing&&island->returning,0,0,0,m)==1;
        if(measured) {measured=tangent_normal_measurement(m);if(!measured)ring_result.reject|=RING_REJ_PATH;}
        if(measured && !tangent_entry_branch(m))measured=0;
        if(!measured && tangent_entry_inner_path()) {
            measured=tangent_normal_measurement(m);
            if(measured && !tangent_entry_branch(m))measured=0;
        }
        if(measured){
            tangent_store_path(m,measured==1);
            if(measured==1) {
                memcpy(mouth_local_edge,inner,sizeof(inner));mouth_local_valid=1;
                mouth_local_ms=mouth_local_curve_ms=vision_ms;mouth_local_yaw=m->yaw_deg;
                mouth_local_revs=m->wheel_revs;
            }
        }
    }
    tangent_entry_evidence(m,measured,mouth_signature!=mouth_previous_signature);
    mouth_previous_signature=mouth_signature;
    ring_result.branch_ok=(uint8_t)(measured!=0);ring_result.path_valid=(uint8_t)measured;
    ring_result.path_source=(uint8_t)(measured?1:0);
    ring_result.branch_mode=(uint8_t)(measured==2?RING_BRANCH_PARTIAL:(measured?RING_BRANCH_STRONG:RING_BRANCH_NONE));
    ring_result.entry_flags=(uint16_t)(measured?RING_ENTRY_MEASURED:0);
    ring_result.decision=RING_DECISION_ASSIST;
    ring_result.entry_turn10=maneuver_turn10;
    if(!measured){
        int y;ring_result.steer_error=0;ring_result.near_x=ring_result.look_x=-1;
        ring_result.guide_x=-1;ring_result.guide_y=255;
        for(y=0;y<RING_H;++y)ring_result.mid[y]=-1;
        /* During the explicit gyro turn an old bottom seed is not a
         * current visual branch. Reacquire from the car centre next frame. */
        if(!have_path || m->now_ms-last_path_fault_ms>RING_PATH_HOLD_MS) {
            have_path=0;seed_left=seed_right=seed_x=seed_inner=-1;
        }
    }
    return 1;
}

static int tangent_exit_heading(void)
{
    if(timed_started && RING_EXIT_NET_MIN_DEG>0.f)
        return maneuver_feedback==3u && exit_net_reached &&
            ring_result.lap_deg>=RING_EXIT_MIN_DEG &&
            ring_result.lap_deg<=RING_EXIT_SEARCH_END_DEG;
    return ring_result.lap_deg>=RING_EXIT_MIN_DEG &&
        ring_result.lap_deg<=RING_EXIT_MIN_DEG+RING_LAP_EXIT_HEADING_TOL &&
        (!timed_started || (maneuver_feedback==3u && lap_completed));
}

/* Predict the previous measured corridor after a small camera yaw change.
 * Translation has no pixel/metre calibration: it is NOT invented here.
 * Current edge measurements and bounded residuals must still agree. */
static int tangent_exit_prediction(const exit_probe_t *p,int y,float yaw)
{
    float a=(yaw-p->yaw)*.01745329252f,c=cosf(a),s=sinf(a);
    float slope,offset,den,old_y;
    if(fabsf(yaw-p->yaw)>(ring_result.exit_turn_request?
       RING_EXIT_FORCE_DEG+RING_EXIT_FORCE_ALIGN_DEG:RING_EXIT_CONTINUE_YAW_DEG) ||
       p->near_x<0 || p->look_x<0)return -1;
    slope=(p->near_x-p->look_x)/(float)(RING_NEAR_ROW-RING_LOOK_ROW);
    offset=p->near_x-slope*RING_NEAR_ROW-RING_W/2;
    den=c-slope*s;
    if(fabsf(den)<.2f)return -1;
    old_y=(y-RING_H+offset*s+RING_H*c)/den;
    if(old_y<21.f || old_y>120.f)return -1;
    return (int)(RING_W/2+(slope*c+s)*old_y+offset*c-RING_H*s+.5f);
}

/* A selected near-to-far corridor, not an all-white/single-run image.
 * The island may remain beside it. The far ROAD must be narrow, measured,
 * connected to the near ROAD and aligned with the near outside boundary.
 * No target is clamped across dark pixels or copied from an old image. */
static int tangent_exit_probe(exit_probe_t *out,int locked,const ring_motion_t *m)
{
    int y,j,last_l=-1,last_r=-1,last_edge=-1,rows=0,streak=0;
    int side=ring_result.ring_side,lane=ring_result.lane_width;
    int16_t outside[RING_H];
    float slope=0,offset=0,centre_offset,norm,pred_slope=0,pred_offset=0;
    const exit_probe_t *old=locked?&exit_locked_path:&exit_candidate_path;
    memset(out,0,sizeof(*out));
    for(y=0;y<RING_H;++y)out->mid[y]=out->left[y]=out->right[y]=outside[y]=-1;
    out->near_x=out->look_x=-1;out->yaw=m->yaw_deg;
    if(!side || lane<RING_MIN_LANE_WIDTH || ring_result.visual_dt_ms==65535u)return 0;
    if(locked) {
        int near=tangent_exit_prediction(old,RING_NEAR_ROW,m->yaw_deg);
        int look=tangent_exit_prediction(old,RING_LOOK_ROW,m->yaw_deg);
        if(near<0 || look<0)return 0;
        pred_slope=(near-look)/(float)(RING_NEAR_ROW-RING_LOOK_ROW);
        pred_offset=near-pred_slope*RING_NEAR_ROW;
    }
    for(y=117;y>=24;--y) {
        int pick=-1,best=10000,expected=RING_W/2;
        if(locked) {
            expected=(int)(pred_slope*y+pred_offset+.5f);
            if(expected<0)break;
        }
        for(j=0;j<row_count[y];++j) {
            run_t r=row_runs[y][j];
            int edge=side>0?r.left:r.right,score;
            if(!connected[y][j] || edge<=2 || edge>=RING_W-3 ||
               r.right-r.left+1<lane*2/3)continue;
            if(last_l<0) {
                if(expected<r.left+2 || expected>r.right-2)continue;
                score=abs(edge-(expected-side*lane/2));
            } else {
                if(r.left>last_r+RING_CONNECT_GAP || r.right<last_l-RING_CONNECT_GAP ||
                   abs(edge-last_edge)>RING_MAX_ROW_STEP)continue;
                if(locked && (expected<r.left+2 || expected>r.right-2))continue;
                score=abs(edge-last_edge);
            }
            if(score<best){pick=j;best=score;}
        }
        if(pick<0)break;
        out->left[y]=row_runs[y][pick].left;out->right[y]=row_runs[y][pick].right;
        outside[y]=(int16_t)(side>0?out->left[y]:out->right[y]);
        last_l=out->left[y];last_r=out->right[y];last_edge=outside[y];
        ++rows;if(y>=98)++out->near_rows;
    }
    if(rows<52 || out->near_rows<12 || !inner_fit(outside,98,117,&slope,&offset))return 0;
    norm=sqrtf(1.f+slope*slope);
    centre_offset=offset+side*lane*.5f*norm;
    for(y=117;y>=24 && outside[y]>=0;--y) {
        int target=(int)(slope*y+centre_offset+.5f);
        int l=out->left[y],r=out->right[y],w=r-l+1;
        int safety=lane/4,expected;
        /* Alignment is measured against the CURRENT near tangent. A
         * curved ring continuation or an unrelated distant lane fails. */
        if(abs((int)(slope*y+offset+.5f)-outside[y])>lane/6 ||
           target-safety<l+2 || target+safety>r-2)break;
        if(locked) {
            expected=(int)(pred_slope*y+pred_offset+.5f);
            if(expected<0 || abs(target-expected)>lane/4)break;
        }
        out->mid[y]=(int16_t)target;
        if(y<=RING_LOOK_ROW && l>2 && r<RING_W-3 &&
           w>=lane*(2.f/3.f)*norm && w<=lane*(4.f/3.f)*norm && abs((l+r)/2-target)<=lane/6) {
            ++streak;if(streak>out->far_rows)out->far_rows=streak;
        } else streak=0;
    }
    out->near_x=out->mid[RING_NEAR_ROW];out->look_x=out->mid[RING_LOOK_ROW];
    return out->near_x>=0 && out->look_x>=0 && out->far_rows>=12;
}

static int tangent_exit_associated(const exit_probe_t *old,const exit_probe_t *p)
{
    int y,n=0;
    for(y=RING_LOOK_ROW;y<=RING_NEAR_ROW;y+=10) {
        int expected=tangent_exit_prediction(old,y,p->yaw);
        if(expected<0 || p->mid[y]<0 || abs(expected-p->mid[y])>ring_result.lane_width/4)return 0;
        ++n;
    }
    return n>=3;
}

#include "ring_exit_opening.h"
#include "ring_exit_corridor.h"

/* A measured outgoing corridor may request the last at most70 degrees
 * toward the incoming straight. The success handover still needs an
 * actually completed lap and positive net travel. left_entry is diagnostic. */
static int tangent_exit_observe(const ring_motion_t *m)
{
    exit_probe_t probe;
    float forward=m->wheel_revs-tangent_lap_revs;
    int valid=0,force_finish=0;
    uint16_t rej=0;
    ring_result.exit_forward_mrev=(int32_t)fmaxf(-2147483000.f,fminf(2147483000.f,forward*1000.f));
    if(!tangent_exit_heading())rej|=RING_EXIT_REJ_LAP;
    if(!ring_result.committed)rej|=RING_EXIT_REJ_COMMIT;
    if(forward<RING_EXIT_MIN_REVS)rej|=RING_EXIT_REJ_MOTION;
    if(ring_result.lap_deg>=(RING_EXIT_NET_MIN_DEG>0.f?RING_EXIT_MIN_DEG:RING_EXIT_FORCE_START_DEG) &&
       (RING_EXIT_NET_MIN_DEG<=0.f || exit_net_reached) &&
       ring_result.lap_deg<=RING_EXIT_SEARCH_END_DEG) {
        valid=tangent_exit_probe(&probe,0,m);
        if(!valid)valid=tangent_exit_corridor(&probe,m);
        /* The outgoing lane can still be sideways beside the island.
         * Its connected opening is observation only, never a fake path. */
        if(!valid)valid=tangent_exit_opening(&probe,m);
    }
    if(valid && timed_started && maneuver_feedback==3u && ring_result.lap_valid &&
       (RING_EXIT_NET_MIN_DEG<=0.f || exit_net_reached)) {
        float remaining=RING_EXIT_MIN_DEG-ring_result.lap_deg;
        float road_angle=atan2f((float)(probe.look_x-probe.near_x),
            (float)(RING_NEAR_ROW-RING_LOOK_ROW))*57.295779513f;
        /* The same original-straight heading is mandatory before AND
         * after360. A late tangent elsewhere on the ring cannot exit. */
        if(fabsf(road_angle-ring_result.ring_side*remaining)>RING_EXIT_FORCE_ALIGN_DEG) {
            valid=0;rej|=RING_EXIT_REJ_HEADING;
        } else if(remaining>0.f && remaining<=RING_EXIT_FORCE_DEG) {
            force_finish=1;rej&=(uint16_t)~RING_EXIT_REJ_LAP;
        } else if(remaining<=0.f && ring_result.lap_deg<=RING_EXIT_SEARCH_END_DEG) {
            /* Correct a modest late observation back to the same straight,
             * never add another70deg or start a second revolution. */
            force_finish=fabsf(remaining)>RING_EXIT_FORCE_ALIGN_DEG;
            rej&=(uint16_t)~RING_EXIT_REJ_LAP;
        }
    }
    if(!valid) {
        rej|=RING_EXIT_REJ_GEOMETRY;exit_candidate_count=0;
    } else {
        if(!exit_candidate_count || m->now_ms-exit_candidate_last_ms>RING_PATH_HOLD_MS ||
           !(tangent_exit_associated(&exit_candidate_path,&probe) ||
             tangent_exit_opening_associated(&exit_candidate_path,&probe))) {
            exit_candidate_count=0;exit_candidate_since=m->now_ms;
        }
        exit_candidate_last_ms=m->now_ms;
        exit_candidate_path=probe;
        if(exit_candidate_count<255u)++exit_candidate_count;
    }
    ring_result.exit_candidate=(uint8_t)valid;
    ring_result.exit_candidate_frames=(uint8_t)exit_candidate_count;
    if(exit_candidate_count<RING_EXIT_CANDIDATE_FRAMES ||
       m->now_ms-exit_candidate_since<RING_EXIT_CANDIDATE_MS)rej|=RING_EXIT_REJ_CANDIDATE;
    ring_result.exit_reject=rej;
    if(!rej && !ring_result.exit_latched) {
        exit_locked_path=probe;exit_track_valid=1;
        exit_start_ms=m->now_ms;exit_start_yaw=m->yaw_deg;exit_start_revs=m->wheel_revs;
        ring_result.circle_observe_only=0;
        ring_result.exit_latched=1;ring_result.exit_confirm_frames=0;
        ring_result.exit_turn_request=(uint8_t)force_finish;
        ring_result.approach_stage=RING_TANGENT_EXIT;
        stable_count=0;change_phase(RING_EXIT,m->now_ms);
    }
    return valid;
}

static void tangent_exit_apply(const exit_probe_t *p)
{
    int y;
    for(y=0;y<RING_H;++y) {
        ring_result.mid[y]=p->mid[y];lane_left[y]=p->left[y];lane_right[y]=p->right[y];inner[y]=-1;
    }
    ring_result.near_x=(int16_t)p->near_x;ring_result.look_x=(int16_t)p->look_x;
    ring_result.guide_y=RING_LOOK_ROW;ring_result.guide_x=(int16_t)p->look_x;
    ring_result.steer_error=(int16_t)clamp((p->near_x-RING_W/2-1)+
        (p->look_x-p->near_x)*2/3,-90,90);
    ring_result.path_valid=1;
}

static int tangent_exit_continue(const ring_motion_t *m)
{
    exit_probe_t current;
    ring_result.exit_candidate=0;
    if(!exit_track_valid || !ring_result.exit_latched ||
       m->now_ms-exit_start_ms>RING_EXIT_TIMEOUT_MS ||
       (ring_result.exit_turn_request?
        (exit_turn_feedback==3u || ring_result.lap_deg<
            fminf(RING_EXIT_MIN_DEG,ring_result.ring_side*(exit_start_yaw-ring_result.lap_origin_yaw))-(RING_EXIT_NET_MIN_DEG>0.f?RING_EXIT_FORCE_ALIGN_DEG:RING_EXIT_FORCE_BACKTRACK_DEG) ||
         ring_result.lap_deg>RING_EXIT_SEARCH_END_DEG):
        (ring_result.lap_deg<RING_EXIT_MIN_DEG ||
         fabsf(m->yaw_deg-exit_start_yaw)>RING_EXIT_CONTINUE_YAW_DEG)) ||
       m->wheel_revs<exit_start_revs-.002f) {
        ring_result.exit_reject|=RING_EXIT_REJ_LOCK_LOST;return 0;
    }
    if(!tangent_exit_probe(&current,1,m)) {
        float angle,expected=ring_result.ring_side*(RING_EXIT_MIN_DEG-ring_result.lap_deg);
        /* During the explicit bounded finish, the original camera corridor
         * can rotate out of view. Reacquire only a CURRENT measured road
         * pointing at the SAME outgoing heading; never reuse old pixels. */
        if(!(tangent_exit_probe(&current,0,m) || tangent_exit_corridor(&current,m))) {
            ring_result.exit_reject|=RING_EXIT_REJ_GEOMETRY|RING_EXIT_REJ_LOCK_LOST;return 0;
        }
        angle=atan2f((float)(current.look_x-current.near_x),
            (float)(RING_NEAR_ROW-RING_LOOK_ROW))*57.295779513f;
        if(fabsf(angle-expected)>RING_EXIT_FORCE_ALIGN_DEG ||
           (!ring_result.exit_turn_request && !tangent_exit_associated(&exit_locked_path,&current))) {
            ring_result.exit_reject|=RING_EXIT_REJ_GEOMETRY|RING_EXIT_REJ_LOCK_LOST;return 0;
        }
    }
    exit_locked_path=current;tangent_exit_apply(&current);ring_result.exit_reject=0;
    ring_result.exit_candidate=1;
    return 1;
}

/* Keep the locked exit until the ordinary lookahead is unambiguous.
 * Distant island pixels are allowed; a second usable near branch or a
 * wide near mouth must not be handed back to ordinary midpoint tracking. */
static int tangent_exit_handover_ready(void)
{
    int y,j,rows=0,lane=ring_result.lane_width;
    for(y=RING_LOOK_ROW;y<=117;++y) {
        int l=exit_locked_path.left[y],r=exit_locked_path.right[y],other=0;
        if(l<=2 || r>=RING_W-3 || r-l+1<lane*2/3 || r-l+1>lane*4/3)continue;
        for(j=0;j<row_count[y];++j) {
            run_t run=row_runs[y][j];
            if(connected[y][j] && run.right-run.left+1>=lane/2 &&
               (run.left!=l || run.right!=r))other=1;
        }
        if(!other && abs((l+r)/2-exit_locked_path.mid[y])<=lane/6)++rows;
    }
    return rows>=35;
}

#include "ring_outer_path.h"

/* Lost steering evidence must not erase a proven physical ring episode.
 * Continue the unchanged ordinary follower, retain only lap/exit context.
 * No old curve is renewed and no yaw target is commanded in this mode. */
static int tangent_circle_observe(const ring_motion_t *m)
{
    int y;
    if(!timed_started || maneuver_feedback!=3u || ring_result.phase!=RING_CIRCLE ||
       !ring_result.committed || !ring_result.left_entry || !ring_result.lap_valid)return 0;
    ring_result.circle_observe_only=1;
    ring_result.path_valid=ring_result.branch_ok=ring_result.path_source=0;
    ring_result.branch_mode=RING_BRANCH_NONE;ring_result.force_entry=0;
    ring_result.steer_error=0;ring_result.near_x=ring_result.look_x=-1;
    ring_result.guide_x=-1;ring_result.guide_y=255;
    for(y=0;y<RING_H;++y)ring_result.mid[y]=-1;
    ring_result.decision=RING_DECISION_OBSERVE;
    ring_result.entry_flags=RING_ENTRY_COMMITTED;
    ring_result.path_age_ms=(uint16_t)fminf(65535.f,(float)(vision_ms-last_path_ms));
    ring_result.lost_frames=(uint8_t)clamp(lost_frames,0,255);
    ring_result.entry_turn10=(int16_t)fmaxf(-32768.f,fminf(32767.f,ring_result.lap_deg*10.f));
    mouth_previous_signature=mouth_signature;(void)m;return 1;
}

static void tangent_active(const island_t *island,const ring_motion_t *m,int cross)
{
    int y,measured=0,reference=0,direct=0,junction=0,local=0;
    int visual_changed=mouth_signature!=mouth_previous_signature;
    uint32_t now=m->now_ms;
    if(now-ring_since>RING_TOTAL_TIMEOUT_MS) {ring_release_control(2,m);return;}
    if(tangent_timed_entry(island,m,cross))return;
    tangent_lap(m);
    ring_result.exit_forward_mrev=(int32_t)fmaxf(-2147483000.f,fminf(2147483000.f,
        (m->wheel_revs-tangent_lap_revs)*1000.f));
    if(ring_result.phase>=RING_EXIT && now-(timed_started?exit_start_ms:phase_since)>RING_EXIT_TIMEOUT_MS) {
        ring_release_control(3,m);return;
    }
    if(ring_result.phase==RING_APPROACH) {
        reference=tangent_reference();
        if(reference) tangent_line_path(0);
        /* This is only a preview. Ordinary error, speed and cross ownership
         * remain with the untouched original controller throughout N->F. */
        ring_result.branch_ok=0;ring_result.path_source=0;ring_result.steer_error=0;
        if(now-phase_since>RING_APPROACH_TIMEOUT_MS) {
            ring_result.decision=RING_DECISION_CANCEL_APPROACH;cancel_uncommitted(m);return;
        }
        if(tangent_mouth_sequence(island,m,reference,cross)) tangent_start_entry(m);
        else {ring_result.steer_error=0;return;}
    }
    ++support_frames;support_travel+=fabsf(m->wheel_revs-support_last_revs);
    support_last_revs=m->wheel_revs;
    assist_total_yaw+=fabsf(m->yaw_deg-assist_yaw);
    assist_total_revs+=fabsf(m->wheel_revs-assist_revs);
    assist_yaw=m->yaw_deg;assist_revs=m->wheel_revs;
    if(ring_result.visual_dt_ms==65535u) {
        /* Unknown capture time is not a new strong observation. In
         * particular it cannot advance F, shared-return or commitment. */
        ring_result.reject|=RING_REJ_STALE;
        tangent_return_count=0;mouth_far.count=0;
        if(!ring_result.exit_latched) {
            exit_candidate_count=0;ring_result.exit_candidate=0;
            ring_result.exit_candidate_frames=0;
            ring_result.exit_reject|=RING_EXIT_REJ_GEOMETRY|RING_EXIT_REJ_CANDIDATE;
        } else {ring_result.exit_candidate=0;ring_result.exit_reject|=RING_EXIT_REJ_LOCK_LOST;}
        goto tangent_lost;
    }
    if(!timed_started) {
        if(ring_result.phase==RING_CIRCLE && ring_result.committed &&
           ring_result.approach_stage!=RING_TANGENT_SHARED && ring_result.left_entry &&
           ring_result.lap_deg>=345.0f && ring_result.lap_deg<=375.0f) {
            reference=tangent_reference();
            if(reference && tangent_near_open()) ++tangent_return_count;
            else tangent_return_count=0;
            if(tangent_return_count>=RING_TANGENT_CONFIRM) {
                int closing_x=-1,closing_y=tangent_far_closure(&closing_x);
                ring_result.approach_stage=RING_TANGENT_SHARED;
                /* Scheduled entry deliberately did not track N->F on approach.
                 * Its recognition-time N is not the return mouth's identity.
                 * Start return association from current pixels/closing below. */
                if(timed_started)memset(&mouth_near,0,sizeof(mouth_near));
                memset(&mouth_far,0,sizeof(mouth_far));mouth_corridor_seen=mouth_ready=0;
                memset(&mouth_near_closing,0,sizeof(mouth_near_closing));
                if(closing_y>=0) mouth_seed(&mouth_near_closing,closing_x,closing_y,m->yaw_deg);
                mouth_previous_signature=mouth_signature;mouth_previous_yaw=m->yaw_deg;mouth_previous_revs=m->wheel_revs;
                tangent_far_revs=m->wheel_revs;
            }
    }
        if(ring_result.approach_stage==RING_TANGENT_SHARED) {
            reference=tangent_reference();
            measured=reference && tangent_line_path(0);
            if(measured && tangent_mouth_sequence(island,m,reference,cross) &&
               (ring_result.lap_deg>=RING_EXIT_MIN_DEG || timed_started) && tangent_exit_heading() &&
               fabsf(m->wheel_revs-tangent_lap_revs)>=RING_EXIT_MIN_REVS) {
                ring_result.approach_stage=RING_TANGENT_EXIT;change_phase(RING_EXIT,now);
            }
        } else if(ring_result.phase>=RING_EXIT || visual_exit_count) {
            reference=tangent_reference();measured=reference && tangent_line_path(0);
        } else {measured=-1;}
    } else {
        if(ring_result.phase<=RING_CIRCLE) (void)tangent_exit_observe(m);
        if(ring_result.phase==RING_EXIT || ring_result.phase==RING_RECOVER) {
            measured=tangent_exit_continue(m);
            if(!measured && (now-exit_start_ms>RING_EXIT_TIMEOUT_MS ||
               (ring_result.exit_turn_request?
                (exit_turn_feedback==3u || ring_result.lap_deg<
                 fminf(RING_EXIT_MIN_DEG,ring_result.ring_side*(exit_start_yaw-ring_result.lap_origin_yaw))-(RING_EXIT_NET_MIN_DEG>0.f?RING_EXIT_FORCE_ALIGN_DEG:RING_EXIT_FORCE_BACKTRACK_DEG) ||
                 ring_result.lap_deg>RING_EXIT_SEARCH_END_DEG):
                (ring_result.lap_deg<RING_EXIT_MIN_DEG ||
                 fabsf(m->yaw_deg-exit_start_yaw)>RING_EXIT_CONTINUE_YAW_DEG)))) {
                ring_release_control(3,m);return;
            }
        } else {measured=-1;}
    }
    if(ring_result.circle_observe_only) {
        /* A newly associated CURRENT curved outside edge may recover ring
         * steering. Do not reacquire an unrelated white midpoint or let
         * an old inner history renew itself during ordinary observation. */
        measured=tangent_outer_path(m);
        if(measured)ring_result.circle_observe_only=0;
        else if(tangent_circle_observe(m))return;
    }
    if(measured<0) {
        measured=0;
        /* The measured near inner edge at the return junction still
         * belongs to ring following; it is not an early straight exit. */
        if(!seed_identity_lost)junction=measured=tangent_return_path();
        if(!measured)
            measured=build_path(island,island->valid && island->growing && island->returning,0,0,0,m)==1;
        if(seed_identity_lost) measured=0;
        /* build_path() only selects the current boundary. Its provisional
         * row offsets are not a steering path, especially at a shoulder.
         * Validate the actual normal BEFORE branch tests or history writes. */
        if(measured && !junction) {
            measured=tangent_normal_measurement(m);
            if(!measured)ring_result.reject|=RING_REJ_PATH;
        }
        if(measured && ring_result.phase==RING_ENTER && !ring_result.committed &&
           !tangent_entry_branch(m)) {
            measured=0;ring_result.reject|=RING_REJ_BRANCH;
        }
        if(!measured && !cross && !seed_identity_lost && tangent_entry_inner_path()) {
            measured=tangent_normal_measurement(m);
            if(measured && tangent_entry_branch(m))ring_result.reject=0;
            else measured=0;
        }
        if(!measured && !cross && !seed_identity_lost) {
            local=tangent_local_path(m);
            if(local && (ring_result.left_entry || tangent_entry_branch(m))) measured=local;
        }
        if(!measured && !cross && !seed_identity_lost) measured=tangent_outer_path(m);
        if(!timed_started && !measured && ring_result.phase==RING_ENTER && !ring_result.left_entry &&
           now-tangent_force_since<=RING_FORCE_TOTAL_MS && assist_total_yaw<=RING_FORCE_ENTRY_YAW_DEG &&
           assist_total_revs<=RING_FORCE_ENTRY_REVS &&
           ring_result.visual_dt_ms!=65535u && !cross && !seed_identity_lost) {
            reference=tangent_reference();
            direct=reference && (tangent_portal_path() || tangent_line_path(1));
        }
    }
    ring_result.branch_ok=(uint8_t)(measured!=0);
    ring_result.path_source=(uint8_t)(measured?((ring_result.phase>=RING_EXIT)?4:
        (measured==3?7:((junction || local)?6:1))):(direct?5:0));
    if(measured) {
        ring_result.branch_mode=(uint8_t)(ring_result.phase>=RING_EXIT?RING_BRANCH_EXIT:
            (measured==2?RING_BRANCH_PARTIAL:((junction || local || measured==3)?RING_BRANCH_JUNCTION:RING_BRANCH_STRONG)));
        ring_result.decision=RING_DECISION_MEASURED;ring_result.stop=0;
        tangent_store_path(m,measured==1);
        if(measured==3)tangent_outer_store(m);
        ring_result.entry_flags=RING_ENTRY_MEASURED|RING_ENTRY_HISTORY_FRESH;
        if(ring_result.phase<=RING_CIRCLE && ring_result.approach_stage!=RING_TANGENT_SHARED &&
           !junction && measured!=3 && !visual_exit_count) {
            if(measured==1) {
                memcpy(mouth_local_edge,inner,sizeof(inner));mouth_local_valid=1;
                mouth_local_ms=mouth_local_curve_ms=vision_ms;mouth_local_yaw=m->yaw_deg;
                mouth_local_revs=m->wheel_revs;
            }
            tangent_entry_evidence(m,measured,visual_changed);
        }
        if(!timed_started) {
            if(ring_result.phase>=RING_EXIT) {
                if(now-phase_since>RING_EXIT_TIMEOUT_MS) {ring_release_control(3,m);return;}
                if(ordinary_lane() && tangent_exit_heading()) ++stable_count;else stable_count=0;
                if(ring_result.phase==RING_EXIT && stable_count>=RING_RECOVER_FRAMES) change_phase(RING_RECOVER,now);
                else if(ring_result.phase==RING_RECOVER && stable_count>=RING_RECOVER_FRAMES) {
                    ++ring_result.exited_events;cooldown_since=now;cooldown_revs=m->wheel_revs;
                    ring_result.timed_entry=0;
                    ring_result.decision=RING_DECISION_EXIT;change_phase(RING_COOLDOWN,now);
                }
        }
        } else {
            if(ring_result.phase>=RING_EXIT) {
                /* Both recovery phases validate the SAME current exit corridor.
                 * Do not re-run the initial15deg window or whole-image ordinary
                 * test, and do not restart the4s deadline at RECOVER. */
                if(tangent_exit_handover_ready() &&
                   (RING_EXIT_NET_MIN_DEG<=0.f || (exit_net_reached &&
                    fabsf(ring_result.lap_deg-RING_EXIT_MIN_DEG)<=RING_EXIT_FORCE_ALIGN_DEG)) &&
                   (ring_result.lap_deg>=RING_EXIT_MIN_DEG ||
                    (ring_result.exit_turn_request && exit_turn_feedback==2u &&
                     ring_result.lap_deg>=RING_EXIT_MIN_DEG-(RING_EXIT_NET_MIN_DEG>0.f?RING_EXIT_FORCE_ALIGN_DEG:RING_EXIT_FORCE_BACKTRACK_DEG))) &&
                   (!ring_result.exit_turn_request || exit_turn_feedback==2u)) {
                    if(stable_count<RING_RECOVER_FRAMES)++stable_count;
                } else stable_count=0;
                ring_result.exit_confirm_frames=(uint8_t)stable_count;
                if(ring_result.phase==RING_EXIT && stable_count>=RING_RECOVER_FRAMES) {
                    change_phase(RING_RECOVER,now);stable_count=0;
                } else if(ring_result.phase==RING_RECOVER && stable_count>=RING_RECOVER_FRAMES &&
                          m->wheel_revs-exit_start_revs>=RING_EXIT_CLEAR_REVS) {
                    ++ring_result.exited_events;cooldown_since=now;cooldown_revs=m->wheel_revs;
                    ring_result.timed_entry=0;ring_result.exit_turn_request=0;ring_result.exit_confirmed=1;
                    ring_result.decision=RING_DECISION_EXIT;change_phase(RING_COOLDOWN,now);
                }
        }
        }
    } else if(direct) {
        ring_result.branch_mode=RING_BRANCH_ASSIST;ring_result.decision=RING_DECISION_ASSIST;
        ring_result.entry_flags=RING_ENTRY_ASSIST|(ring_result.ring_side<0?RING_ENTRY_ASSIST_LEFT:0);
        ring_result.path_valid=0;ring_result.stop=0;tangent_store_path(m,0);
    } else {
tangent_lost:
        if(ring_result.exit_latched) {stable_count=0;ring_result.exit_confirm_frames=0;}
        if(ring_result.phase==RING_EXIT && ring_result.exit_turn_request &&
           exit_turn_feedback<=1u && ring_result.visual_dt_ms!=65535u &&
           now-exit_start_ms<=RING_EXIT_FORCE_TIMEOUT_MS) {
            /* A newly commanded finite gyro turn may temporarily rotate the
             * road out of view. Do not credit pixels or reset its budget. */
            ring_result.path_valid=ring_result.branch_ok=0;
            ring_result.path_source=0;ring_result.branch_mode=RING_BRANCH_NONE;
            ring_result.steer_error=0;ring_result.near_x=ring_result.look_x=-1;
            for(y=0;y<RING_H;++y)ring_result.mid[y]=-1;
            ring_result.decision=RING_DECISION_ASSIST;return;
        }
        /* Once entry is proven, a missing ring branch hands steering to
         * the CURRENT ordinary image immediately, without retaining an
         * old turn. The ring episode itself remains alive for its exit. */
        if(tangent_circle_observe(m))return;
        if(ring_result.phase==RING_ENTER) tangent_left_count=0;
        ++lost_frames;ring_result.reject|=RING_REJ_BRANCH;
        ring_result.hold_reject=hold_rejections(m);
        ring_result.steer_error=trusted_steer;ring_result.near_x=trusted_near;ring_result.look_x=trusted_look;
        ring_result.path_valid=0;ring_result.guide_x=-1;ring_result.guide_y=255;
        for(y=0;y<RING_H;++y) ring_result.mid[y]=-1;
        if(!ring_result.hold_reject) {
            ring_result.path_source=2;ring_result.branch_mode=RING_BRANCH_HOLD;
            ring_result.decision=RING_DECISION_HOLD;
        } else if(timed_started && have_path && !seed_identity_lost &&
                  now-last_path_fault_ms<=RING_PATH_HOLD_MS) {
            /* Short handover gap: keep only the last observed steering,
             * with no extra yaw target or refreshed visual evidence. */
            ring_result.path_source=2;ring_result.branch_mode=RING_BRANCH_HOLD;
            ring_result.decision=RING_DECISION_HOLD;
        } else if(!ring_result.left_entry && can_cancel_uncommitted()) {
            ring_result.decision=RING_DECISION_CANCEL_HOLD;cancel_uncommitted(m);return;
        } else {ring_release_control(1,m);return;}
        if(now-last_path_fault_ms>RING_PATH_TIMEOUT_MS) {ring_release_control(1,m);return;}
    }
    ring_result.lost_frames=(uint8_t)clamp(lost_frames,0,255);
    ring_result.path_age_ms=ring_result.strong_age_ms=(uint16_t)fminf(65535.0f,(float)(vision_ms-last_path_ms));
    ring_result.hold_turn10=(int16_t)fminf(32767.0f,fabsf(m->yaw_deg-last_yaw)*10);
    ring_result.entry_turn10=(int16_t)fmaxf(-32768,fminf(32767,ring_result.lap_deg*10));
    /* CIRCLE confirms the visual branch; it does not mean the F turn has
     * reached its heading. Carry the SAME non-renewable F budget through
     * this transition. No force without a current measured/portal path. */
    ring_result.force_entry=(uint8_t)(!timed_started && (ring_result.phase==RING_ENTER ||
        (ring_result.phase==RING_CIRCLE && ring_result.approach_stage==RING_TANGENT_LEFT)) &&
        !ring_result.stop && !ring_result.fault && !cross && !seed_identity_lost &&
        (direct || measured) && now-tangent_force_since<=RING_FORCE_TOTAL_MS &&
        assist_total_yaw<=RING_FORCE_ENTRY_YAW_DEG &&
        assist_total_revs<=RING_FORCE_ENTRY_REVS);
    if(ring_result.committed) ring_result.entry_flags|=RING_ENTRY_COMMITTED;
    mouth_previous_signature=mouth_signature;
}


void ring_update(const uint8_t image[RING_H][RING_W],const ring_motion_t *m)
{
    island_t island;
    uint32_t dt=m->now_ms-last_frame_ms,now=m->now_ms;
    int y,cross,ordinary,structure=0,measured=0,route,acquired=0,assisted=0;
#ifdef RING_HOST_TEST
    ring_scan_pixels=0;
    ring_graph_checks=0;
#endif
    ++ring_result.frame_sequence;
    ring_result.visual_dt_ms=interval_supplied?supplied_interval:
        (uint16_t)(dt>65534u?65534u:dt);
    /* Vision owns evidence/hold lifetimes; real control milliseconds own
     * total/phase/fault deadlines. Unknown vdt invalidates cached evidence,
     * but must NEVER fabricate elapsed time and shorten a 30-second guard. */
    if(ring_result.visual_dt_ms!=65535u) vision_ms+=ring_result.visual_dt_ms;
    interval_supplied=0;last_frame_ms=m->now_ms;
    ring_result.candidate=ring_result.candidate_detected=ring_result.branch_ok=0;
    ring_result.force_entry=0;
    ring_result.path_source=ring_result.classification=0;ring_result.reject=0;
    ring_result.mouth_near_x=ring_result.mouth_near_y=-1;
    ring_result.mouth_far_x=ring_result.mouth_far_y=-1;
    ring_result.mouth_ref=0;ring_result.mouth_gate=0;ring_result.mouth_source=0;
    ring_result.mouth_near_missed=ring_result.mouth_pass_reason=0;
    ring_result.branch_mode=RING_BRANCH_NONE;
    ring_result.inner_rows=ring_result.associated_rows=ring_result.target_support=0;
    ring_result.guide_y=255u;ring_result.guide_x=-1;
    ring_result.strong_age_ms=(uint16_t)((vision_ms-last_path_ms)>65535u?65535u:vision_ms-last_path_ms);
    if(ring_result.phase!=RING_COOLDOWN && ring_result.phase!=RING_FAULT) {
        ring_result.decision=RING_DECISION_NONE;ring_result.entry_flags=ring_result.hold_reject=0;
        ring_result.lost_frames=0;ring_result.path_age_ms=0;
        ring_result.hold_turn10=ring_result.entry_turn10=ring_result.entry_travel_mrev=0;
    }
    if(ring_result.visual_dt_ms==65535u && (observing || have_path)) ring_result.reject|=RING_REJ_STALE;
    if(!RING_ENABLE || !m->ready || ring_result.phase==RING_FAULT) return;
    if(reentry_latched && now-reentry_since>=RING_REENTRY_BLOCK_MS)
        reentry_latched=0;
    scan_geometry(image,m);island=find_island();cross=cross_evidence();ordinary=observed_ordinary();
    ring_result.candidate_detected=(uint8_t)island.valid;
    ring_result.island_near=island.valid?(int16_t)island.bottom:-1;
    if(!island.valid) ring_result.reject|=RING_REJ_NO_SPLIT;
    if(RING_TRACK_MODE && ring_result.phase>=RING_APPROACH && ring_result.phase<=RING_RECOVER) {
        ring_result.classification=(uint8_t)(cross?1:(island.valid?2:0));
        ring_result.cross_veto=(uint8_t)cross;
        tangent_active(&island,m,cross);return;
    }
    if(ring_result.phase==RING_COOLDOWN) {
        if(!island.valid && now-cooldown_since>=RING_COOLDOWN_MS &&
           fabsf(m->wheel_revs-cooldown_revs)>=RING_COOLDOWN_REVS) {
            have_path=lost_frames=0;reset_observation();
            ring_result.route=ring_result.entry_side=0;
            ring_result.path_valid=ring_result.committed=ring_result.circle_observe_only=0;ring_result.turn_deg=0;
            ring_result.ring_side=0;ring_result.approach_stage=RING_TANGENT_NONE;
            ring_result.left_entry=ring_result.lap_valid=0;
            ring_result.release_reason=0;
            ring_result.exit_candidate=ring_result.exit_latched=ring_result.exit_confirmed=0;
            ring_result.exit_candidate_frames=ring_result.exit_confirm_frames=0;
            ring_result.exit_reject=0;ring_result.exit_forward_mrev=0;
            exit_track_valid=exit_net_reached=0;exit_candidate_count=0;
            ring_result.lap_deg=ring_result.lap_origin_yaw=0;
            ring_result.near_row=ring_result.far_row=-1;tangent_return_seen=0;
            for(y=0;y<RING_H;++y) previous_mid[y]=previous_inner[y]=ring_result.mid[y]=-1;
            change_phase(RING_NORMAL,now);
        }
        return;
    }
    if(ring_result.phase==RING_NORMAL) {
        ring_result.path_valid=0;ring_result.near_x=ring_result.look_x=-1;
        ring_result.confirm_frames=0;
        if(observing) {
            ++observe_frames;observe_travel+=fabsf(m->wheel_revs-observe_last_revs);
            observe_last_revs=m->wheel_revs;
            if(ring_result.visual_dt_ms==65535u || vision_ms-observe_since>RING_OBSERVE_MS ||
               observe_frames>(int)RING_OBSERVE_FRAMES) {
                ring_result.reject|=RING_REJ_EXPIRED;reset_observation();
            }
        }
        if(island.valid) {
            if(observing && (abs(island.center-candidate_x)>RING_CORRIDOR_TOL ||
                             abs(island.center-candidate_anchor)>RING_CORRIDOR_TOL*2)) {
                ring_result.reject|=RING_REJ_CORRIDOR;reset_observation();
            }
            route=RING_ROUTE?RING_ROUTE:(observing?confirm_route:(island.center<=RING_W/2?1:-1));
            if(!observing) {
                observing=1;observe_since=vision_ms;observe_frames=1;observe_last_revs=m->wheel_revs;
                candidate_x=candidate_anchor=island.center;confirm_route=route;
            }
            candidate_x=island.center;
            observed_growth|=island.growing;observed_return|=island.returning;
            ring_result.score=(uint8_t)(2+island.bridge+observed_growth+observed_return);
            structure=observed_growth && observed_return;
            if(!structure) ring_result.reject|=RING_REJ_NO_RETURN;
        } else {
            confirm_count=0;
            if(ring_result.score) --ring_result.score;
        }
        ring_result.classification=(uint8_t)(cross?(structure?3:1):(structure?2:0));
        ring_result.cross_veto=(uint8_t)cross;
        if(cross || original_state==0) {
            ring_result.reject|=cross?(structure?RING_REJ_CONFLICT:RING_REJ_CROSS):RING_REJ_PATH;
            reset_observation();return;
        }
        ring_result.candidate=(uint8_t)(observing && ring_result.score>=3);
        if(!island.valid || !structure) {
            if(!observing && !island.valid && observed_ordinary()) {
                int w=observed_right[RING_NEAR_ROW]-observed_left[RING_NEAR_ROW]+1;
                if(w>=RING_MIN_LANE_WIDTH && w<=RING_MAX_LANE_WIDTH)
                    ring_result.lane_width=(int16_t)((ring_result.lane_width*7+w)/8);
            }
            return;
        }
        ring_result.route=(int8_t)confirm_route;
        ring_result.entry_side=(int8_t)confirm_route;
        measured=build_path(&island,1,0,cross,ordinary,m);
        ring_result.branch_ok=(uint8_t)(measured!=0);
        ring_result.path_source=(uint8_t)(measured?1:0);
        /* Candidates/standby preview remain visible, but blocked frames
         * cannot bank confirmation or emit another entry/beep. Only NORMAL
         * acquisition is gated: never cut the current ring at15 seconds. */
        if(RING_TRACK_MODE && reentry_latched) {
            reset_observation();return;
        }
        if(measured) {
            ++confirm_count;
            memcpy(previous_mid,ring_result.mid,sizeof(previous_mid));
            memcpy(previous_inner,inner,sizeof(previous_inner));
        }
        else {confirm_count=0;ring_result.reject|=RING_REJ_PATH;}
        ring_result.confirm_frames=(uint8_t)confirm_count;
        if(confirm_count<RING_CONFIRM_FRAMES) return;
        ring_since=now;entry_yaw=confirm_yaw=m->yaw_deg;
        entry_revs=confirm_revs=m->wheel_revs;
        ring_result.committed=0;cancel_count=0;lost_frames=0;
        assist_state=assist_side=assist_count=assist_candidate=assist_clock=0;
        assist_total_yaw=assist_total_revs=0;
        entry_origin_row=entry_last_row=entry_progress_row=entry_high_row=-1;entry_changes=entry_measurements=0;
        support_frames=0;support_travel=0;support_last_revs=m->wheel_revs;
        last_path_ms=vision_ms;last_path_fault_ms=now;
        last_yaw=m->yaw_deg;have_path=1;
        memcpy(previous_mid,ring_result.mid,sizeof(previous_mid));
        memcpy(previous_inner,inner,sizeof(previous_inner));
        change_phase(RING_APPROACH,now);++ring_result.detected_events;
        if(RING_TRACK_MODE){reentry_since=now;reentry_latched=1;}
        if(RING_TRACK_MODE) {
            ring_result.ring_side=(int8_t)-ring_result.route;
            ring_result.approach_stage=RING_TANGENT_NEAR_SEEN;
            ring_result.left_entry=ring_result.lap_valid=0;
            ring_result.lap_deg=ring_result.lap_origin_yaw=0;
            ring_result.near_row=(int16_t)island.bottom;ring_result.far_row=-1;
            tangent_near_close=tangent_gap=tangent_left_count=tangent_return_count=0;
            tangent_return_seen=0;mouth_corridor_seen=mouth_ready=mouth_open_seen=0;
            mouth_far_source=0;mouth_local_valid=mouth_local_changes=0;
            mouth_near_cleared=mouth_clear_count=mouth_pass_reason=0;
            mouth_bottom_count=0;mouth_bottom_since=0;
            for(y=0;y<RING_H;++y) mouth_previous_lower[y]=-1;
            memset(&mouth_near,0,sizeof(mouth_near));memset(&mouth_far,0,sizeof(mouth_far));
            memset(&mouth_near_closing,0,sizeof(mouth_near_closing));
            mouth_previous_signature=mouth_signature;mouth_previous_yaw=m->yaw_deg;mouth_previous_revs=m->wheel_revs;
            mouth_seed(&mouth_near,ring_result.ring_side>0?island.left:island.right,island.bottom,m->yaw_deg);
            tangent_reference();ring_result.steer_error=0;
            if(RING_TIMED_ENTRY){
                ring_result.timed_entry=1;ring_result.force_entry=1;
                timed_started=0;maneuver_feedback=0;maneuver_turn10=0;
                have_path=0;seed_left=seed_right=seed_x=seed_inner=-1;
                ring_result.path_valid=(uint8_t)(ring_result.mouth_ref && tangent_line_path(0));
            }
            return;
        }
#if !RING_TRACK_MODE
        acquired=1;
#endif
    } else {
        structure=island.valid && island.growing && island.returning;
        ring_result.classification=(uint8_t)(cross?(structure?3:1):(structure?2:0));
        ring_result.cross_veto=(uint8_t)cross;
    }
    if(now-ring_since>RING_TOTAL_TIMEOUT_MS) {fail(1,now);return;}
    ring_result.turn_deg=(m->yaw_deg-entry_yaw)*ring_result.route*RING_YAW_SIGN;
    if(ring_result.phase==RING_APPROACH && now-phase_since>RING_APPROACH_TIMEOUT_MS) {
        if(can_cancel_uncommitted()) {ring_result.decision=RING_DECISION_CANCEL_APPROACH;cancel_uncommitted(m);}
        else fail(2,now);
        return;
    }
    if(ring_result.visual_dt_ms==65535u || ring_result.visual_dt_ms>RING_PATH_HOLD_MS) {
        entry_origin_row=entry_last_row=entry_progress_row=entry_high_row=-1;entry_changes=entry_measurements=0;
    }
    if(!acquired) {
        ++support_frames;support_travel+=fabsf(m->wheel_revs-support_last_revs);
        support_last_revs=m->wheel_revs;
        measured=build_path(&island,structure,ring_result.phase>=RING_EXIT,cross,ordinary,m);
    }
    if(seed_identity_lost ||
       (ring_result.phase<RING_EXIT && (ordinary || (cross && !ring_result.committed)))) measured=0;
    ring_result.branch_ok=(uint8_t)(measured && ring_result.phase<RING_EXIT);
    ring_result.path_source=(uint8_t)(measured?(ring_result.phase>=RING_EXIT?4:1):0);
    if(!measured) {
        ring_result.branch_mode=RING_BRANCH_NONE;
        ring_result.guide_x=-1;ring_result.guide_y=255u;
    }
    if(measured) {
        if(measured==1) {
            have_path=1;last_path_ms=vision_ms;last_path_fault_ms=now;
            last_yaw=m->yaw_deg;
            support_frames=0;support_travel=0;support_last_revs=m->wheel_revs;
            memcpy(strong_inner,inner,sizeof(strong_inner));
        }
        lost_frames=0;ring_result.stop=0;
        trusted_steer=ring_result.steer_error;trusted_near=ring_result.near_x;trusted_look=ring_result.look_x;
        if(lane_left[117]>=0 && ring_result.mid[117]>=0 &&
           (measured==1 || (inner[117]>2 && inner[117]<RING_W-3))) {
            seed_left=lane_left[117];seed_right=lane_right[117];seed_x=ring_result.mid[117];
            /* The incoming white bridge has no island edge at this row;
             * its outer road boundary must not masquerade as that edge. */
            seed_inner=inner[117]>2 && inner[117]<RING_W-3?inner[117]:-1;
        }
        if(measured==1 && ring_result.phase<RING_EXIT && !cross && !ordinary && ring_result.visual_dt_ms!=65535u)
            remember_entry_geometry();
        if(measured==1) {
            memcpy(previous_mid,ring_result.mid,sizeof(previous_mid));
            memcpy(previous_inner,inner,sizeof(previous_inner));
        } else for(y=24;y<=117;++y) if(inner[y]>2 && inner[y]<RING_W-3) {
            /* Missing rows in a partial view have no new branch identity.
             * Do not replace their last measured corridor with a white-area
             * offset; that would prevent genuine strong-edge reacquisition. */
            previous_mid[y]=ring_result.mid[y];previous_inner[y]=inner[y];
        }
        ring_result.decision=RING_DECISION_MEASURED;
    } else {
        ++lost_frames;
        ring_result.reject|=RING_REJ_BRANCH;
        /* Zero source means no trusted control path. A bounded hold keeps
         * only the last control, NOT a fresh observation or CIRCLE credit.
         * No uncalibrated pixel rotation is attempted on the raw camera. */
        ring_result.hold_reject=hold_rejections(m);
        if(ring_result.hold_reject) {
            entry_origin_row=entry_last_row=entry_progress_row=entry_high_row=-1;entry_changes=entry_measurements=0;
        }
        /* Failed construction must not replace the control we are holding. */
        ring_result.steer_error=trusted_steer;
        ring_result.near_x=trusted_near;ring_result.look_x=trusted_look;
        if(!ring_result.hold_reject) {
            ring_result.path_source=2;ring_result.stop=0;ring_result.decision=RING_DECISION_HOLD;
            ring_result.branch_mode=RING_BRANCH_HOLD;
        }
        for(y=0;y<RING_H;++y) ring_result.mid[y]=-1;
        stable_count=exit_count=0;
        assisted=entry_assist(m,0,cross,ordinary);
    }
    /* Decide physical entry before any cancellation. The island edge must
     * actually evolve into the near field; yaw on a fixed island/road is
     * insufficient. Short missing frames may finish the motion gate only
     * inside ALL existing hold bounds, never after they have expired. */
    ring_result.entry_flags=(uint16_t)((ring_result.entry_flags&(RING_ENTRY_ASSIST|RING_ENTRY_ASSIST_LEFT)) |
        (assist_state==2?RING_ENTRY_ASSIST_SPENT:0) |
        (measured==1?RING_ENTRY_MEASURED:0) |
        (entry_measurements>=3?RING_ENTRY_HISTORY:0) |
        (entry_geometry_progressed()?RING_ENTRY_PROGRESSED:0) |
        (entry_last_row>=96?RING_ENTRY_NEAR:0) |
        ((m->yaw_deg-confirm_yaw)*ring_result.route*RING_YAW_SIGN>=RING_COMMIT_DEG?RING_ENTRY_TURN_OK:0) |
        (fabsf(m->wheel_revs-confirm_revs)>=0.05f?RING_ENTRY_TRAVEL_OK:0) |
        (((measured==1 && ring_result.visual_dt_ms!=65535u) ||
          (ring_result.path_source==2 && !seed_identity_lost))?RING_ENTRY_HISTORY_FRESH:0) |
        (cross?RING_ENTRY_CROSS:0) | (ordinary?RING_ENTRY_ORDINARY:0));
    if(ring_result.phase>=RING_ENTER && ring_result.phase<=RING_CIRCLE && !cross && !ordinary &&
       (ring_result.entry_flags&(RING_ENTRY_PROGRESSED|RING_ENTRY_NEAR|RING_ENTRY_TURN_OK|
                                RING_ENTRY_TRAVEL_OK|RING_ENTRY_HISTORY_FRESH))==
                               (RING_ENTRY_PROGRESSED|RING_ENTRY_NEAR|RING_ENTRY_TURN_OK|
                                RING_ENTRY_TRAVEL_OK|RING_ENTRY_HISTORY_FRESH)) {
        if(!ring_result.committed) ring_result.decision=RING_DECISION_COMMIT;
        ring_result.committed=1;
    }
    if(ring_result.committed) ring_result.entry_flags|=RING_ENTRY_COMMITTED;
    ring_result.lost_frames=(uint8_t)clamp(lost_frames,0,255);
    ring_result.path_age_ms=(uint16_t)((vision_ms-last_path_ms)>65535u?65535u:vision_ms-last_path_ms);
    ring_result.strong_age_ms=ring_result.path_age_ms;
    ring_result.hold_turn10=(int16_t)fminf(32767.0f,fabsf(m->yaw_deg-last_yaw)*10.0f);
    ring_result.entry_turn10=(int16_t)fmaxf(-32768.0f,fminf(32767.0f,
        (m->yaw_deg-confirm_yaw)*ring_result.route*RING_YAW_SIGN*10.0f));
    ring_result.entry_travel_mrev=(int16_t)fminf(32767.0f,fabsf(m->wheel_revs-confirm_revs)*1000.0f);
    if(can_cancel_uncommitted() && !ring_result.branch_ok &&
       (cross || (!island.valid && ordinary))) ++cancel_count;
    else cancel_count=0;
    if(cancel_count>=RING_CANCEL_FRAMES) {
        if(ring_result.decision<RING_DECISION_ASSIST_RELEASE)
            ring_result.decision=cross?RING_DECISION_CANCEL_CROSS:RING_DECISION_CANCEL_ORDINARY;
        cancel_uncommitted(m);return;
    }
    if(!measured) {
        if(ring_result.hold_reject && !assisted) {
            if(can_cancel_uncommitted()) {
                if(ring_result.decision<RING_DECISION_ASSIST_RELEASE)
                    ring_result.decision=RING_DECISION_CANCEL_HOLD;
                cancel_uncommitted(m);return;
            }
            ring_result.stop=1;ring_result.decision=RING_DECISION_STOP_LOST;
        }
        if(now-last_path_fault_ms>RING_PATH_TIMEOUT_MS) fail(3,now);
        return;
    }
    if(measured==1 && ring_result.phase==RING_APPROACH && island.valid &&
       island.bottom>=RING_ENTRY_ROW && ring_result.branch_ok) {
        entry_yaw=m->yaw_deg;entry_revs=m->wheel_revs;change_phase(RING_ENTER,now);
    }
    ring_result.turn_deg=(m->yaw_deg-entry_yaw)*ring_result.route*RING_YAW_SIGN;
    if(ring_result.phase==RING_ENTER) {
        if(measured==1 && ring_result.committed && ring_result.branch_ok && ring_result.path_source==1 && ring_result.turn_deg>=35.0f &&
           fabsf(m->wheel_revs-entry_revs)>=0.1f) ++stable_count;
        else stable_count=0;
        if(stable_count>=(int)RING_BRANCH_CONFIRM) change_phase(RING_CIRCLE,now);
    } else if(ring_result.phase==RING_CIRCLE) {
        if(measured==1 && ring_result.branch_ok && ring_result.turn_deg>=RING_EXIT_MIN_DEG &&
           fabsf(m->wheel_revs-entry_revs)>=RING_EXIT_MIN_REVS && exit_opening()) ++exit_count;
        else exit_count=0;
        if(exit_count>=RING_EXIT_CONFIRM_FRAMES) {
            /* Leave on the local NEAR tangent. Reusing the full curved lap
             * midline here would keep guiding around the island again. */
            exit_slope=(ring_result.mid[102]-ring_result.mid[114])/(-12.0f);
            exit_slope=fmaxf(-1.0f,fminf(1.0f,exit_slope));
            exit_offset=ring_result.mid[RING_NEAR_ROW]-exit_slope*RING_NEAR_ROW;
            change_phase(RING_EXIT,now);
        }
    } else if(ring_result.phase==RING_EXIT || ring_result.phase==RING_RECOVER) {
        if(now-phase_since>RING_EXIT_TIMEOUT_MS) {fail(4,now);return;}
        if(!island.valid && ordinary_lane()) ++stable_count;
        else stable_count=0;
        if(ring_result.phase==RING_EXIT && stable_count>=RING_RECOVER_FRAMES) change_phase(RING_RECOVER,now);
        else if(ring_result.phase==RING_RECOVER && stable_count>=RING_RECOVER_FRAMES) {
            ++ring_result.exited_events;cooldown_since=now;cooldown_revs=m->wheel_revs;
            ring_result.decision=RING_DECISION_EXIT;
            reset_observation();change_phase(RING_COOLDOWN,now);
        }
    }
    if(measured) entry_assist(m,measured,cross,ordinary);
    if(assist_state==2) ring_result.entry_flags|=RING_ENTRY_ASSIST_SPENT;
}
