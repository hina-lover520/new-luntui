#ifndef CODE_RING_EXIT_CORRIDOR_H_
#define CODE_RING_EXIT_CORRIDOR_H_

/* Current connected exit corridor after a completed revolution. Unlike
 * the entry follower, its seed need not contain the car centre at row117.
 * A cropped junction may hide part of the outside edge. Visible edge
 * samples must agree with the incoming-road heading and support a fit;
 * every returned control point is checked against CURRENT cached runs. */
#ifdef RING_HOST_TEST
static uint32_t ring_corridor_cache_checks;
#endif
static unsigned ring_corridor_checks_remaining;
static int tangent_corridor_check(void)
{
    if(!ring_corridor_checks_remaining)return 0;
    --ring_corridor_checks_remaining;
#ifdef RING_HOST_TEST
    ++ring_corridor_cache_checks;
#endif
    return 1;
}
static int tangent_exit_corridor(exit_probe_t *out,const ring_motion_t *m)
{
    static const int seeds[]={117,108,98,88};
    int si,j,y,best=-1,lane=ring_result.lane_width,side=ring_result.ring_side;
    float expected=side*(RING_EXIT_MIN_DEG-ring_result.lap_deg);
    float prior=-tanf(expected*.01745329252f),norm=sqrtf(1.f+prior*prior);
    exit_probe_t candidate;
    ring_corridor_checks_remaining=32000u;
#ifdef RING_HOST_TEST
    ring_corridor_cache_checks=0;
#endif
    if(!timed_started || maneuver_feedback!=3u || !ring_result.committed ||
       !ring_result.lap_valid || ring_result.visual_dt_ms==65535u ||
       ring_result.lap_deg<RING_EXIT_MIN_DEG ||
       (RING_EXIT_NET_MIN_DEG>0.f && !exit_net_reached) ||
       ring_result.lap_deg>RING_EXIT_SEARCH_END_DEG ||
       lane<RING_MIN_LANE_WIDTH || lane>RING_MAX_LANE_WIDTH || (side!=1 && side!=-1))return 0;
    for(si=0;si<4;++si)for(j=0;j<row_count[seeds[si]];++j) {
        int sy=seeds[si],edge=side>0?row_runs[sy][j].left:row_runs[sy][j].right;
        int16_t measured[RING_H];
        int support=0,top=118,bottom=-1,near=0,far=0,streak=0,rows=0,holes=0,gap=0,max_gap=0;
        float offset=edge-prior*sy,slope,fit,centre,fit_norm;
        if(!tangent_corridor_check())return 0;
        if(!connected[sy][j] || edge<=2 || edge>=RING_W-3)continue;
        for(y=0;y<RING_H;++y)measured[y]=-1;
        for(y=117;y>=24;--y) {
            int k,target=(int)floorf(prior*y+offset+side*lane*.5f*norm+.5f);
            for(k=0;k<row_count[y];++k) {
                run_t r=row_runs[y][k];int e=side>0?r.left:r.right;
                if(!tangent_corridor_check())return 0;
                if(!connected[y][k] || target<r.left+2 || target>r.right-2 ||
                   e<=2 || e>=RING_W-3 || fabsf(e-(prior*y+offset))>lane/8.f+2.f)continue;
                measured[y]=(int16_t)e;++support;
                if(y<top)top=y;
                if(y>bottom)bottom=y;
                break;
            }
        }
        /* End clipping is allowed. A missing outside edge in the MIDDLE
         * is a junction arm, not a continuous measured outgoing straight. */
        for(y=top;y<=bottom;++y) {
            if(measured[y]<0) {++gap;if(gap>max_gap)max_gap=gap;}
            else gap=0;
        }
        if(support<32 || bottom-top<42 || max_gap>6 || support*4<(bottom-top+1)*3 ||
           !inner_fit(measured,top,bottom,&slope,&fit) ||
           fabsf(atanf(-slope)*57.295779513f-expected)>RING_EXIT_FORCE_ALIGN_DEG)continue;
        fit_norm=sqrtf(1.f+slope*slope);centre=fit+side*lane*.5f*fit_norm;
        memset(&candidate,0,sizeof(candidate));candidate.yaw=m->yaw_deg;
        candidate.near_x=candidate.look_x=-1;
        for(y=0;y<RING_H;++y)candidate.mid[y]=candidate.left[y]=candidate.right[y]=-1;
        for(y=117;y>=24;--y) {
            int k,target=(int)floorf(slope*y+centre+.5f),found=0;
            for(k=0;k<row_count[y];++k) {
                run_t r=row_runs[y][k];int e=side>0?r.left:r.right,w=r.right-r.left+1;
                if(!tangent_corridor_check())return 0;
                if(!connected[y][k] || target-lane/6<r.left+2 || target+lane/6>r.right-2)continue;
                if(e>2 && e<RING_W-3 && fabsf(e-(slope*y+fit))>lane/8.f+2.f)continue;
                candidate.mid[y]=(int16_t)target;candidate.left[y]=r.left;candidate.right[y]=r.right;
                ++rows;found=1;
                if(y>=98)++near;
                if(y<=RING_LOOK_ROW && r.left>2 && r.right<RING_W-3 &&
                   w>=lane*(2.f/3.f)*fit_norm && w<=lane*1.5f*fit_norm &&
                   abs((r.left+r.right)/2-target)<=lane/5) {
                    ++streak;if(streak>far)far=streak;
                } else streak=0;
                break;
            }
            if(!found) {streak=0;if(y>=RING_LOOK_ROW && y<=RING_NEAR_ROW)++holes;}
        }
        candidate.near_x=candidate.mid[RING_NEAR_ROW];candidate.look_x=candidate.mid[RING_LOOK_ROW];
        candidate.near_rows=near;candidate.far_rows=far;
        if(rows<45 || near<8 || far<8 || holes || candidate.near_x<0 || candidate.look_x<0)continue;
        if(support+far+near>best) {best=support+far+near;*out=candidate;}
    }
    return best>=0;
}
#endif
