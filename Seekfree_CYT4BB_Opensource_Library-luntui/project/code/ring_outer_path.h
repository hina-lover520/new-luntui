#ifndef CODE_RING_OUTER_PATH_H_
#define CODE_RING_OUTER_PATH_H_

/* Included by ring.c after tangent_normal_path(). This is a CURRENT
 * outside-edge measurement of an already committed ring branch. It may
 * maintain control when the inside edge is clipped by the image border;
 * it cannot recognise a ring, commit entry or confirm an exit. */
typedef struct {
    int valid,route,count,delivered;
    uint32_t episode,ms,frame,delivered_ms;
    float yaw,revs,delivered_yaw,delivered_revs;
    int16_t edge[RING_H];
} ring_outer_history_t;
static ring_outer_history_t ring_outer_history;

static void tangent_outer_reset(void)
{
    memset(&ring_outer_history,0,sizeof(ring_outer_history));
}

static void tangent_outer_remember(const int16_t edge[RING_H],const ring_motion_t *m,int count)
{
    ring_outer_history.valid=1;ring_outer_history.route=ring_result.route;
    ring_outer_history.episode=ring_since;ring_outer_history.ms=m->now_ms;
    ring_outer_history.frame=ring_result.frame_sequence;
    ring_outer_history.yaw=m->yaw_deg;ring_outer_history.revs=m->wheel_revs;
    ring_outer_history.count=count;
    memcpy(ring_outer_history.edge,edge,sizeof(ring_outer_history.edge));
}

static int tangent_outer_fresh(const ring_motion_t *m)
{
    return ring_outer_history.valid && ring_outer_history.delivered &&
        ring_outer_history.episode==ring_since && ring_outer_history.route==ring_result.route &&
        m->now_ms-ring_outer_history.delivered_ms<=RING_PATH_HOLD_MS &&
        fabsf(m->yaw_deg-ring_outer_history.delivered_yaw)<=RING_HOLD_YAW_DEG &&
        fabsf(m->wheel_revs-ring_outer_history.delivered_revs)<=RING_HOLD_REVS;
}

/* Return3 means an associated, measured outside curve. It is distinct
 * from1 (measured inside edge) and2 (bounded weak/extrapolated support). */
static int tangent_outer_path(const ring_motion_t *m)
{
    int16_t edge[RING_H],predicted[RING_H],saved_inner[RING_H];
    int y,k,j,first=117,n=0,curved=0,associated=0,bad=0,normal,side=ring_result.route;
    int count=1,known=0;
    float angle;
    if(!timed_started || maneuver_feedback!=3u || ring_result.phase!=RING_CIRCLE ||
       !ring_result.committed || !ring_result.left_entry || !have_path ||
       seed_identity_lost || (side!=1 && side!=-1) ||
       ring_result.visual_dt_ms==65535u || ring_result.visual_dt_ms>RING_PATH_HOLD_MS) {
        tangent_outer_reset();return 0;
    }
    if(ring_outer_history.valid && (ring_outer_history.episode!=ring_since ||
       ring_outer_history.route!=side)) tangent_outer_reset();
    for(y=0;y<RING_H;++y)edge[y]=predicted[y]=-1;
    /* The CURRENT acquired bottom component supplies both row boundaries.
     * Do not jump past a clipping/discontinuity to a background edge. */
    for(y=117;y>=24;--y) {
        int l=observed_left[y],r=observed_right[y],e=side>0?r:l;
        if(l<0 || e<=2 || e>=RING_W-3 ||
           (y<117 && abs(e-edge[y+1])>8))break;
        edge[y]=(int16_t)e;first=y;++n;
    }
    if(n<30 || first>88 || observed_left[RING_NEAR_ROW]<0 ||
       observed_right[117]-observed_left[117]+1<ring_result.lane_width*2/3 ||
       observed_right[117]-observed_left[117]+1>ring_result.lane_width*5/2)return 0;
    for(y=first;y+24<=117 && !curved;++y)for(k=y+24;k<=117;k+=4) {
        int mid=(y+k)/2;
        float chord=edge[y]+(edge[k]-edge[y])*(float)(mid-y)/(k-y);
        if(side*(edge[mid]-chord)>=3.f){curved=1;break;}
    }
    /* A straight/cross replacement cannot renew a ring from its yaw alone. */
    if(!curved)return 0;
    if(ring_outer_history.valid && m->now_ms-ring_outer_history.ms<=RING_PATH_HOLD_MS &&
       fabsf(m->yaw_deg-ring_outer_history.yaw)<=RING_HOLD_YAW_DEG &&
       fabsf(m->wheel_revs-ring_outer_history.revs)<=RING_HOLD_REVS) {
        if(ring_outer_history.frame==ring_result.frame_sequence)return 0;
        angle=(m->yaw_deg-ring_outer_history.yaw)*.01745329252f;
        for(y=24;y<=117;++y)if(ring_outer_history.edge[y]>2) {
            float x=ring_outer_history.edge[y]-RING_W/2,f=119-y;
            int yy=(int)floorf(119-(sinf(angle)*x+cosf(angle)*f)+.5f);
            int xx=(int)floorf(RING_W/2+cosf(angle)*x-sinf(angle)*f+.5f);
            if(yy>=24 && yy<=117 && xx>2 && xx<RING_W-3)predicted[yy]=(int16_t)xx;
        }
        for(y=first;y<=117;++y) {
            int p=-1;
            for(k=y-2;k<=y+2;++k)if(k>=24 && k<=117 && predicted[k]>=0 &&
                (p<0 || abs(k-y)<abs(p-y)))p=k;
            if(p>=0){++associated;if(abs(edge[y]-predicted[p])>RING_CORRIDOR_TOL/2)++bad;}
        }
        known=associated>=20 && bad*4<=associated;
        if(known)count=ring_outer_history.count<2?ring_outer_history.count+1:2;
    }
    if(!known && m->now_ms-last_path_fault_ms>RING_PATH_HOLD_MS)return 0;
    tangent_outer_remember(edge,m,count);
    if(count<2)return 0;
    memcpy(saved_inner,inner,sizeof(saved_inner));memcpy(inner,edge,sizeof(inner));
    /* Use the same two-dimensional normal construction, pointing inward
     * from the outside edge. Restore branch identity before returning. */
    ring_result.route=(int8_t)-side;normal=tangent_normal_path();ring_result.route=(int8_t)side;
    memcpy(inner,saved_inner,sizeof(inner));
    if(!normal)return 0;
    for(y=24;y<=117;++y)if(ring_result.mid[y]>=0) {
        int x=ring_result.mid[y],margin=ring_result.lane_width/8,white=0;
        if(margin<2)margin=2;
        for(j=0;j<row_count[y];++j)if(connected[y][j] &&
            row_runs[y][j].left==observed_left[y] && row_runs[y][j].right==observed_right[y] &&
            x-margin>=row_runs[y][j].left+2 && x+margin<=row_runs[y][j].right-2) {
            white=1;break;
        }
        if(!white){ring_result.path_valid=0;return 0;}
    }
    for(y=0;y<RING_H;++y) {
        lane_left[y]=ring_result.mid[y]>=0?observed_left[y]:-1;
        lane_right[y]=ring_result.mid[y]>=0?observed_right[y]:-1;
    }
    ring_result.inner_rows=0;ring_result.associated_rows=(uint8_t)associated;
    return 3;
}

/* Call AFTER tangent_store_path(m,0) for source3 only. This refreshes
 * delivered control from current measurements without replacing the
 * strong INSIDE-edge identity or granting entry/exit evidence. */
static void tangent_outer_store(const ring_motion_t *m)
{
    ring_outer_history.delivered=1;ring_outer_history.delivered_ms=m->now_ms;
    ring_outer_history.delivered_yaw=m->yaw_deg;ring_outer_history.delivered_revs=m->wheel_revs;
    last_path_fault_ms=m->now_ms;
    support_frames=0;support_travel=0;support_last_revs=m->wheel_revs;
    seed_left=observed_left[117];seed_right=observed_right[117];
    seed_x=ring_result.mid[117];
    seed_inner=ring_result.route>0?seed_left:seed_right;
}
#endif
