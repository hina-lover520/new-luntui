#ifndef CODE_RING_EXIT_OPENING_H_
#define CODE_RING_EXIT_OPENING_H_

/* Included by ring.c after exit_probe_t and the cached row graph exist.
 * This is an OBSERVATION of a sideward outgoing straight, not a steering
 * path. In particular, line anchors can lie outside the camera. Never
 * pass this result to tangent_exit_apply() or infer path_valid from it.
 * Only cached runs are read; image pixels and graph ownership are intact. */
#ifdef RING_HOST_TEST
static uint32_t ring_opening_cache_checks;
#endif
/* This optional observer runs only on the final-lap angular window. Bound
 * its cached work as well as its samples: a noisy many-run image cannot
 * turn one new feature into an unbounded visual-frame delay. Exhaustion
 * means unknown geometry, with no candidate or path credit. */
static unsigned ring_opening_checks_remaining;
static int tangent_opening_check(void)
{
    if(!ring_opening_checks_remaining)return 0;
    --ring_opening_checks_remaining;
#ifdef RING_HOST_TEST
    ++ring_opening_cache_checks;
#endif
    return 1;
}
static int tangent_opening_white(float xf,float yf,int required)
{
    int x=(int)floorf(xf+.5f),y=(int)floorf(yf+.5f),j;
    if(!tangent_opening_check())return -1;
    if(x<1 || x>RING_W-2 || y<24 || y>117)return -1;
    for(j=0;j<row_count[y];++j) {
        if(!tangent_opening_check())return -1;
        if(x>=row_runs[y][j].left && x<=row_runs[y][j].right)
            return !required || connected[y][j];
    }
    return 0;
}

/* Distance to a CURRENT measured dark boundary along the ray's normal.
 * Returning0 means clipped/unknown, not a fabricated black edge. */
static int tangent_opening_edge(float x,float y,float nx,float ny,int lane)
{
    int d;
    for(d=2;d<=lane*2;d+=2) {
        int white=tangent_opening_white(x+nx*d,y+ny*d,0);
        if(white<0)return 0;
        if(!white)return d;
    }
    return 0;
}

static int tangent_exit_opening(exit_probe_t *out,const ring_motion_t *m)
{
    enum { OPENING_SAMPLES=49 };
    int anchor,lane=ring_result.lane_width,side=ring_result.ring_side;
    int chosen=0,best_score=-1;
    float remaining=RING_EXIT_MIN_DEG-ring_result.lap_deg;
    float a,s,c,best_anchor=0.f;
#ifdef RING_HOST_TEST
    ring_opening_cache_checks=0;
#endif
    ring_opening_checks_remaining=48000u;
    if(!ring_result.committed || !ring_result.lap_valid || !timed_started ||
       maneuver_feedback!=3u || (side!=1 && side!=-1) ||
       ring_result.visual_dt_ms==65535u || lane<RING_MIN_LANE_WIDTH || lane>RING_MAX_LANE_WIDTH ||
       (RING_EXIT_NET_MIN_DEG>0.f?
        (!exit_net_reached || ring_result.lap_deg<RING_EXIT_MIN_DEG || ring_result.lap_deg>RING_EXIT_SEARCH_END_DEG ||
         fabsf(remaining)>RING_EXIT_FORCE_DEG):
        (remaining<=0.f || remaining>RING_EXIT_FORCE_DEG)) ||
       ring_result.phase!=RING_CIRCLE)return 0;
    a=side*remaining*.01745329252f;s=sinf(a);c=cosf(a);
    if(c<.3f)return 0;
    /* The outgoing lane need not pass through the vehicle centre. Its
     * base must still lie in the SAME acquired near-road component. */
    for(anchor=RING_W/2-lane;anchor<=RING_W/2+lane;anchor+=6) {
        int outside[OPENING_SAMPLES],inside[OPENING_SAMPLES];
        int sample,first=-1,last=-1,strip=0,longest=0;
        int outer_min=10000,outer_max=0,outer_count=0,outer_first=-1,outer_last=-1;
        int narrow=0,narrow_max=0,bulge=0,score;
        float margin=lane*.25f;
        if(tangent_opening_white((float)anchor,117.f,1)!=1)continue;
        for(sample=0;sample<OPENING_SAMPLES;++sample) {
            float t=sample*4.f,x=anchor+s*t,y=117.f-c*t;
            int centre=tangent_opening_white(x,y,1),left,right;
            outside[sample]=inside[sample]=0;
            if(centre!=1)break;
            left=tangent_opening_white(x-c*margin,y-s*margin,1);
            right=tangent_opening_white(x+c*margin,y+s*margin,1);
            if(left==1 && right==1) {
                ++strip;if(strip>longest)longest=strip;
                if(first<0)first=sample;
                last=sample;
            } else strip=0;
            if(left<0 || right<0)continue;
            outside[sample]=tangent_opening_edge(x,y,-side*c,-side*s,lane);
            inside[sample]=tangent_opening_edge(x,y,side*c,side*s,lane);
            if(outside[sample]>=lane/4 && outside[sample]<=lane*3/4 &&
               left==1 && right==1) {
                int edge=outside[sample];
                if(edge<outer_min)outer_min=edge;
                if(edge>outer_max)outer_max=edge;
                if(outer_first<0)outer_first=sample;
                outer_last=sample;++outer_count;
                if(inside[sample]>=lane/4 &&
                   inside[sample]+edge>=lane*2/3 && inside[sample]+edge<=lane*3/2) {
                    ++narrow;if(narrow>narrow_max)narrow_max=narrow;
                } else narrow=0;
            } else narrow=0;
        }
        /* A diagonal through an ordinary straight/cross arm is too short
         * or has a moving outside boundary. The entire measured outside
         * segment must remain straight, not just a single far white ray. */
        if(first<0 || last-first<lane/4 || longest*4<lane ||
           outer_count<7 || (outer_last-outer_first)*4<lane ||
           outer_max-outer_min>4 || narrow_max*4<lane/3)continue;
        /* The selected INNER side must show a continuous convex dark
         * island boundary. A rectangular T/cross corner has a step then
         * a flat edge; it cannot supply this measured inward bow. */
        for(sample=outer_first;sample+8<=outer_last && !bulge;++sample) {
            int end;
            for(end=sample+8;end<=outer_last;++end) {
                int k,valid=1,mid=(sample+end)/2;
                float chord;
                if(!inside[sample] || !inside[end] || !inside[mid])continue;
                for(k=sample+1;k<=end;++k) {
                    if(!tangent_opening_check())return 0;
                    if(!inside[k] || abs(inside[k]-inside[k-1])>lane/4+2){valid=0;break;}
                }
                if(!valid)continue;
                chord=inside[sample]+(inside[end]-inside[sample])*
                    (float)(mid-sample)/(end-sample);
                if(chord-inside[mid]>=3.f){bulge=1;break;}
            }
        }
        if(!bulge)continue;
        score=longest*2+outer_count-(outer_max-outer_min)*4-
            abs((outer_min+outer_max)/2-lane/2)*12;
        if(score>best_score){best_score=score;best_anchor=(float)anchor;chosen=1;}
    }
    if(!ring_opening_checks_remaining)return 0;
    if(!chosen)return 0;
    memset(out,0,sizeof(*out));out->yaw=m->yaw_deg;
    for(anchor=0;anchor<RING_H;++anchor) {
        int j,x=(int)floorf(best_anchor+(117-anchor)*s/c+.5f);
        out->mid[anchor]=out->left[anchor]=out->right[anchor]=-1;
        if(anchor<24 || anchor>117 || x<1 || x>RING_W-2)continue;
        for(j=0;j<row_count[anchor];++j) {
            if(!tangent_opening_check())return 0;
            if(connected[anchor][j] && x>=row_runs[anchor][j].left+2 && x<=row_runs[anchor][j].right-2) {
            out->mid[anchor]=(int16_t)x;
            out->left[anchor]=row_runs[anchor][j].left;
            out->right[anchor]=row_runs[anchor][j].right;
            if(anchor>=98)++out->near_rows;
            else ++out->far_rows;
            break;
            }
        }
    }
    out->near_x=(int)floorf(best_anchor+(117-RING_NEAR_ROW)*s/c+.5f);
    out->look_x=(int)floorf(best_anchor+(117-RING_LOOK_ROW)*s/c+.5f);
    return 1;
}

/* Associate LINE OBSERVATIONS rather than requiring all four old control
 * rows to be in view. Rotation preserves normal distance to a line. No
 * translated pixels, guessed target points or control credit are added. */
static int tangent_exit_opening_associated(const exit_probe_t *old,const exit_probe_t *now)
{
    float oa=atan2f((float)(old->look_x-old->near_x),
        (float)(RING_NEAR_ROW-RING_LOOK_ROW));
    float na=atan2f((float)(now->look_x-now->near_x),
        (float)(RING_NEAR_ROW-RING_LOOK_ROW));
    float expected=oa-(now->yaw-old->yaw)*.01745329252f;
    float od=cosf(oa)*(old->near_x-RING_W/2)-sinf(oa)*(119-RING_NEAR_ROW);
    float nd=cosf(na)*(now->near_x-RING_W/2)-sinf(na)*(119-RING_NEAR_ROW);
    return fabsf(na-expected)<=RING_EXIT_FORCE_ALIGN_DEG*.01745329252f &&
        fabsf(nd-od)<=ring_result.lane_width/4;
}

#endif
