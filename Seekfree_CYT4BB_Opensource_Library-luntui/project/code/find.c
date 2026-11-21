/*
 * find.c
 *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
 *  Created on: 2025??4??26??
 *      Author: 39653
 */
#include "find.h"
#include "image.h"
#include "string.h"
#include "zf_device_mt9v03x.h"
#include "globals.h"

#define TRACK_H                 MT9V03X_H           /* image rows (120) */
#define TRACK_W                 MT9V03X_W           /* image cols (188) */
#define CROSS_WIDE_MIN          120                 /* min white-run width(px) -> cross band; tune to your track */
#define CROSS_LOST_MARGIN         3                 /* edge "lost" if within this many px of the image edge */
#define CROSS_MIN_ROWS            6                 /* min consecutive both-edges-lost rows to call a cross */
#define CROSS_MIN_LANE_W         4                  /* min plausible normal lane width(px) */
#define CROSS_HALF_MAX           30                 /* clamp filled lane half-width(px), prevent completion blow-up */
#define CROSS_HOLD_FRAMES         5                 /* frames to hold cross state after last detection (anti-flicker) */
#define CROSS_FIT_ROWS           10                 /* rows below the bar for the lane-heading least-squares fit */
#define CROSS_SLOPE_MAX          3.0f               /* clamp fitted lane slope (px/row), guards against noise */
#define STEER_BIAS               1                  /* px; >0 shifts reported centre LEFT (fixes 'car runs right'), <0 shifts RIGHT */
#define STEER_LOOKAHEAD_BASE     6                  /* 最底采样行(倒数第 6 行), 贴线对齐用 */
#define STEER_LOOKAHEAD_STEP     6                  /* 底部采样行间隔(行) */
#define STEER_LOOKAHEAD_N        3                  /* 底部采样行数(>=2, 只取图像底部, 不上半段) */
#define STEER_LOOKAHEAD_ROWS     12.0f              /* 前瞻投影距离(行): 误差 += 斜率(px/行) × 该值; 左右晃/急弯甩出就调小 */

uint8 mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W];
uint8 mt9v03x_image_project[MT9V03X_H][MT9V03X_W];
uint8 display_image[MT9V03X_H][MT9V03X_W];
uint8_t labelBuffer[MT9V03X_W*MT9V03X_H]={0};//??????
uint8 threshold;
int cnt[MT9V03X_W];
uint8 max;
uint8 max_raw;
int16 track_width[TRACK_H];
static int16  g_left[TRACK_H];            /* left edge x                */
static int16  g_right[TRACK_H];           /* right edge x               */
static int16  g_mid[TRACK_H];             /* midline x                  */
static int16  g_vleft[TRACK_H];           /* left virtual edge x        */
static int16  g_vright[TRACK_H];          /* right virtual edge x       */
int16_t g_left_g;                         /* ʮ�֣�ʮ���е���߽�           */
int16_t g_right_g;                        /* ʮ�֣�ʮ���е��ұ߽�           */
int16_t g_hight_g;                        /* ʮ�֣�ʮ��������(�������)      */
static int16_t g_cross_top_g;             /* ʮ�ִ�����(Զ)�кţ����ڲ������� */
static uint8  g_mid_col;                 /* ʮ�������У�ƫ��Ļ�м䡢�׵�϶ࣩ */
static uint8  g_cross_hold = 0;          /* anti-flicker: frames left to hold the cross state */
int16_t g_center_col;              /* chosen track centre column: cross/roundabout centroid, else longest white column */
int16_t g_center_raw;              /* normal steering centre: raw-image white-line midline (bottom rows) */
int16_t g_center_far;              /* look-ahead steering centre: raw-image white-line midline at a farther row */
static float g_lookahead_slope;    /* 底部采样行拟合出的中线斜率(px/行), 用于前瞻 */
float QQQQ;
float count19;
float count20;
float count21;

float screen_x;
float screen_y;
int xx=0;//???X????
int yy=0;//?????????
void Set_project(void)
{
    for (uint8 i = 0; i < MT9V03X_H; i++)            // rows 0..119
    {
        for (uint8 j = 0; j < MT9V03X_W; j++)        // cols 0..187
        {
            mt9v03x_image_project[i][j] = mt9v03x_image[i][j];
        }
    }

    for (uint8 n = 0; n < MT9V03X_W; n++)            // top & bottom border
    {
        mt9v03x_image_project[0][n] = 0;
        mt9v03x_image_project[MT9V03X_H - 1][n] = 0;
    }

    for (uint8 m = 0; m < MT9V03X_H; m++)            // left & right border
    {
        mt9v03x_image_project[m][0] = 0;
        mt9v03x_image_project[m][MT9V03X_W - 1] = 0;
    }
}

void find_amax(unsigned char imageIn[MT9V03X_H][MT9V03X_W])
{
    uint8 i, j;
    threshold = (uint8)GetOSTU(imageIn);   // Otsu threshold

    for (j = 0; j < MT9V03X_W; j++)
    {
        cnt[j] = 0;
        for (i = 0; i < MT9V03X_H; i++)          // from bottom row upward
        {
            if (imageIn[MT9V03X_H - 1 - i][j] > threshold)
                cnt[j]++;
            else
                break;
        }
    }

    max = 0;
    for (j = 1; j < MT9V03X_W; j++)
    {
        if (cnt[j] > cnt[max])
            max = j;
    }
    {   /* centre of the tied-longest columns (not the leftmost), so max
           reports the line centre instead of its left edge */
        uint8 j0 = max, j1 = max;
        while (j0 > 0 && cnt[j0 - 1] == cnt[max]) j0--;
        while (j1 < MT9V03X_W - 1 && cnt[j1 + 1] == cnt[max]) j1++;
        max = (uint8)((j0 + j1) / 2);
    }
}

/* Longest white column of the RAW camera image (perspective, bottom-anchored).
   Normal-driving steering centre: reads the raw frame directly (NOT the IPM
   bird-eye), so it is the track column right under the car. Uses a local Otsu
   threshold and its own max_raw; does not touch global threshold/max. */
void find_amax_raw(void)
{
    uint8 th = (uint8)GetOSTU(mt9v03x_image);
    uint8 i, j;
    for (j = 0; j < MT9V03X_W; j++)
    {
        cnt[j] = 0;
        for (i = 0; i < MT9V03X_H; i++)
        {
            if (mt9v03x_image[MT9V03X_H - 1 - i][j] > th)
                cnt[j]++;
            else
                break;
        }
    }
    max_raw = 0;
    for (j = 1; j < MT9V03X_W; j++)
    {
        if (cnt[j] > cnt[max_raw])
            max_raw = j;
    }
    {   /* centre of the tied-longest columns (not the leftmost) */
        uint8 j0 = max_raw, j1 = max_raw;
        while (j0 > 0 && cnt[j0 - 1] == cnt[max_raw]) j0--;
        while (j1 < MT9V03X_W - 1 && cnt[j1 + 1] == cnt[max_raw]) j1++;
        max_raw = (uint8)((j0 + j1) / 2);
    }
}

/* --------------------------------------------------------------------------
 * find_mid_raw(): normal steering centres from the RAW camera image.
 *   只用图像底部的几个采样行(不上半段, 上半中线易出界):
 *     各采样行用中线 mid_at_row 取白线中心, 无车道则回退最长白列 max_raw;
 *     对 (深度, 中心) 做最小二乘拟合出斜率 g_lookahead_slope (px/行);
 *     g_center_raw = 最底采样行中心(贴线对齐);
 *     g_center_far  = 最底中心 + 斜率 × STEER_LOOKAHEAD_ROWS(前瞻投影, 供显示/调试).
 * -------------------------------------------------------------------------- */
static int16_t mid_at_row(uint8 th, int row)
{
    int left = -1, right = -1;
    for (int j = 0; j < MT9V03X_W; j++)
        if (mt9v03x_image[row][j] > th) { left = j; break; }
    for (int j = MT9V03X_W - 1; j >= 0; j--)
        if (mt9v03x_image[row][j] > th) { right = j; break; }

    if (left >= 0 && right >= left && (right - left) < CROSS_WIDE_MIN)
        return (int16_t)((left + right) / 2);
    return -1;
}

void find_mid_raw(void)
{
    uint8 th = (uint8)GetOSTU(mt9v03x_image);

    /* 底部几个采样行的中线 (depth = 距底部的行数, 越上越大 = 越靠前) */
    int16_t c[STEER_LOOKAHEAD_N];
    for (int k = 0; k < STEER_LOOKAHEAD_N; k++)
    {
        int row = MT9V03X_H - STEER_LOOKAHEAD_BASE - k * STEER_LOOKAHEAD_STEP;
        if (row < 1) row = 1;
        if (row > MT9V03X_H - 2) row = MT9V03X_H - 2;
        int16_t m = mid_at_row(th, row);
        c[k] = (m >= 0) ? m : (int16_t)max_raw;
    }

    g_center_raw = c[0];   /* 最底采样行中心(贴线对齐) */

    /* 最小二乘拟合斜率: 中心(px) 随 depth(行) 的变化率 -> 车道方向 */
    {
        float sx = 0.0f, sy = 0.0f, sxy = 0.0f, sxx = 0.0f;
        for (int k = 0; k < STEER_LOOKAHEAD_N; k++)
        {
            float x = (float)(STEER_LOOKAHEAD_BASE + k * STEER_LOOKAHEAD_STEP);  /* depth from bottom */
            float y = (float)c[k];
            sx += x; sy += y; sxy += x * y; sxx += x * x;
        }
        float n = (float)STEER_LOOKAHEAD_N;
        float denom = n * sxx - sx * sx;
        g_lookahead_slope = (denom != 0.0f) ? (n * sxy - sx * sy) / denom : 0.0f;
    }

    /* 前瞻投影中心(供显示/调试), 实际误差计算见 track_steer_error */
    g_center_far = (int16_t)((float)g_center_raw + g_lookahead_slope * STEER_LOOKAHEAD_ROWS);
}

void find_LR(unsigned char imageIn[MT9V03X_H][MT9V03X_W], uint8 get_max)
{
    for (int i = 1; i < MT9V03X_H - 1; i++)
    {
        g_left[i] = 0;
        for (int j = get_max; j >= 0; j--)
        {
            if (imageIn[i][j] == 0)
            {
                g_left[i] = j + 1;
                break;
            }
        }
        g_right[i] = MT9V03X_W - 1;
        for (int j = get_max; j < MT9V03X_W; j++)
        {
            if (imageIn[i][j] == 0)
            {
                g_right[i] = j - 1;
                break;
            }
        }
        /* �������߲��ߣ���߽�̫�����Ե(��15)ʱ���������ұ߽����м���15���� */
        /* ���ұ߽��ȡ����ֹ g_left=188 / g_right=-1 ����Խ�紥����Ļ���� */
        if (g_left[i]  < 0)               g_left[i]  = 0;
        if (g_left[i]  > MT9V03X_W - 1)   g_left[i]  = MT9V03X_W - 1;
        if (g_right[i] < 0)               g_right[i] = 0;
        if (g_right[i] > MT9V03X_W - 1)   g_right[i] = MT9V03X_W - 1;

        if (g_left[i] <= 15)                                    /* ��߽綪�� �� ���ұ߽�������60 */
        {
            g_mid[i] = g_right[i] - 60;
        }
        else if (g_right[i] >= MT9V03X_W - 1 - 15)              /* �ұ߽綪�� �� ����߽�������60 */
        {
            g_mid[i] = g_left[i] + 60;
        }
        else
        {
            g_mid[i] = (g_left[i] + g_right[i]) / 2;
        }

        /* ���߽߱��ȡ����ֹ���ŵ���ĻʱԽ�紥������ */
        if (g_mid[i] < 0)               g_mid[i] = 0;
        if (g_mid[i] > MT9V03X_W - 1)   g_mid[i] = MT9V03X_W - 1;
    }
}

void binarize(unsigned char imageIn[MT9V03X_H][MT9V03X_W])
{
    int i, j;
    for (i = 0; i < MT9V03X_H; i++)
        for (j = 0; j < MT9V03X_W; j++)
            mt9v03x_image_BandW[i][j] = (imageIn[i][j] > threshold) ? 255 : 0;
}

void show_track_line(void)
{
    for (int i = 1; i < MT9V03X_H - 1; i++)
    {
        uint16 sy = (uint16)(i * 160 / MT9V03X_H);
        ips200_draw_point((uint16)(g_left[i] * 240 / MT9V03X_W), sy, RGB565_RED);
        ips200_draw_point((uint16)(g_right[i] * 240 / MT9V03X_W), sy, RGB565_BLUE);
        ips200_draw_point((uint16)(g_mid[i] * 240 / MT9V03X_W), sy, RGB565_GREEN);
    }

    /* ���ԣ���⵽ʮ��ʱ����ʮ���л�һ�����ɫ���ߣ��۲첹���Ƿ���Ч */
    if (g_hight_g >= 0)
    {
        uint16 sy = (uint16)(g_hight_g * 160 / MT9V03X_H);
        ips200_draw_line(0, sy, 239, sy, RGB565_MAGENTA);
    }

    /* debug: row1=max,bottom,top row2=below ref(L,R,hw) row3=filled(L,R,M) */
    if (g_hight_g >= 1 && g_hight_g < MT9V03X_H - 1)
    {
        int16_t below = g_hight_g + 1;
        if (below > MT9V03X_H - 2) below = g_hight_g;

        ips200_show_int(0, 272, g_mid_col, 3);                  /* ��������(ƫ���İ׵�϶�) */
        ips200_show_int(40, 272, g_hight_g, 3);           /* ���� */
        ips200_show_int(80, 272, g_cross_top_g, 3);       /* ���� */

        ips200_show_int(0, 288, g_left[below], 3);       /* �ο��� */
        ips200_show_int(40, 288, g_right[below], 3);     /* �ο��� */
        ips200_show_int(80, 288, (g_right[below] - g_left[below]) / 2, 3); /* ��� */

        ips200_show_int(0, 304, g_left[g_hight_g], 3);   /* ������ */
        ips200_show_int(40, 304, g_right[g_hight_g], 3); /* ������ */
        ips200_show_int(80, 304, g_mid[g_hight_g], 3);   /* ������(=��������) */
    }
}



/*****************************************************************************
 * arrive_cross(): detect a cross (4-way intersection) and "fill" it with a
 * straight vertical lane so the car keeps going straight through it.
 *
 * Standard smart-car method (corner point / both-edges-lost):
 *   1. The cross's horizontal bar = the longest contiguous run of rows where
 *      BOTH edges are lost (white reaches both image edges).
 *   2. Confirm a real cross: the narrow vertical lane must still exist just
 *      ABOVE and just BELOW that bar (edges present on both sides).
 *   3. Steering centre = the vertical-lane centre extrapolated straight
 *      through the bar (near lane first, far lane fallback, then the longest
 *      white column). No 8-neighbourhood centroid: the wide bar biases it.
 *****************************************************************************/
void arrive_cross(void)
{
    int16_t bar_far   = -1;      /* cross bar top (far) row     */
    int16_t bar_near  = -1;      /* cross bar bottom (near) row */
    int16_t run_start = -1;
    int16_t run_len   = 0;
    int16_t best_len  = 0;

    /* 1) longest run of rows with BOTH edges lost (white spans the frame). */
    for (int i = MT9V03X_H - 2; i >= 1; i--)       /* bottom -> top */
    {
        int lost = (g_left[i]  <= CROSS_LOST_MARGIN) &&
                   (g_right[i] >= MT9V03X_W - 1 - CROSS_LOST_MARGIN);
        if (lost)
        {
            if (run_len == 0) run_start = (int16_t)i;   /* near end */
            run_len++;
            if (run_len > best_len)
            {
                best_len = run_len;
                bar_near = run_start;
                bar_far  = (int16_t)i;
            }
        }
        else
        {
            run_len = 0;
        }
    }

    if (best_len < CROSS_MIN_ROWS)
    {
        /* no bar this frame: hold the last cross state a few frames so a
           detection flicker does not drop back to normal steering (which can
           follow a side arm at the cross and turn the car out). */
        if (g_cross_hold > 0) { g_cross_hold--; return; }
        g_hight_g     = -1;
        g_cross_top_g = -1;
        g_left_g      = -1;
        g_right_g     = -1;
        g_mid_col     = 0;
        return;
    }

    /* 2) the narrow vertical lane must continue on at least one side. */
    int ok_above = 0, ok_below = 0;
    int ia = bar_far  - 1;
    int ib = bar_near + 1;
    if (ia >= 1 && ia < MT9V03X_H - 1)
    {
        int w = g_right[ia] - g_left[ia];
        ok_above = (w >= CROSS_MIN_LANE_W && w < CROSS_WIDE_MIN &&
                    g_left[ia]  > CROSS_LOST_MARGIN &&
                    g_right[ia] < MT9V03X_W - 1 - CROSS_LOST_MARGIN);
    }
    if (ib >= 1 && ib < MT9V03X_H - 1)
    {
        int w = g_right[ib] - g_left[ib];
        ok_below = (w >= CROSS_MIN_LANE_W && w < CROSS_WIDE_MIN &&
                    g_left[ib]  > CROSS_LOST_MARGIN &&
                    g_right[ib] < MT9V03X_W - 1 - CROSS_LOST_MARGIN);
    }
    if (!ok_above && !ok_below)
    {
        if (g_cross_hold > 0) { g_cross_hold--; return; }
        g_hight_g     = -1;
        g_cross_top_g = -1;
        g_left_g      = -1;
        g_right_g     = -1;
        g_mid_col     = 0;
        return;                      /* no lane on either side -> not a cross */
    }

    /* 3) steering centre: least-squares fit the incoming lane centre (rows just
           below the bar) and extrapolate that heading straight through the bar.
           A fitted slope follows the lane's real direction, so a slightly-angled
           entry still exits the cross straight (vs a constant centre that keeps
           the entry angle). */
    int16_t ref_hw = -1;
    float   slope  = 0.0f;
    float   icept  = (float)(MT9V03X_W / 2);
    int     fitted = 0;

    {
        int n = 0;
        float sx = 0.0f, sy = 0.0f, sxy = 0.0f, sxx = 0.0f;
        int w_sum = 0;
        int i;
        for (i = bar_near + 1; i <= bar_near + CROSS_FIT_ROWS && i < MT9V03X_H - 1; i++)
        {
            int w = g_right[i] - g_left[i];
            if (w >= CROSS_MIN_LANE_W && w < CROSS_WIDE_MIN)
            {
                float x = (float)i;
                float y = (float)g_mid[i];
                sx  += x;
                sy  += y;
                sxy += x * y;
                sxx += x * x;
                w_sum += w;
                n++;
            }
        }
        if (n >= 2)
        {
            float den = (float)n * sxx - sx * sx;
            if (den != 0.0f)
            {
                slope = ((float)n * sxy - sx * sy) / den;
                icept = (sy - slope * sx) / (float)n;
                ref_hw = (w_sum / n) / 2;
                fitted = 1;
            }
        }
    }

    if (slope >  CROSS_SLOPE_MAX) slope =  CROSS_SLOPE_MAX;
    if (slope < -CROSS_SLOPE_MAX) slope = -CROSS_SLOPE_MAX;

    int16_t mid_col;
    if (fitted)
    {
        mid_col = (int16_t)(slope * (float)bar_near + icept);   /* heading at the near edge of the bar */
    }
    else
    {
        /* fallback: average near lane first, far lane second, longest white col last */
        int16_t ref_mid = -1;
        for (int dir = 0; dir < 2 && ref_mid < 0; dir++)
        {
            int i  = (dir == 0) ? (bar_near + 1) : (bar_far - 1);
            int st = (dir == 0) ? (+1) : (-1);
            int cnt = 0, mid_sum = 0, w_sum2 = 0;
            for (; i >= 1 && i < MT9V03X_H - 1 && cnt < 3; i += st)
            {
                int w = g_right[i] - g_left[i];
                if (w >= CROSS_MIN_LANE_W && w < CROSS_WIDE_MIN)
                {
                    mid_sum += g_mid[i];
                    w_sum2  += w;
                    cnt++;
                }
                else if (cnt > 0)
                    break;
            }
            if (cnt > 0)
            {
                ref_mid = mid_sum / cnt;
                ref_hw  = (w_sum2 / cnt) / 2;
            }
        }
        if (ref_mid >= 0) mid_col = ref_mid;
        else              mid_col = (int16_t)max;
        if (mid_col < 0)  mid_col = MT9V03X_W / 2;
    }

    if (ref_hw < 0)  ref_hw = 15;
    if (ref_hw < 3)  ref_hw = 3;
    if (ref_hw > CROSS_HALF_MAX) ref_hw = CROSS_HALF_MAX;

    if (mid_col < 0)             mid_col = 0;
    if (mid_col > MT9V03X_W - 1) mid_col = MT9V03X_W - 1;

    g_hight_g     = bar_near;
    g_cross_top_g = bar_far;
    g_mid_col     = (uint8)mid_col;
    g_center_col  = mid_col;
    g_left_g      = mid_col - ref_hw;
    g_right_g     = mid_col + ref_hw;
    g_cross_hold  = CROSS_HOLD_FRAMES;         /* keep cross state -> no flicker */

    /* 4) fill the bar with a straight lane that follows the fitted heading. */
    for (int i = bar_far; i <= bar_near; i++)
    {
        int c = (int)(slope * (float)i + icept);
        if (c < 0)             c = 0;
        if (c > MT9V03X_W - 1) c = MT9V03X_W - 1;

        g_mid[i]   = c;
        g_left[i]  = c - ref_hw;
        g_right[i] = c + ref_hw;

        if (g_left[i]  < 0)                g_left[i]  = 0;
        if (g_left[i]  > MT9V03X_W - 1)    g_left[i]  = MT9V03X_W - 1;
        if (g_right[i] < 0)                g_right[i] = 0;
        if (g_right[i] > MT9V03X_W - 1)    g_right[i] = MT9V03X_W - 1;
    }
}



/* --------------------------------------------------------------------------
 * line-follow output helpers (called from core0 main loop, then written into
 * the dual-core shared memory for core1).
 *   track_steer_error(): near midline error + far look-ahead error (px).
 *                        >0 = track on the right.
 *   track_state():       0 = lost, 1 = normal straight, 2 = cross detected.
 * -------------------------------------------------------------------------- */
int16_t track_steer_error(void)
{
    /* Steering error (px) = near alignment error + bottom-slope look-ahead:
       - cross / roundabout: junction centroid (arrive_cross -> g_center_col),
         no look-ahead (keep straight through the cross).
       - normal track: near midline (g_center_raw) for alignment + local
         bottom slope (g_lookahead_slope × ROWS) as look-ahead for curves. */
    if (g_hight_g >= 0)
    {
        return (int16_t)(g_center_col - STEER_BIAS - (MT9V03X_W / 2));
    }

    int16_t near_err = (int16_t)(g_center_raw - STEER_BIAS - (MT9V03X_W / 2));
    int16_t look_err = (int16_t)(g_lookahead_slope * STEER_LOOKAHEAD_ROWS);   /* 底部斜率投影 -> 前瞻 */
    return (int16_t)(near_err + look_err);
}

uint8_t track_state(void)
{
    if (g_hight_g >= 0)
        return 2;                           /* cross */

    /* lost: the near rows have almost no white pixels (track not visible) */
    int w = 0;
    int r0 = MT9V03X_H - 12;
    if (r0 < 1) r0 = 1;

    for (int i = r0; i <= MT9V03X_H - 2; i++)
        for (int j = 0; j < MT9V03X_W; j++)
            if (mt9v03x_image_BandW[i][j] != 0)
                w++;

    if (w < 40)                             /* tune: min white-pixel count near car */
        return 0;                           /* lost */

    return 1;                               /* normal */
}


void Set_image_twovalues(uint8 value)   ////???????????????
{
    for(uint8 i=0;i<MT9V03X_H;i++)
    {

        for(uint8 j=0;j<MT9V03X_W;j++)
        {

                 // ?????????
                  mt9v03x_image_BandW[i][j] = (mt9v03x_image[i][j] >= value) ? 1 : 0;
}
}
                  erosion_operation(mt9v03x_image_BandW);//??????
                 // ??????????
      for(uint8 l=0;l<MT9V03X_H;l++)
        {
           for(uint8 k=0;k<MT9V03X_W;k++)
              {
                  display_image[l][k] = mt9v03x_image_BandW[l][k] * 255;
              }
        }
}

volatile uint8_t dotcnt=0;
volatile uint8_t dotlie[188];//?????????????????????????��???????????????????
volatile uint8_t dothang[120];//???
volatile short sumlie=0,sumhang=0;


void Seek_Beacon ()//??????
{
  uint8_t tm=0;
  uint8_t nr=0; //??
  uint8_t nc=0; //??

  dotcnt=0;
  for (nr = 1; nr <MT9V03X_H - 1; nr++)
  {
    for (nc = 1; nc <MT9V03X_W - 1; nc++)
    {
      // 188*120????????????????????????????????????
      if ((mt9v03x_image_BandW[nr - 1][nc] +mt9v03x_image_BandW[nr + 1][nc] +mt9v03x_image_BandW[nr][nc + 1] + mt9v03x_image_BandW[nr][nc - 1] > 1))
      {
        dothang[dotcnt]=nr;     // ??????��?????????��??????????????
        dotlie[dotcnt++]=nc;    // ??????��?????????��??????????
      }
    }
  }
  if(dotcnt) // ?????��??
  {
    sumlie=0,sumhang=0;         // ?????��???
    for(tm=0;tm<dotcnt;tm++)
    {
      sumlie+=dotlie[tm];     // ???��?????????????
      sumhang+=dothang[tm];   // ???��?????????????????????
    }
    sumlie =sumlie/dotcnt;      // ????????????
    sumhang=sumhang/dotcnt;     // ??????????????????
  }

  ips200_draw_line(0,sumhang,188,sumhang,RGB565_RED);
  ips200_draw_line(sumlie,0,sumlie,120,RGB565_RED);

  int8_t comparative=MT9V03X_W/2-sumlie;
  //ips200_show_int(150,150,sumhang,8);
  ips200_show_int(150,150,comparative,8);
  return;
}



int16_t OFFSET0;    // ????????????????????????
int16_t OFFSET1;    // ?????
int16_t OFFSET2;    // ???????????
int area_map[100]={0};


/*****************************************************************************
*?????????void SearchNeighbor_4(unsigned char *bitmap, unsigned char *labelmap,uint16 labelIndex,uint16 pixelIndex, Queue *queue)
*?????????????????????????????????pixelIndex???????????????????????????????????????????????????????????????��??????????????
*       ?????????????????????????????????????????????????????????????????��?��????????
*?????????bitmap:????????  labelmap:??????
*???????????
*??????2022??4??3??
*??    ???
******************************************************************************/
uint16_t area;//??????????????????
int Connected_x[100];// ????????????x???????
int Connected_y[100];// ????????????y???????
short Connected_xmin[100];//??��??????
short Connected_ymin[100];//??��??????
short Connected_xmax[100];//????????
short Connected_ymax[100];//?????????
int32_t Connected_light[100];//?????????????????????????????????????????

//??????????
static int NeighborDirection[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};  //?????????????????????????
void SearchNeighbor_4  (unsigned char *bitmap, // ?????????0??1??
                        unsigned char *labelmap, // ??????????????????????????????
                        uint8 labelIndex,// ?????????????????????uint16???uint8
                        int pixelIndex,  // ??????????????????
                        Queue *queue,// ???��????????????????????
                        int x,int y,// ??????????????????x=?��?y=?��?
                        unsigned char *lightmap) // ????????????????????????????
{
  int searchIndex,i,length;
  int qx,qy;
  labelmap[pixelIndex]=labelIndex;//????????? pixelIndex ????????????? labelIndex
  length=MT9V03X_W*MT9V03X_H;//?????????????????????????????
  
  //area++;  // ???????
  //Connected_x[labelIndex] += x;
  //Connected_y[labelIndex] += y;
  //Connected_light[labelIndex] += lightmap[pixelIndex];
    
  for(i=0;i<4;i++)//??????? (x,y) ????????????index = y * width + x???????????????????searchIndex = pixelIndex + ??y * width + ??x????? NeighborDirection ????????
  {
   searchIndex=pixelIndex+NeighborDirection[i][0]*MT9V03X_W+NeighborDirection[i][1];//?????????????????

   if(searchIndex >= 0 && searchIndex <length && bitmap[searchIndex] == 1 && labelmap[searchIndex] == 0)//???????????��,??8??????��???????????
   {
     
     labelmap[searchIndex]=labelIndex;//??????????????????��????????
     area++;
     

     
     //qy=y;
     //qx=(int)((searchIndex-y)/(MT9V03X_W));
     qy = searchIndex / MT9V03X_W;  // ??????????
     qx = searchIndex % MT9V03X_W;  // ??????????
     
     /*if(qy>Connected_ymax[labelIndex])
          Connected_ymax[labelIndex]=qy;
     if(qy<Connected_ymin[labelIndex])
          Connected_ymin[labelIndex]=qy;
     if(qx>Connected_xmax[labelIndex])
          Connected_xmax[labelIndex]=qx;
     if(qx<Connected_xmin[labelIndex])
          Connected_xmin[labelIndex]=qx;
     */
     Connected_x[labelIndex]+=qx;
     Connected_y[labelIndex]+=qy;
     //if(Connected_x[labelIndex]>=32767)
       //Connected_x[labelIndex]=0;

     Connected_light[labelIndex]+=lightmap[searchIndex];

     //??????????????????
     if(area>=65535)
     {
       area=65535;
       break;//break
     }
     PushQueue(queue,searchIndex);//??????????��??????????????? ???????????????
   }
  }
}


/*****************************************************************************
*?????????void ConnectedComponentLabeling(unsigned char *bitmap, unsigned char * labelmap)
*?????????????????,???????��?
*       ?????????????????????��????????????????????????????????????????????????????
*?????????bitmap:????????  labelmap:??????
*???????????????????????
*??????2022??4??3??
*??    ?????
*????????????????????�B???i??2???
          area_map[i]????????????????
          Connected_x[i],Connected_y[i],?????????????????
          Connected_light[i]???????????????????
******************************************************************************/
//????????????????????
uint16_t areamax=0;//???��???????????????
uint8_t labelmax=0,//???��???????
        labeldel[100]={0}, //???????��??3???????????
        labelcount=0;  //???????????

//bitmap:????????  labelmap:??????
void ConnectedComponentLabeling(unsigned char *bitmap, unsigned char * labelmap)
{
  uint8_t i,j,labelIndex=1,del=0,x=1;
  int index,popIndex;
  Queue *queue = NULL;
  queue = (Queue*)malloc(sizeof(Queue));
  queue->first = NULL;
  queue->last = NULL;
  memset(labelmap,0,MT9V03X_W*MT9V03X_H);//????????,?????0?????
  memset(labeldel,0,100);//??????��??3???????????
  memset(area_map,0,100);
  //memset(Connected_x,0,100);
  //memset(Connected_y,0,100);
  memset(Connected_x, 0, sizeof(Connected_x));
  memset(Connected_y, 0, sizeof(Connected_y));
  memset(Connected_light,0,100);
  areamax=0;
  labelcount=0;
  labelmax=1;
  int flag_1;
  int last_labelIndex=0;

  for(i = 1; i <=MT9V03X_H-1 ; i++)
  {
    for(j = 1; j <=MT9V03X_W-1 ; j++)
    {
      index=i*(MT9V03X_W)+j;//??????? ???????????????�� ????????
      if(bitmap[index]==1 && labelmap[index]==0)//?????��?????
      {
        /*if(labelIndex>=8)
        {
        break;
        }*/
        labelIndex++;//???????
        SearchNeighbor_4(bitmap,labelmap,labelIndex,index,queue,i,j,(unsigned char *)mt9v03x_image);//????????????? ????????????????????
        popIndex=PopQueue(queue);//????????????????????
        while(popIndex>-1)// pop???? ???????????????????????????????????  ??pop???????????��??????????????????
        {
         
          SearchNeighbor_4(bitmap,labelmap,labelIndex,popIndex,queue,i,j,(unsigned char *)mt9v03x_image);//?????��??????????????????????
          popIndex = PopQueue(queue);//???????????????????????��????????��??,?????????????????????,??????????????
          
        }
      }
      flag_1=labelIndex - last_labelIndex;
      last_labelIndex = labelIndex;

      if(flag_1==1)
      {
          area_map[labelIndex]=area; //?????????????

          Connected_x[labelIndex]=(int)(Connected_x[labelIndex]/area);  //?????????x
          Connected_y[labelIndex]=(int)(Connected_y[labelIndex]/area);  //?????????y
          Connected_light[labelIndex]=(int)(Connected_light[labelIndex]/area);
          
          
          flag_1 = 0;
      }

      if(x!=labelIndex)//?????????????
      {
        x=labelIndex;//X???????��??????????
        if(area>areamax)//??????��???????
        {
          areamax=area;
          labelmax=labelIndex;
        }

        if(area<=1)//     ???????��?,???��???
        {
          labeldel[del]=labelIndex;
          del++;
          if(del>=9) del=9;
        }

        area=0;
      }
    }
  }
  free(queue);//???????????????��??

  //??????????????
  labelcount=labelIndex-1;

}

/********************************************
????????��?
        ???--???      ????      ???
????  0~1500   0~120   0~255       0~
???->��  ???->??  ?????->??
???->??  ???->??  ?????->??

????->??->??????
???->??->??????
*********************************************/
int quanzhi_weight[100];
void quanzhong_light_panduan(void)
{
    float zz_return;
    memset(quanzhi_weight,0,100);
    for(int i=2;i<=labelcount+2;i++)
    {
        zz_return = Square_detection(Connected_x[i],Connected_y[i]);
        if(zz_return<0.8)//????????
        {
          
            quanzhi_weight[i]=0;
            continue;
        }
        if(((Connected_x[i]<30&&(Connected_y[i]<30||Connected_y[i]>100)) || (Connected_x[i]>130&&(Connected_y[i]<30||Connected_y[i]>100)))&&area_map[i]>30)//??????
        {
            quanzhi_weight[i]=0;
            continue;
        }
        if((Connected_x[i]>54&&Connected_x[i]<74&&Connected_y[i]>30&&Connected_y[i]<100 )&& area_map[i]<10)//?????��
        {
            quanzhi_weight[i]=0;
            continue;
        }
        //????????????????????
        //120??1500?????????????????????????????????
        //120???????????1500?????????????????????????
        quanzhi_weight[i]= (int) (((1-(abs((int)Connected_x[i]-94)*1.0/94))+(1-(abs((int)Connected_y[i]-60)*1.0/60)))*10+(area_map[i]*1.0/10000)+Connected_light[i]+zz_return*10);
 
    }

}
int center_x ; 
int center_y ;
//??????????
int find_max(void)
{
    int wei=2;
    int maxx=quanzhi_weight[2];
    for(int i=3;i<=labelcount+2;i++)
    {
        if(quanzhi_weight[i]>maxx)
        {
            maxx = quanzhi_weight[i];
            wei=i;
        }
    }
     //????
          
          float screen_x = Connected_x[wei] * 128.0f / MT9V03X_W;
          float screen_y = Connected_y[wei] * 81.0f / MT9V03X_H;
          
          center_x =Connected_x[wei]; 
          center_y =Connected_y[wei];
          //xx = (int)screen_x;
          //yy = (int)screen_y;
          
           //?????????? (128??81)
          //float screen_x = center_x[wei] * (128.0f / MT9V03X_W);
          //float screen_y = center_y[wei] * (81.0f / MT9V03X_H);
          //float screen_x = center_x[wei] * (127.0f / (MT9V03X_W - 1));  // ???0-127
          //float screen_y = center_y[wei] * (81.0f / (MT9V03X_H - 1));   // ???0-81
          //xx = (int)screen_x;
          //yy = (int)screen_y;
           xx = (int)screen_x;  // ????????
           yy = (int)screen_y;
          
          //xx=(int)center_x[wei];
          //yy=(int)center_y[wei];
          
          if(xx>=127)xx=127;
          if(xx<=0)xx=0;
          if(yy>=81)yy=81;
          if(yy<=0)yy=0;
    return wei;
}

float KKKK;




//?????????????��??
/*void quanzhong_light(void)
{
    dotcnt=0;
    int zxc=find_max();
    int16_t comparative=MT9V03X_W/2-sumlie;
                 //???   ????
    

    if_look_light = 0;

    if(quanzhi_weight[zxc]!=0)
    {
        QQQQ = 1;//???????????

        if_look_light = 1;//????????

        keep_going = 70;//??????
        eye_look_light = 1;
        sumlie = Connected_y[zxc];
        sumhang = Connected_x[zxc];
        dotcnt++;
        //move_k1 = 0.015;

        target_turn = comparative ;
if(sumhang >=80 )
{

    target_turn = 0 ;

}



        if (count19 ==1 )
            {
                count20 = 20 ;
                count19 = 0 ;
            }
            else
            {
                count20 = 0 ;
            }

        if(target_turn <= 7 + count20 && target_turn >= -7 - count20 )
        {

            target_turn = 0;
            count19 = 1;

        }

        ips200_show_int(150,130,target_turn,8);
        if(target_turn  <= 25 && target_turn  >= -25 )//&& double_speed() <= 700)
            {


              target_turn = 0;
              count21 ++ ;
              if(count21 >= 80)
              {

              set_speed = 0;

              }


            }
            else
            {
                count21 = 0;
                set_speed = 0;
            }

    }

    else
    {
      //  move_k1 = 0.015;
        keep_going-- ;
        if(keep_going <= 10)//?????????
        {
           // qianhou_zhongzhi = 1.54;
            set_speed = 0;
            keep_going = 5;
            if_turn = 1 ;
        }
        else if(keep_going > 10)//?????????????
        {
            //qianhou_zhongzhi = 3.0;
            set_speed = 0 ;
            if_turn = 0 ;
        }

        ips200_show_int(150,130,target_turn,8);

        if (QQQQ == 1)  //??????��????????????????????????,1,??????????????????????1??0
        {               //2,???????????????????????0??1
        target_turn = 0 ;
        QQQQ = 0 ;
        }
        eye_look_light = 2;//???????????
    }
}
*/

/*---------------------------------------------------------------
????    ????Bin_Image_Filter
????    ???????????????????????????????????��?????????????????????????????????????????????????????
         ??????????????????????????��?????????????????????????????????????????????????????
????    ??????
???? ?? ?????
?????????
----------------------------------------------------------------*/
void Bin_Image_Filter (void)
{
  int  nr; //??
  int  nc; //??

  for (nr = 1; nr < MT9V03X_H - 1; nr++)
  {
    for (nc = 1; nc < MT9V03X_W- 1; nc = nc + 1)
    {
      if ((mt9v03x_image_BandW[nr][nc] == 0)
          && (mt9v03x_image_BandW[nr - 1][nc] + mt9v03x_image_BandW[nr + 1][nc] + mt9v03x_image_BandW[nr][nc + 1] + mt9v03x_image_BandW[nr][nc - 1] > 2))
      {
          mt9v03x_image_BandW[nr][nc] = 1;
      }
      else if ((mt9v03x_image_BandW[nr][nc] == 1)
               && (mt9v03x_image_BandW[nr - 1][nc] + mt9v03x_image_BandW[nr + 1][nc] + mt9v03x_image_BandW[nr][nc + 1] + mt9v03x_image_BandW[nr][nc - 1] < 2))
      {
          mt9v03x_image_BandW[nr][nc] = 0;
      }
    }
  }
}

/*************************************************************************
*  ?????????void eye_show(void)
*  ?????????????????????????��????????????????
*  ???????????
*  ???????????
*  ??????2022??3??31??
*  ??    ???  Bin_Image[LCDH][LCDW]; ??????????????
*************************************************************************/
void eye_show (void)//??????
{
  for(int i = 1 ;i<MT9V03X_H-1;i++)
  {
      mt9v03x_image_BandW[i][sumlie] = 1;
  }
  for(int i = 1 ;i<MT9V03X_W-1;i++)
  {
      mt9v03x_image_BandW[sumhang][i] = 1;
  }
    return;
}

/**************************************************************************
*                                                                          *
*  ?????????Seek_Road(void)                                                   *
*  ??????????????????????                                                      *
*  ???????????                                                            *
*  ?????????????��                                                      *
*  ??????2017-07-16                                                    *
*  ??    ??????��??0???????????????????????1?????                            *
*            ????????????��????????????��?????                                 *
*            ????????????????????????????                                  *
*            ????????????????????????????                                  *
**************************************************************************/
void Seek_Road (void)
{
    int16_t nr; //??
    int16_t nc; //??
    int16_t temp = 0; //??????
    //for(nr=1; nr<MAX_ROW-1; nr++)
    temp = 0;
    int16 MAX_COL=MT9V03X_W;   //???��????��????��?
    for (nr = 8; nr < 40; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET0 = temp;
    temp = 0;
    for (nr = 40; nr < 80; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET1 = temp;
    temp = 0;
    for (nr = 80; nr < 110; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET2 = temp;
    return;
}

/*****************************************************************************
*?????????void PushQueue(Queue *queue,int data)
*????????????
*???????????
*???????????
*??????2022??4??3??
*??    ??????,??????????��?????,???????????��
******************************************************************************/
void PushQueue(Queue *queue,int data)   //?????queue?��????????data???????
{
    QNode *p=NULL;//*p????��????? QNode???????????????????
    p=(QNode*)malloc(sizeof(QNode));//??????????��??(p?????????????)
    p->data=data;
    if(queue->first==NULL)//?????��?????????�� ????��???????? ???????????????��  ??????
    {
     queue->first=p;
     queue->last=p;
     p->next=NULL;
    }
    else  //?????????
    {
     p->next=NULL;//???????????��???--
     queue->last->next=p;//?????��?��?????????P
     queue->last=p;//?????��?��
    }
}
//
/*****************************************************************************
*?????????int PopQueue(Queue *queue)
*?????????????
*???????????
*???????????
*??????2022??4??3??
*??    ???????,????????????????,?????�??????
******************************************************************************/
int PopQueue(Queue *queue) //?????queue????????????????????????
{
    QNode *p=NULL;
    int data;
    if(queue->first==NULL)//?????pop??????-1
    {
     return -1;
    }
     p=queue->first;//???P??????
     data=p->data;//????????????
    if(queue->first->next == NULL)//????????????
    {
     queue->first=NULL;
     queue->last=NULL;
    }
    else
    {
     queue->first=p->next;
    }
    free(p);//?????????????????
    return data;
}


/*************************************************************************
*  ?????????float Square_detection (int x,int y)
*  ????????????????��??????��?
*  ???????????
*  ???????????
*  ??????2022??7??11??
*  ??    ???  mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W]; ??????????????
*************************************************************************/
uint8_t shang=0,xia=0,zuo=0,you=0;
float Square_detection(int16_t x,int16_t y)
{
   // uint8_t tm=0;
    int16_t nr=x; //??
    int16_t nc=y; //??
    shang=0,xia=0,zuo=0,you=0;  //????????
    dotcnt=0;

if(nr>3&&nr<116&&nc>3&&nc<184){         //??????????��?????????
    if((mt9v03x_image_BandW[nr][nc] == 1) || (mt9v03x_image_BandW[nr + 1][nc] + mt9v03x_image_BandW[nr + 3][nc]+ mt9v03x_image_BandW[nr - 1][nc] +mt9v03x_image_BandW[nr - 3][nc] +
      mt9v03x_image_BandW[nr][nc + 1] +mt9v03x_image_BandW[nr][nc + 3]+mt9v03x_image_BandW[nr][nc -3] +mt9v03x_image_BandW[nr][nc - 1] >= 4) )
    {
        while (mt9v03x_image_BandW[nr - shang][nc] ==1)
        {
            shang++;
        }
        while (mt9v03x_image_BandW[nr + xia][nc] == 1)
        {
            xia++;
        }
        while (mt9v03x_image_BandW[nr][nc - zuo] == 1)
        {
            zuo++;
        }
        while (mt9v03x_image_BandW[nr][nc + you] == 1)
        {
            you++;
        }
        //???????
        return (abs(zuo+you)/2.0) / (abs(shang+xia)/2.0);

    }
    else return (0);
}
else return (0);
}

