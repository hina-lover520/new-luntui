/*
 * track_detect.c
 * ------------------------------------------------------------------
 *  环岛 + 十字 赛道图像处理 —— 状态机实现
 * ------------------------------------------------------------------
 *  约定:
 *      白(1) = 赛道, 黑(0) = 背景
 *      y 向下增大(底部=近处), x 向右增大
 *  原则:
 *      补线 / 虚拟中线 只在变量层计算, 绝不修改二值图像像素
 * ------------------------------------------------------------------
 */

#include "track_detect.h"
#include "image.h"              /* GetOSTU 大津法求阈值 */
#include "find.h"               /* Bin_Image_Filter 孤立噪点滤波 */
#include <string.h>

/* 灰度原图(库内部定义) 与 二值图(本项目 find.c 中定义) */
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8 mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W];
extern uint8 display_image[MT9V03X_H][MT9V03X_W];

/* 便捷宏: 二值图像素 */
#define IMG(y, x)               (mt9v03x_image_BandW[(y)][(x)])

/* ==================================================================
 * 全局输出
 * ================================================================== */
track_state_t g_track_state    = STATE_NORMAL;
int16         g_track_mid_x    = (TRACK_W / 2);
int16         g_track_error    = 0;
uint8         g_track_stop     = 0;
uint8         g_track_threshold = TRACK_THRESHOLD;

/* ==================================================================
 * 内部状态变量
 * ================================================================== */
/* 边线 / 中线 (按行) */
static int16  g_left[TRACK_H];            /* 左边界 x                */
static int16  g_right[TRACK_H];           /* 右边界 x                */
static int16  g_mid[TRACK_H];             /* 中线 x (真实 或 虚拟)   */
static int16  g_vleft[TRACK_H];           /* 虚拟补线 左边界 x       */
static int16  g_vright[TRACK_H];          /* 虚拟补线 右边界 x       */
static uint8  g_valid[TRACK_H];           /* 该行是否有成对真实边线  */
static uint8  g_is_virtual[TRACK_H];      /* 该行中线是否为虚拟补线  */

/* 最长白列窗口 */
static int16  g_win_center;               /* 窗口中心(最长白列)      */
static int16  g_win_left;                 /* 窗口左界                */
static int16  g_win_right;                /* 窗口右界                */
static uint8  g_win_frozen;               /* 窗口是否冻结(十字B分支) */

/* 环岛拐点 */
static track_point_t g_entrance;          /* 入口拐点(画面靠下)      */
static track_point_t g_inner;             /* 内圆拐点(画面中上)      */
static uint8  g_entrance_ok;              /* 入口拐点有效标志         */
static uint8  g_inner_ok;                 /* 内圆拐点有效标志         */
static int16  g_ring_side;                /* +1=岛在左, -1=岛在右     */
static int16  g_ring_last_inner_x;        /* 历史内侧边线x(容错兜底)  */
static int    g_ring_lost_cnt;            /* 内圆拐点丢失容错计数     */

/* 十字 */
static int    g_cross_timer;              /* 补线计时(帧数)          */
static int    g_cross_recover;            /* 退出十字恢复稳定计数    */
static int16  g_mid_prev[TRACK_H];        /* 上一帧中线快照(冻结兜底)*/
static track_point_t g_cross_l_bot;       /* 十字左下拐点            */
static track_point_t g_cross_l_top;       /* 十字左上拐点            */
static track_point_t g_cross_r_bot;       /* 十字右下拐点            */
static track_point_t g_cross_r_top;       /* 十字右上拐点            */
static uint8  g_cross_corners_ok;         /* 十字四拐点有效标志      */

/* 十字检测调试诊断(供 UART/TFT 输出) */
int16 g_dbg_lcnt = 0;                    /* 左边缘拐点个数           */
int16 g_dbg_rcnt = 0;                    /* 右边缘拐点个数           */
int16 g_dbg_island = 0;                  /* 黑岛判定结果(0白道/1黑岛)*/
int16 g_dbg_lspan = 0;                   /* 左边缘上下拐点Y差        */
int16 g_dbg_rspan = 0;                   /* 右边缘上下拐点Y差        */
int16 g_dbg_win_l = 0;                   /* 扫描窗口左界             */
int16 g_dbg_win_r = 0;                   /* 扫描窗口右界             */
int16 g_dbg_ref_w = 0;                   /* 近处参考赛道宽度(宽臂十字) */
int16 g_dbg_wide_c = 0;                  /* 最大连续宽行数(宽臂十字) */
int16 g_dbg_lc_x[4] = {-1,-1,-1,-1};     /* 左边缘前4个拐点 x        */
int16 g_dbg_lc_y[4] = {-1,-1,-1,-1};     /* 左边缘前4个拐点 y        */
int16 g_dbg_rc_x[4] = {-1,-1,-1,-1};     /* 右边缘前4个拐点 x        */
int16 g_dbg_rc_y[4] = {-1,-1,-1,-1};     /* 右边缘前4个拐点 y        */

/* ==================================================================
 * 工具: 坐标映射 (图像 188x120 -> 全屏灰度图显示区 128x160)
 * ================================================================== */
static inline uint16 mapx(int16 x) { return (uint16)(((int32)x * (int)tft180_width_max) / TRACK_W); }
static inline uint16 mapy(int16 y) { return (uint16)(((int32)y * (int)tft180_height_max) / TRACK_H); }

/* 屏幕坐标钳位 (TFT180 默认竖屏 128x160, 画叉偏移要防越界触发 assert) */
static inline int clipx(int v) { if (v < 0) return 0; if (v >= (int)tft180_width_max) return (int)tft180_width_max - 1; return v; }
static inline int clipy(int v) { if (v < 0) return 0; if (v >= (int)tft180_height_max) return (int)tft180_height_max - 1; return v; }

/* ==================================================================
 * 初始化
 * ================================================================== */
void track_init(void)
{
    g_track_state   = STATE_NORMAL;
    g_track_mid_x   = TRACK_W / 2;
    g_track_error   = 0;
    g_track_stop    = 0;

    g_win_center    = TRACK_W / 2;
    g_win_left      = g_win_center - TRACK_WINDOW_HALF;
    g_win_right     = g_win_center + TRACK_WINDOW_HALF;
    if (g_win_left  < 0)         g_win_left  = 0;
    if (g_win_right >= TRACK_W)  g_win_right = TRACK_W - 1;
    g_win_frozen    = 0;

    g_entrance_ok   = 0;
    g_inner_ok      = 0;
    g_ring_side     = 0;
    g_ring_last_inner_x = TRACK_W / 2;
    g_ring_lost_cnt = 0;

    g_cross_timer   = 0;
    g_cross_recover = 0;
    g_cross_corners_ok = 0;

    memset(g_left, 0, sizeof(g_left));
    memset(g_right, 0, sizeof(g_right));
    memset(g_mid, 0, sizeof(g_mid));
    memset(g_vleft, 0, sizeof(g_vleft));
    memset(g_vright, 0, sizeof(g_vright));
    memset(g_valid, 0, sizeof(g_valid));
    memset(g_is_virtual, 0, sizeof(g_is_virtual));
    memset(g_mid_prev, 0, sizeof(g_mid_prev));
}

/* ==================================================================
 * 二值化: 灰度 -> 0/1 (白=赛道)
 * 注: 若你已有 sobel+腐蚀膨胀 管线, 可跳过本函数, 直接调 track_border_clear()
 * ================================================================== */
void track_binarize(uint8 threshold)
{
    int y, x;
    for (y = 0; y < TRACK_H; y++)
    {
        for (x = 0; x < TRACK_W; x++)
        {
            IMG(y, x) = (mt9v03x_image[y][x] >= threshold) ? 1 : 0;
        }
    }
}

/* ==================================================================
 * 大津法统一阈值二值化 (纯大津法: 全图所有行用同一个阈值)
 *   修复: 之前"行自适应"在远处(顶部)把阈值降得过低, 远处蓝背景/亮地面
 *   被整片误判为白 -> 边线爆炸到全宽 -> 十字拐点消失/环岛误触发。
 *   现在阈值 = 大津法结果(EMA平滑+钳位), 远处近处一致。
 *   同时刷新 display_image 供 TFT 显示。
 * ================================================================== */
void track_binarize_adaptive(uint8 base)
{
    int y, x;
    int th = base;
    if (th < TRACK_ROW_MIN) th = TRACK_ROW_MIN;
    if (th > TRACK_ROW_MAX) th = TRACK_ROW_MAX;

    for (y = 0; y < TRACK_H; y++)
    {
        for (x = 0; x < TRACK_W; x++)
        {
            IMG(y, x) = (mt9v03x_image[y][x] >= th) ? 1 : 0;
        }
    }

    for (y = 0; y < TRACK_H; y++)
    {
        for (x = 0; x < TRACK_W; x++)
        {
            display_image[y][x] = IMG(y, x) * 255;
        }
    }
}

/* ==================================================================
 * 求二值化阈值
 *   TRACK_USE_OTSU=1 -> 大津法(GetOSTU)自动阈值, 并做上下限钳位
 *   TRACK_USE_OTSU=0 -> 固定阈值 TRACK_THRESHOLD
 * 返回: 实际使用的阈值; 同时写入 g_track_threshold(供调试显示)
 * 注:   蓝白赛道灰度为三峰(黑背景/蓝背景/白道), 大津法偏双峰模型,
 *       故用上下限钳位兜底, 避免极端光照下阈值跑飞。
 * ================================================================== */
uint8 track_get_threshold(void)
{
    static int16 smoothed = 0;                      /* 平滑后阈值(跨帧保持) */
#if TRACK_USE_OTSU
    int16 t = GetOSTU(mt9v03x_image);
    t -= TRACK_OTSU_MARGIN;              /* 纯大津法=0: 不再向下偏移(偏移把蓝背景误判白→远处整片白) */
    if (t < TRACK_OTSU_MIN) t = TRACK_OTSU_MIN;
    if (t > TRACK_OTSU_MAX) t = TRACK_OTSU_MAX;
    /* 阈值时间平滑(EMA): 新阈值只占 1/TRACK_OTSU_SMOOTH, 避免逐帧抖动把近处白道闪断 */
    if (smoothed == 0) smoothed = t;
    else smoothed = (smoothed * (TRACK_OTSU_SMOOTH - 1) + t) / TRACK_OTSU_SMOOTH;
    g_track_threshold = (uint8)smoothed;
#else
    g_track_threshold = TRACK_THRESHOLD;
#endif
    return g_track_threshold;
}

/* ==================================================================
 * 腐蚀+孤立点滤波去噪 (消除边缘毛刺, 且不伤十字拐角)
 *   1) erosion_operation 腐蚀 TRACK_ERODE_TIMES 次: 去掉边界细毛刺;
 *   2) Bin_Image_Filter (4邻域): 白点被黑包围(<=1个白邻点)置黑去孤立噪点,
 *      黑点被白包围(>=3个白邻点)置白填洞。拐角处白点仍有2个白邻点, 不会被削掉。
 *   注意: 腐蚀会收缩赛道边界, 次数过多会把窄道/远处细道腐蚀掉。
 * ================================================================== */
void track_erode(void)
{
    int k, y, x;
    for (k = 0; k < TRACK_ERODE_TIMES; k++)
    {
        erosion_operation(mt9v03x_image_BandW, TRACK_ERODE_VALUE, TRACK_ERODE_THRESHOLD);
    }
    Bin_Image_Filter();                             /* 孤立噪点去除 + 填洞(4邻域, 不削拐角) */
    /* 同步刷新显示缓冲, 让 TFT 显示去噪后的二值图(便于调参) */
    for (y = 0; y < TRACK_H; y++)
    {
        for (x = 0; x < TRACK_W; x++)
        {
            display_image[y][x] = mt9v03x_image_BandW[y][x] * 255;
        }
    }
}

/* ==================================================================
 * 图像最外圈置黑
 *   手动把最外圈一圈像素(上/下/左/右各1px)置黑:
 *   最长白列计数和左右扫边线遇到黑点即停, 最外圈全黑可以保证
 *   扫线永远不会越出图像范围。
 * ================================================================== */
void track_border_clear(void)
{
    int y, x;

    for (x = 0; x < TRACK_W; x++)
    {
        IMG(0, x)          = 0;              /* 顶行   */
        IMG(TRACK_H - 1, x) = 0;             /* 底行   */
    }
    for (y = 0; y < TRACK_H; y++)
    {
        IMG(y, 0)          = 0;              /* 左列   */
        IMG(y, TRACK_W - 1) = 0;             /* 右列   */
    }
}

/* ==================================================================
 * 更新扫描窗口
 *   只统计图像下半近处区域, 计算:
 *     白点质心(平均x)   -> 道路中心 g_win_center
 *     白点最左/最右列   -> 道路范围, 外扩余量后得到扫描窗口
 *   窗口正好框住整条赛道, 后续扫线找到的是真实道路边线。
 *   窗口冻结(十字B分支)时不更新。
 *   (注: 原"最长白列"对整片白道会因并列取最左列导致窗口偏左, 已弃用)
 * ================================================================== */
static void track_update_window(void)
{
    int x, y;
    int32 sum = 0;
    int32 cnt = 0;
    int left  = TRACK_W - 1;
    int right = 0;

    if (g_win_frozen) return;                       /* 冻结时不更新窗口 */

    for (y = TRACK_NEAR_Y_START; y <= TRACK_NEAR_Y_END; y++)
    {
        for (x = 0; x < TRACK_W; x++)
        {
            if (IMG(y, x))
            {
                sum += x;
                cnt++;
                if (x < left)  left  = x;
                if (x > right) right = x;
            }
        }
    }

    if (cnt > 0)
    {
        g_win_center = (int16)(sum / cnt);          /* 白点质心 = 道路中心 */
        g_win_left   = left  - TRACK_WINDOW_MARGIN; /* 道路左界外扩         */
        g_win_right  = right + TRACK_WINDOW_MARGIN; /* 道路右界外扩         */
    }
    else
    {
        g_win_center = TRACK_W / 2;                 /* 底部无白点 -> 回退图像中心 */
        g_win_left   = g_win_center - TRACK_WINDOW_HALF;
        g_win_right  = g_win_center + TRACK_WINDOW_HALF;
    }

    if (g_win_left   < 0)        g_win_left   = 0;
    if (g_win_right  >= TRACK_W) g_win_right  = TRACK_W - 1;
    if (g_win_center < 0)        g_win_center = 0;
    if (g_win_center >= TRACK_W) g_win_center = TRACK_W - 1;
}

/* ==================================================================
 * 最长白列扫线 (从底边往上)
 *   1) 每列自最低白点起向上数连续白点长度, 最长的一列 = 赛道中心列
 *      (白列); 平局取离上一帧中心列更近的一列, 防止中心来回跳。
 *   2) 逐行(自下而上)找该行离中心列最近的白段, 白段两端 = 左右边线;
 *      偏差超过 TRACK_MAX_OFF 或宽度过小的行判无效。
 *   图像最外圈已由 track_border_clear 置黑, 白点计数/扫边遇黑即停,
 *   不会扫出图像范围。
 * ================================================================== */
static void track_scan_edges(void)
{
    int y, x;
    int best_cnt = 0;
    int center = TRACK_W / 2;
    static int16 prev_center = TRACK_W / 2;

    /* 1. 最长白列: 每列自底向上数连续白点(最外圈黑, 遇黑即停) */
    for (x = 1; x < TRACK_W - 1; x++)
    {
        int cnt = 0;
        for (y = TRACK_H - 2; y >= 1; y--)
        {
            if (IMG(y, x)) cnt++;
            else if (cnt > 0) break;              /* 从最低白点起数, 上方断开即停 */
        }
        if (cnt > best_cnt)
        {
            best_cnt = cnt;
            center   = x;
        }
        else if (cnt == best_cnt)
        {
            int d1 = (x > prev_center) ? (x - prev_center) : (prev_center - x);
            int d2 = (center > prev_center) ? (center - prev_center) : (prev_center - center);
            if (d1 < d2) center = x;              /* 平局取离上一帧中心列更近的 */
        }
    }
    if (best_cnt > 0) prev_center = (int16)center;

    /* 调试显示用: 白列长度 / 中心列 */
    g_dbg_ref_w  = (int16)best_cnt;
    g_dbg_wide_c = (int16)center;

    /* 2. 逐行: 离中心列最近的白段 = 赛道, 两端 = 左右边线 */
    memset(g_valid, 0, sizeof(g_valid));
    for (y = TRACK_SCAN_Y_START; y >= TRACK_SCAN_Y_END; y--)
    {
        int best_l = -1, best_r = -1, best_d = TRACK_MAX_OFF + 1;

        x = 1;
        while (x < TRACK_W - 1)
        {
            if (IMG(y, x))
            {
                int rl = x;
                while (x < TRACK_W - 1 && IMG(y, x)) x++;
                int rr = x - 1;
                int rc = (rl + rr) / 2;
                int d  = (rc > center) ? (rc - center) : (center - rc);
                if (d < best_d) { best_d = d; best_l = rl; best_r = rr; }
            }
            else
            {
                x++;
            }
        }

        if (best_l >= 0 && best_d <= TRACK_MAX_OFF && (best_r - best_l + 1) >= TRACK_MIN_TRACK_W)
        {
            g_left[y]  = (int16)best_l;
            g_right[y] = (int16)best_r;
            g_valid[y] = 1;
        }
        else
        {
            g_left[y]  = 0;
            g_right[y] = 0;
        }
    }
}

#if TRACK_USE_CROSS
/* ==================================================================
 * 十字宽臂触发 (不用八邻域拐点)
 *   近处参考宽度 = 底部有效行平均白段宽; 自下而上统计连续"宽行"
 *   (白段宽 >= 参考宽*TRACK_CROSS_WIDE_RATIO/10 且 >= TRACK_CROSS_WIDE_MIN);
 *   连续 >= TRACK_CROSS_WIDE_ROWS 行 -> 触发十字。
 *   透视关系: 正常赛道越往上越窄, 中部出现远超近处的宽白段只可能是
 *   十字横臂这类横向白色结构。
 * 返回: 1=触发十字
 * ================================================================== */
static uint8 track_check_cross_wide(void)
{
    int y, n = 0;
    int32 ref_w = 0;
    int consec = 0;

    for (y = TRACK_NEAR_Y_START; y <= TRACK_NEAR_Y_END; y++)
    {
        if (g_valid[y]) { ref_w += g_right[y] - g_left[y] + 1; n++; }
    }
    g_dbg_ref_w  = (n > 0) ? (int16)(ref_w / n) : 0;
    g_dbg_wide_c = 0;

    if (n < TRACK_CROSS_NEAR_MIN || g_dbg_ref_w <= 0) return 0;

    for (y = TRACK_NEAR_Y_START - 1; y >= TRACK_SCAN_Y_END; y--)
    {
        if (g_valid[y])
        {
            int w = g_right[y] - g_left[y] + 1;
            if ((int32)w * 10 >= ref_w * TRACK_CROSS_WIDE_RATIO && w >= TRACK_CROSS_WIDE_MIN)
            {
                consec++;
                if (consec > g_dbg_wide_c) g_dbg_wide_c = (int16)consec;
                if (consec >= TRACK_CROSS_WIDE_ROWS) return 1;
            }
            else
            {
                consec = 0;
            }
        }
        else
        {
            consec = 0;
        }
    }
    return 0;
}
#endif /* TRACK_USE_CROSS */

/* ==================================================================
 * 直道: 左右边线中点 -> 中线
 * ================================================================== */
static void track_midline_normal(void)
{
    int y;
    for (y = 0; y < TRACK_H; y++)
    {
        if (g_valid[y])
        {
            g_mid[y]       = (g_left[y] + g_right[y]) / 2;
            g_is_virtual[y] = 0;
        }
        else
        {
            g_mid[y]       = 0;
            g_is_virtual[y] = 0;
        }
    }
}

/* ==================================================================
 * 八邻域拐点检测 (对一条边线 x=f(y))
 *   相邻两行构成一个 8 邻域方向向量, 相邻两段方向向量突变(折角)即为拐点。
 *   纯平滑圆弧: 方向每行变化很小 -> 不产生拐点 (符合“必须有折角”前提)。
 * 返回: 检测到的拐点个数
 * ================================================================== */
static int track_detect_corners_edge(const int16 *edge, const uint8 *valid,
                                     track_point_t *corners, int maxc)
{
    int cnt = 0, y, d1, d2, turn;

    for (y = TRACK_SCAN_Y_END + 1; y <= TRACK_SCAN_Y_START - 1; y++)
    {
        if (!valid[y - 1] || !valid[y] || !valid[y + 1]) continue;

        d1 = edge[y] - edge[y - 1];                 /* 上一段方向(dx) */
        d2 = edge[y + 1] - edge[y];                 /* 下一段方向(dx) */
        turn = (d2 > d1) ? (d2 - d1) : (d1 - d2);   /* 方向突变绝对值 */

        if (turn >= TRACK_CORNER_TURN)
        {
            if (cnt < maxc)
            {
                corners[cnt].x = edge[y];
                corners[cnt].y = y;
                cnt++;
            }
        }
    }
    return cnt;
}

/* ==================================================================
 * 拐点降噪
 *   空间距离很近的拐点判定为噪点, 直接过滤。
 * ================================================================== */
static int track_corner_denoise(track_point_t *corners, int cnt)
{
    int i, j, keep = 0, dup;
    track_point_t out[64];

    for (i = 0; i < cnt; i++)
    {
        dup = 0;
        for (j = 0; j < keep; j++)
        {
            int dx = corners[i].x - out[j].x; if (dx < 0) dx = -dx;
            int dy = corners[i].y - out[j].y; if (dy < 0) dy = -dy;
            if (dx * dx + dy * dy < TRACK_CORNER_MIN_DIST * TRACK_CORNER_MIN_DIST)
            {
                dup = 1; break;
            }
        }
        if (!dup && keep < 64) out[keep++] = corners[i];
    }
    for (i = 0; i < keep; i++) corners[i] = out[i];
    return keep;
}

#if TRACK_USE_8NEIGH
/* ==================================================================
 * 环岛触发检测
 *   在拐点列表中找:
 *     入口拐点 = y 最大(画面靠下) 的拐点
 *     内圆拐点 = 在入口上方且 Y 轴差值 > 阈值 的拐点里, 最靠近入口的那个
 *   满足条件则保存两点, 并确定环岛在左还是右。
 * 返回: 1=触发, 0=未触发
 * ================================================================== */
static uint8 track_check_ring(track_point_t *corners, int cnt)
{
    int i, best_e = -1, best_in = -1;

    /* 入口拐点: y 最大(最靠下) */
    for (i = 0; i < cnt; i++)
    {
        if (best_e < 0 || corners[i].y > corners[best_e].y) best_e = i;
    }
    if (best_e < 0) return 0;

    /* 内圆拐点: 在入口上方、Y轴差值>阈值的拐点中, 选最靠近入口的 */
    for (i = 0; i < cnt; i++)
    {
        if (corners[i].y < corners[best_e].y - TRACK_RING_Y_DIFF_MIN)
        {
            if (best_in < 0 || corners[i].y > corners[best_in].y) best_in = i;
        }
    }
    if (best_in < 0) return 0;

    g_entrance    = corners[best_e];
    g_inner       = corners[best_in];
    g_entrance_ok = 1;
    g_inner_ok    = 1;

    /* 环岛侧: 内圆拐点相对入口拐点的横向偏移 -> 岛在左(+1) 或 岛在右(-1) */
    g_ring_side = (g_inner.x < g_entrance.x) ? +1 : -1;

    return 1;
}
#endif /* TRACK_USE_8NEIGH */

#if TRACK_USE_8NEIGH
/* ==================================================================
 * 十字四拐点中心黑岛判别
 *   十字中心 = 实心白道; 环岛中心 = 黑色圆岛。
 *   在四个拐点围成的矩形中心采样二值图, 若黑点占比高 -> 是环岛。
 * 返回: 1=中心是黑岛(环岛, 不判十字), 0=中心是白道(十字)
 * ================================================================== */
static uint8 track_cross_center_is_dark(const track_point_t *lb, const track_point_t *lt,
                                        const track_point_t *rb, const track_point_t *rt)
{
    int y_top = (lt->y < rt->y) ? lt->y : rt->y;
    int y_bot = (lb->y > rb->y) ? lb->y : rb->y;
    int x_l   = (lb->x < lt->x) ? lb->x : lt->x;
    int x_r   = (rb->x > rt->x) ? rb->x : rt->x;
    int cy    = (y_top + y_bot) / 2;
    int cx    = (x_l + x_r) / 2;
    int half_w = (x_r - x_l) / 8 + 1;              /* 中心窗口半宽 ≈ 框宽1/8 */
    int half_h = (y_bot - y_top) / 6 + 1;          /* 中心窗口半高 ≈ 框高1/6 */
    int dy, dx, dark = 0, total = 0;

    for (dy = -half_h; dy <= half_h; dy++)
    {
        int yy = cy + dy;
        if (yy < 0 || yy >= TRACK_H) continue;
        for (dx = -half_w; dx <= half_w; dx++)
        {
            int xx = cx + dx;
            if (xx < 0 || xx >= TRACK_W) continue;
            total++;
            if (!IMG(yy, xx)) dark++;
        }
    }
    if (total == 0) return 0;
    return ((dark * 100 / total) >= TRACK_CROSS_ISLAND_DARK_PCT) ? 1 : 0;
}

/* ==================================================================
 * 十字四拐点检测 (八邻域)
 *   十字: 直道在横道处外扩再收回, 左/右边缘各出现两个拐点。
 *     左边缘: 下拐点(y最大, 外扩处) + 上拐点(y最小, 收回处)
 *     右边缘: 对称。
 *   判定: 左右边缘都找到一对 且 上/下拐点 Y 差 >= TRACK_CROSS_MIN_DY
 *     且 四拐点中心不是黑岛(排除环岛: 环岛中心是黑色圆岛)
 * 返回: 1=检测到十字; 并保存四个拐点
 * ================================================================== */
static uint8 track_check_cross(const int16 *ledge, const int16 *redge, const uint8 *valid)
{
    track_point_t lc[16], rc[16];
    int ln = track_detect_corners_edge(ledge, valid, lc, 16);
    int rn = track_detect_corners_edge(redge, valid, rc, 16);
    int i, lb = -1, lt = -1, rb = -1, rt = -1;

    /* 调试: 记录原始拐点个数/窗口/拐点坐标(供 UART/TFT 诊断) */
    g_dbg_lcnt   = ln;
    g_dbg_rcnt   = rn;
    g_dbg_island = 0;
    g_dbg_lspan  = 0;
    g_dbg_rspan  = 0;
    g_dbg_win_l  = g_win_left;
    g_dbg_win_r  = g_win_right;
    for (i = 0; i < 4; i++)
    {
        g_dbg_lc_x[i] = g_dbg_lc_y[i] = -1;
        g_dbg_rc_x[i] = g_dbg_rc_y[i] = -1;
    }
    for (i = 0; i < ln && i < 4; i++) { g_dbg_lc_x[i] = lc[i].x; g_dbg_lc_y[i] = lc[i].y; }
    for (i = 0; i < rn && i < 4; i++) { g_dbg_rc_x[i] = rc[i].x; g_dbg_rc_y[i] = rc[i].y; }

    g_cross_corners_ok = 0;                        /* 每帧重新判定 */

    for (i = 0; i < ln; i++)
    {
        if (lb < 0 || lc[i].y > lc[lb].y) lb = i;   /* 左下拐点 = y最大 */
        if (lt < 0 || lc[i].y < lc[lt].y) lt = i;   /* 左上拐点 = y最小 */
    }
    for (i = 0; i < rn; i++)
    {
        if (rb < 0 || rc[i].y > rc[rb].y) rb = i;   /* 右下拐点 = y最大 */
        if (rt < 0 || rc[i].y < rc[rt].y) rt = i;   /* 右上拐点 = y最小 */
    }
    if (lb < 0 || lt < 0 || rb < 0 || rt < 0) return 0;

    g_dbg_lspan = lc[lb].y - lc[lt].y;             /* 调试: 左上下Y差 */
    g_dbg_rspan = rc[rb].y - rc[rt].y;             /* 调试: 右上下Y差 */

    /* 左右边都要有足够高的十字(上/下拐点 Y 差) */
    if (g_dbg_lspan < TRACK_CROSS_MIN_DY) return 0;
    if (g_dbg_rspan < TRACK_CROSS_MIN_DY) return 0;

    /* 区分十字 vs 环岛: 十字中心=白道, 环岛中心=黑岛 */
#if TRACK_CROSS_ISLAND_CHECK
    g_dbg_island = track_cross_center_is_dark(&lc[lb], &lc[lt], &rc[rb], &rc[rt]);
    if (g_dbg_island) return 0;
#endif

    g_cross_l_bot = lc[lb];
    g_cross_l_top = lc[lt];
    g_cross_r_bot = rc[rb];
    g_cross_r_top = rc[rt];
    g_cross_corners_ok = 1;
    return 1;
}
#endif /* TRACK_USE_8NEIGH */

/* ==================================================================
 * 判断边线是否平直 (最大方向突变 <= 阈值)
 * ================================================================== */
static uint8 track_edge_straight(const int16 *edge, const uint8 *valid)
{
    int y, max_turn = 0;
    for (y = TRACK_SCAN_Y_END + 1; y <= TRACK_SCAN_Y_START - 1; y++)
    {
        if (!valid[y - 1] || !valid[y] || !valid[y + 1]) continue;
        int d1 = edge[y] - edge[y - 1];
        int d2 = edge[y + 1] - edge[y];
        int t  = (d2 > d1) ? (d2 - d1) : (d1 - d2);
        if (t > max_turn) max_turn = t;
    }
    return (max_turn <= TRACK_RING_EXIT_STRAIGHT) ? 1 : 0;
}

/* ==================================================================
 * 环岛控制
 *   - 放弃直道"左右最外侧边线中点"算法
 *   - 拐点连线补线: 内侧路径 = 过(入口拐点, 内圆拐点)的直线,
 *     第一拐点连向岛(中间黑色), 第二拐点沿同一斜率延伸到远端汇合边;
 *     逐行中线 = (外边界 + 内侧拐点连线)/2
 *   - 入口拐点以下 = 单赛道, 用正常左右边线中心
 *   - 容错: 短时丢失, 沿用历史控制点兜底
 *   - 退出: 内侧边线拐点消失 且 边线走势恢复平直 -> 切回直道
 * 返回: 目标中线 x (近处控制点)
 * ================================================================== */
static int16 track_ring_control(void)
{
    int y;
    const int16 *outer = (g_ring_side > 0) ? g_left : g_right;   /* 选定分支外边界 */
    int16 ctrl = TRACK_W / 2;
    uint8 found = 0;
    int16 last = g_ring_last_inner_x;

    /* 内侧路径 = 过 入口拐点(g_entrance) 与 内圆拐点(g_inner) 的直线:
     *   slope = dx/dy (定点放大16)。第一拐点连向岛(中间黑色),
     *   第二拐点沿同一斜率继续延伸到远端汇合边。 */
    int32 dx      = (int32)g_inner.x - g_entrance.x;
    int32 dy      = (int32)g_entrance.y - g_inner.y;    /* 入口在下方, dy>0 */
    int32 slope16 = (dy > 0) ? ((dx * 16) / dy) : 0;

    /* 内侧边线拐点检测(仅用于退出判定) */
    track_point_t corners[64];
    int cnt = track_detect_corners_edge(outer, g_valid, corners, 64);
    cnt = track_corner_denoise(corners, cnt);

    /* 逐行填虚拟中线 */
    for (y = TRACK_SCAN_Y_END; y <= TRACK_SCAN_Y_START; y++)
    {
        if (g_valid[y])
        {
            if (y > g_entrance.y)
            {
                /* 入口拐点以下 = 单赛道, 用正常左右边线中心 */
                g_mid[y] = (int16)((g_left[y] + g_right[y]) / 2);
            }
            else
            {
                /* 拐点连线: 中线 = (外边界 + 内侧拐点连线)/2 */
                int32 inner_x = g_inner.x + (slope16 * (g_inner.y - y)) / 16;
                if (inner_x < 0)        inner_x = 0;
                if (inner_x >= TRACK_W) inner_x = TRACK_W - 1;
                g_mid[y] = (int16)((outer[y] + inner_x) / 2);
            }
            last = g_mid[y];
        }
        else
        {
            g_mid[y] = last;                     /* 断行沿用上一有效值 */
        }
        g_is_virtual[y] = 1;                     /* 环岛路径 = 虚拟补线(黄色) */
    }

    /* 近处控制点 = 最靠下有效行的分支中心 */
    for (y = TRACK_SCAN_Y_START; y >= TRACK_SCAN_Y_END; y--)
    {
        if (g_valid[y]) { ctrl = g_mid[y]; found = 1; break; }
    }

    if (found)
    {
        g_ring_last_inner_x = ctrl;
        g_ring_lost_cnt     = 0;
    }
    else
    {
        ctrl = g_ring_last_inner_x;              /* 容错: 沿用历史控制点 */
        g_ring_lost_cnt++;
        if (g_ring_lost_cnt > TRACK_RING_LOST_TOL)
        {
            g_track_state = STATE_NORMAL;        /* 长时间丢失 -> 回直道兜底 */
            return (TRACK_W / 2);
        }
    }

    /* 退出环岛: 内侧边线拐点消失 且 边线走势恢复平直 */
    if (cnt == 0 && track_edge_straight(outer, g_valid))
    {
        g_track_state = STATE_NORMAL;
        g_entrance_ok = 0;
        g_inner_ok    = 0;
    }

    return ctrl;
}

/* ==================================================================
 * 十字坐标补线 (虚拟中线)
 *   十字区域图像上边线物理断开, 用下半近处真实边线坐标向上补出虚拟中线。
 *   只在变量层计算(g_mid[]/g_vleft[]/g_vright[]), 不改二值图像像素。
 *   补线策略:
 *     TRACK_CROSS_USE_SLOPE=1 -> 近/远带平均拟合赛道斜率, 沿斜率向远处外推(默认)
 *     TRACK_CROSS_USE_SLOPE=0 -> 垂直直行补线(十字直行通过)
 * ================================================================== */
static void track_fill_cross_line(void)
{
    int y, y_bot = -1;

    /* 先: 有效行中线 = 真实左右边线中点 */
    for (y = TRACK_SCAN_Y_END; y <= TRACK_SCAN_Y_START; y++)
    {
        if (g_valid[y])
        {
            g_mid[y]        = (g_left[y] + g_right[y]) / 2;
            g_is_virtual[y] = 0;
        }
    }

    /* 找最靠下的有效行(近处基准) */
    for (y = TRACK_SCAN_Y_START; y >= TRACK_SCAN_Y_END; y--)
    {
        if (g_valid[y]) { y_bot = y; break; }
    }

    if (y_bot < 0)
    {
        /* 下半近处边线也消失(十字压到画面底部): 用上一帧中线快照兜底 */
        for (y = TRACK_SCAN_Y_END; y <= TRACK_SCAN_Y_START; y++)
        {
            if (!g_valid[y])
            {
                g_mid[y]        = g_mid_prev[y];
                g_is_virtual[y] = 1;
            }
        }
        return;
    }

#if TRACK_CROSS_USE_SLOPE
    {
        /* 数学外推: 用近处“近/远两条带”的平均边线位置拟合赛道斜率,
         * 再沿斜率向远处直线延长生成虚拟中线。
         *   - 带内多行平均 -> 抑制单行边线噪声;
         *   - 定点放大 -> 避免整数除法截断(旧实现用相邻行 dy≈1, 噪声被放大上百倍);
         *   - 参考不足时落入下方垂直补线兜底。 */
        int32 s_ln = 0, s_rn = 0, s_lf = 0, s_rf = 0;   /* 近/远带 左右边线累计 */
        int32 s_yn = 0, s_yf = 0;                       /* 近/远带 行号累计     */
        int   c_n  = 0, c_f  = 0;                       /* 近/远带 有效行计数   */

        /* 近带: 自 y_bot 向上取 TRACK_CROSS_SLOPE_BAND 个有效行 */
        for (y = y_bot; y >= TRACK_SCAN_Y_END && c_n < TRACK_CROSS_SLOPE_BAND; y--)
        {
            if (g_valid[y]) { s_ln += g_left[y]; s_rn += g_right[y]; s_yn += y; c_n++; }
        }
        /* 远带: 自 y_bot-GAP 再向上取 TRACK_CROSS_SLOPE_BAND 个有效行 */
        for (y = y_bot - TRACK_CROSS_SLOPE_GAP; y >= TRACK_SCAN_Y_END && c_f < TRACK_CROSS_SLOPE_BAND; y--)
        {
            if (g_valid[y]) { s_lf += g_left[y]; s_rf += g_right[y]; s_yf += y; c_f++; }
        }

        if (c_n >= TRACK_CROSS_SLOPE_MIN && c_f >= TRACK_CROSS_SLOPE_MIN)
        {
            int32 yn = s_yn / c_n, yf = s_yf / c_f;
            int32 ln = s_ln / c_n, lf = s_lf / c_f;
            int32 rn = s_rn / c_n, rf = s_rf / c_f;
            int32 dy = yn - yf;                         /* > 0 */

            if (dy >= TRACK_CROSS_SLOPE_DY_MIN)
            {
                int32 sl = ((ln - lf) * TRACK_CROSS_SLOPE_SCALE) / dy;   /* 斜率 dx/dy */
                int32 sr = ((rn - rf) * TRACK_CROSS_SLOPE_SCALE) / dy;

                for (y = TRACK_SCAN_Y_END; y <= TRACK_SCAN_Y_START; y++)
                {
                    if (g_valid[y]) continue;
                    int32 vl = ln - (sl * (yn - y)) / TRACK_CROSS_SLOPE_SCALE;
                    int32 vr = rn - (sr * (yn - y)) / TRACK_CROSS_SLOPE_SCALE;
                    if (vl < 0) vl = 0;                 if (vl >= TRACK_W) vl = TRACK_W - 1;
                    if (vr < 0) vr = 0;                 if (vr >= TRACK_W) vr = TRACK_W - 1;
                    g_vleft[y]  = (int16)vl;
                    g_vright[y] = (int16)vr;
                    g_mid[y]    = (int16)((vl + vr) / 2);
                    g_is_virtual[y] = 1;
                }
                return;
            }
        }
    }
    /* 参考带不足 / 行差过小 -> 落入下方垂直补线兜底 */
#endif

    /* 垂直直行补线: 以基准行中点为虚拟中线, 向上垂直延伸(直行通过十字) */
    {
        int16 center = (g_left[y_bot] + g_right[y_bot]) / 2;
        for (y = TRACK_SCAN_Y_END; y <= TRACK_SCAN_Y_START; y++)
        {
            if (g_valid[y]) continue;
            g_vleft[y]  = g_left[y_bot];
            g_vright[y] = g_right[y_bot];
            g_mid[y]    = center;
            g_is_virtual[y] = 1;
        }
    }
}

/* ==================================================================
 * 取近处控制点中线 x (最靠下有效/虚拟行的中线)
 * ================================================================== */
static int16 track_get_control_mid(void)
{
    int y;
    for (y = TRACK_SCAN_Y_START; y >= TRACK_SCAN_Y_END; y--)
    {
        if (g_valid[y] || g_is_virtual[y]) return g_mid[y];
    }
    return (TRACK_W / 2);
}

/* ==================================================================
 * 主状态机 (每帧调用一次)
 * ================================================================== */
void track_process(void)
{
    int16 ctrl_mid = TRACK_W / 2;
    int y, near_valid;

    switch (g_track_state)
    {
    /* ============ 直道 STATE_NORMAL ============ */
    case STATE_NORMAL:
        /* 更新扫描窗口(道路质心+范围, 调试用) */
        track_update_window();
        /* 最长白列扫线: 从底边往上找最长白列, 再逐行扫左右边线 */
        track_scan_edges();

#if TRACK_USE_CROSS && TRACK_USE_8NEIGH
        /* ---- 十字触发A: 八邻域识别四个拐点 ---- */
        if (track_check_cross(g_left, g_right, g_valid))
        {
            g_track_state   = STATE_CROSS;
            g_cross_timer   = 0;
            g_cross_recover = 0;
            g_win_frozen    = 0;
            track_fill_cross_line();               /* PID立即改用虚拟补线中线 */
            ctrl_mid = track_get_control_mid();
            break;
        }
#endif

#if TRACK_USE_CROSS
        /* ---- 十字触发W: 白段宽度突增(不用八邻域拐点) ---- */
        if (track_check_cross_wide())
        {
            g_track_state   = STATE_CROSS;
            g_cross_timer   = 0;
            g_cross_recover = 0;
            g_win_frozen    = 0;
            track_fill_cross_line();               /* PID立即改用虚拟补线中线 */
            ctrl_mid = track_get_control_mid();
            break;
        }

        /* ---- 十字触发B(兜底): 连续>=N行扫不到成对边线 ---- */
        {
            int miss = 0;
            for (y = TRACK_SCAN_Y_START; y >= TRACK_SCAN_Y_END; y--)
            {
                if (g_valid[y]) miss = 0;
                else
                {
                    miss++;
                    if (miss >= TRACK_CROSS_MISS_ROW) break;
                }
            }
            if (miss >= TRACK_CROSS_MISS_ROW)
            {
                g_track_state = STATE_CROSS;
                g_cross_timer   = 0;
                g_cross_recover = 0;
                g_win_frozen    = 0;
                track_fill_cross_line();           /* PID立即改用虚拟补线中线 */
                ctrl_mid = track_get_control_mid();
                break;
            }
        }
#endif

#if TRACK_USE_8NEIGH
        /* ---- 环岛触发: 八邻域拐点检测 ---- */
        {
            track_point_t corners[64];
            int cnt = 0;
            cnt += track_detect_corners_edge(g_left,  g_valid, &corners[cnt], 64 - cnt);
            cnt += track_detect_corners_edge(g_right, g_valid, &corners[cnt], 64 - cnt);
            cnt  = track_corner_denoise(corners, cnt);

            if (track_check_ring(corners, cnt))
            {
                /* 入口拐点-内圆拐点连线作为环岛初始航向(两点已保存) */
                g_track_state   = STATE_RING;
                g_ring_lost_cnt = 0;
                track_scan_edges();                /* 重新扫线, 供环岛内侧跟踪 */
                ctrl_mid = track_ring_control();
                break;
            }
        }
#endif

        /* ---- 正常行驶: 左右边线中点 -> 中点PID ---- */
        track_midline_normal();
        ctrl_mid = track_get_control_mid();
        break;

    /* ============ 环岛 STATE_RING ============ */
    case STATE_RING:
        /* 八邻域拐点检测此处关闭; 放弃中点算法, 跟踪内侧边线固定偏移 */
        track_scan_edges();
        ctrl_mid = track_ring_control();           /* 内部处理退出判定 */
        break;

    /* ============ 十字 STATE_CROSS ============ */
    case STATE_CROSS:
        /* 八邻域拐点检测保持关闭(十字无有效边线, 不产生拐点) */
        track_scan_edges();

        /* 下半近处是否存在真实边线 */
        near_valid = 0;
        for (y = TRACK_NEAR_Y_START; y <= TRACK_NEAR_Y_END; y++)
        {
            if (g_valid[y]) near_valid++;
        }

        if (near_valid >= TRACK_CROSS_NEAR_MIN)
        {
            /* 分支A: 下半近处仍有真实边线
             *   最长白列继续运行, 持续更新窗口;
             *   向上扫线找不到成对边线; PID用虚拟补线中线, 丢弃扫线无效结果。*/
            g_win_frozen = 0;
            track_update_window();
            track_fill_cross_line();               /* 正常补线 */
            memcpy(g_mid_prev, g_mid, sizeof(g_mid));  /* 快照, 冻结兜底用 */
        }
        else
        {
            /* 分支B: 下半近处边线也消失(下面两个拐点消失)
             *   停止更新窗口, 锁存上一帧补线和中线不动(不重扫重填);
             *   启动补线最大帧数计时器。*/
            g_win_frozen = 1;
            g_cross_timer++;
            if (g_cross_timer >= TRACK_CROSS_TIMEOUT)
            {
                g_track_stop = 1;                  /* 安全兜底: 停车 */
            }
        }

        ctrl_mid = track_get_control_mid();

        /* 退出十字: 底部连续有效行恢复(下半重新扫描出成对真实边线) */
        {
            int consec = 0;
            for (y = TRACK_SCAN_Y_START; y >= TRACK_SCAN_Y_END; y--)
            {
                if (g_valid[y]) consec++;
                else break;
            }
            if (consec >= TRACK_CROSS_EXIT_ROWS)
            {
                g_cross_recover++;
                if (g_cross_recover >= TRACK_CROSS_EXIT_FRAMES)
                {
                    g_track_state = STATE_NORMAL;  /* 全部逻辑恢复 */
                    g_cross_timer   = 0;
                    g_win_frozen    = 0;
                }
            }
            else
            {
                g_cross_recover = 0;
            }
        }
        break;

    default:
        g_track_state = STATE_NORMAL;
        break;
    }

    /* 统一输出: PID 误差 = 中线 - 图像中心 */
    g_track_mid_x = ctrl_mid;
    g_track_error = ctrl_mid - (TRACK_W / 2);
}

/* ==================================================================
 * TFT 叠加显示 (全屏灰度图上): 边线 + 中线路径
 *   图像坐标(188x120) -> 全屏(128x160)
 *   真实边线=白, 真实中线=绿, 虚拟补线中线=黄, 拐点/航向=红
 * ================================================================== */
void track_draw_debug(void)
{
#if TRACK_DEBUG_DRAW
    int y;

    /* 1. 左右真实边线(白点) */
    for (y = TRACK_SCAN_Y_END; y <= TRACK_SCAN_Y_START; y++)
    {
        if (g_valid[y])
        {
            tft180_draw_point(mapx(g_left[y]),  mapy(y), RGB565_WHITE);
            tft180_draw_point(mapx(g_right[y]), mapy(y), RGB565_WHITE);
        }
    }

    /* 2. 中线路径(真实=绿, 虚拟补线=黄) */
    for (y = TRACK_SCAN_Y_END; y < TRACK_SCAN_Y_START; y++)
    {
        if ((g_valid[y] || g_is_virtual[y]) && (g_valid[y + 1] || g_is_virtual[y + 1]))
        {
            uint16 col = g_is_virtual[y] ? RGB565_YELLOW : RGB565_GREEN;
            tft180_draw_line(mapx(g_mid[y]),  mapy(y),
                             mapx(g_mid[y + 1]), mapy(y + 1), col);
        }
    }

    /* 3. 环岛拐点(红叉) 与 初始航向连线(红) */
    if (g_entrance_ok)
    {
        int ex = mapx(g_entrance.x), ey = mapy(g_entrance.y);
        tft180_draw_line(clipx(ex - 3), clipy(ey - 3), clipx(ex + 3), clipy(ey + 3), RGB565_RED);
        tft180_draw_line(clipx(ex - 3), clipy(ey + 3), clipx(ex + 3), clipy(ey - 3), RGB565_RED);
    }
    if (g_inner_ok)
    {
        int ix = mapx(g_inner.x), iy = mapy(g_inner.y);
        tft180_draw_line(clipx(ix - 3), clipy(iy - 3), clipx(ix + 3), clipy(iy + 3), RGB565_RED);
        tft180_draw_line(clipx(ix - 3), clipy(iy + 3), clipx(ix + 3), clipy(iy - 3), RGB565_RED);
    }
    if (g_entrance_ok && g_inner_ok)
    {
        tft180_draw_line(mapx(g_entrance.x), mapy(g_entrance.y),
                         mapx(g_inner.x),    mapy(g_inner.y), RGB565_RED);
    }

    /* 3.5 十字四拐点(紫叉) */
    if (g_cross_corners_ok)
    {
        int bx0 = mapx(g_cross_l_bot.x), by0 = mapy(g_cross_l_bot.y);
        int bx1 = mapx(g_cross_l_top.x), by1 = mapy(g_cross_l_top.y);
        int bx2 = mapx(g_cross_r_bot.x), by2 = mapy(g_cross_r_bot.y);
        int bx3 = mapx(g_cross_r_top.x), by3 = mapy(g_cross_r_top.y);
        tft180_draw_line(clipx(bx0 - 3), clipy(by0 - 3), clipx(bx0 + 3), clipy(by0 + 3), RGB565_MAGENTA);
        tft180_draw_line(clipx(bx0 - 3), clipy(by0 + 3), clipx(bx0 + 3), clipy(by0 - 3), RGB565_MAGENTA);
        tft180_draw_line(clipx(bx1 - 3), clipy(by1 - 3), clipx(bx1 + 3), clipy(by1 + 3), RGB565_MAGENTA);
        tft180_draw_line(clipx(bx1 - 3), clipy(by1 + 3), clipx(bx1 + 3), clipy(by1 - 3), RGB565_MAGENTA);
        tft180_draw_line(clipx(bx2 - 3), clipy(by2 - 3), clipx(bx2 + 3), clipy(by2 + 3), RGB565_MAGENTA);
        tft180_draw_line(clipx(bx2 - 3), clipy(by2 + 3), clipx(bx2 + 3), clipy(by2 - 3), RGB565_MAGENTA);
        tft180_draw_line(clipx(bx3 - 3), clipy(by3 - 3), clipx(bx3 + 3), clipy(by3 + 3), RGB565_MAGENTA);
        tft180_draw_line(clipx(bx3 - 3), clipy(by3 + 3), clipx(bx3 + 3), clipy(by3 - 3), RGB565_MAGENTA);
    }

    /* 4. (状态/参数文字已移除: 双图显示占满屏幕, 无独立文字区) */
#endif
}

/* ==================================================================
 * 十字/环岛 诊断信息文字面板 (TFT 左上角, 白字黑底, 覆盖在灰度图上)
 *   T  = 二值化阈值       S  = 状态(0直道/1环岛/2十字)
 *   L  = 左边缘拐点数     R  = 右边缘拐点数     I = 中心黑岛(0白/1黑)
 *   LS = 左上下拐点Y差    RS = 右上下拐点Y差
 *   W  = 扫描窗口[左-右]
 *   L行 = 左边缘 最上拐点(x,y) 最下拐点(x,y)
 *   R行 = 右边缘 最上拐点(x,y) 最下拐点(x,y)
 *   未检测到用 -1,-1 表示。数据在十字误判成环岛时冻结(直道每帧刷新)。
 * ================================================================== */
void track_draw_debug_info(void)
{
    char buf[32];
#if TRACK_USE_8NEIGH
    int lt_i = 0, lb_i = 0, rt_i = 0, rb_i = 0;

    /* 拐点按 y 从小到大存储: 第0个=最上(远), 最后一个=最下(近) */
    if (g_dbg_lcnt > 0) { lt_i = 0; lb_i = (g_dbg_lcnt - 1 > 3) ? 3 : (g_dbg_lcnt - 1); }
    if (g_dbg_rcnt > 0) { rt_i = 0; rb_i = (g_dbg_rcnt - 1 > 3) ? 3 : (g_dbg_rcnt - 1); }
#endif

    tft180_set_color(RGB565_WHITE, RGB565_BLACK);

    sprintf(buf, "T=%d S=%d", (int)g_track_threshold, (int)g_track_state);
    tft180_show_string(0, 0, buf);

#if TRACK_USE_8NEIGH
    sprintf(buf, "L=%d R=%d I=%d", (int)g_dbg_lcnt, (int)g_dbg_rcnt, (int)g_dbg_island);
    tft180_show_string(0, 16, buf);

    sprintf(buf, "LS=%d RS=%d", (int)g_dbg_lspan, (int)g_dbg_rspan);
    tft180_show_string(0, 32, buf);
#elif TRACK_USE_CROSS
    /* 宽臂十字调试: 近处参考宽 / 最大连续宽行数 */
    sprintf(buf, "rw=%d wc=%d", (int)g_dbg_ref_w, (int)g_dbg_wide_c);
    tft180_show_string(0, 16, buf);
#else
    /* 最长白列调试: 中心列 / 白列长度 */
    sprintf(buf, "col=%d h=%d", (int)g_dbg_wide_c, (int)g_dbg_ref_w);
    tft180_show_string(0, 16, buf);
#endif

    sprintf(buf, "W=%d-%d", (int)g_dbg_win_l, (int)g_dbg_win_r);
    tft180_show_string(0, 48, buf);

#if TRACK_USE_8NEIGH
    sprintf(buf, "L %d,%d %d,%d",
            (int)g_dbg_lc_x[lt_i], (int)g_dbg_lc_y[lt_i],
            (int)g_dbg_lc_x[lb_i], (int)g_dbg_lc_y[lb_i]);
    tft180_show_string(0, 64, buf);

    sprintf(buf, "R %d,%d %d,%d",
            (int)g_dbg_rc_x[rt_i], (int)g_dbg_rc_y[rt_i],
            (int)g_dbg_rc_x[rb_i], (int)g_dbg_rc_y[rb_i]);
    tft180_show_string(0, 80, buf);
#endif
}
