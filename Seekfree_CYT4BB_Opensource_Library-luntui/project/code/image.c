/*
 * image.c
 *
 *  Created on: 2025年3月5日
 *      Author: dell
 */
#include "zf_common_headfile.h"
#include "image.h"
#include "find.h"

uint8 base_image[MT9V03X_H][MT9V03X_W];

/************************************************************************
*  函数名称：void Get_mt9v03x_image_BandW (unsigned char mode)
*  功能说明：图像二值化到mt9v03x_image_BandW[][]
*  参数说明：mode  ：
*    0：使用大津法阈值
*    1：使用平均阈值
*    2: sobel 算子改进型  手动阈值，同时输出改为提取边沿的图像
*    3：sobel 算子改进型   动态阈值，同时输出改为提取边沿的图像
*  函数返回：无
*  修改时间：2022年3月29日
*  备    注：  Get_mt9v03x_image_BandW(0); //使用大津法二值化
************************************************************************/
void Get_mt9v03x_image_BandW (unsigned char mode)
{
  unsigned short i = 0, j = 0;
  unsigned short Threshold = 0;
  unsigned long tv = 0;

  if (mode == 0)
  {
      Threshold = GetOSTU(mt9v03x_image);  //大津法阈值
  }
  if (mode == 1)
  {
    //累加
    for (i = 0; i < MT9V03X_H; i++)
    {
      for (j = 0; j < MT9V03X_W; j++)
      {
        tv += mt9v03x_image[i][j];   //累加
      }
    }
    Threshold =(unsigned short)(tv / MT9V03X_H / MT9V03X_W);   //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
    Threshold = Threshold+10;      //此处阈值设置，根据环境的光线来设定
  }
  else if (mode == 2)
  {
    Threshold = 80;                          //手动调节阈值
    lq_sobel(mt9v03x_image, mt9v03x_image_BandW, (unsigned char) Threshold);

    return;

  }
  else if (mode == 3)
  {
    lq_sobelAutoThreshold(mt9v03x_image, mt9v03x_image_BandW);  //动态调节阈值
    return;
  }

  /* 二值化 */
   for (i = 0; i < MT9V03X_H; i++)
  {
    for (j = 0; j < MT9V03X_W; j++)
    {
      if (mt9v03x_image[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
          mt9v03x_image_BandW[i][j] = 255;
      else
          mt9v03x_image_BandW[i][j] = 0;
    }
  }
}


/************************************************************************
*  函数名称：short GetOSTU (unsigned char tmImage[MT9V03X_H][MT9V03X_W])
*  功能说明：大津法求阈值大小
*  参数说明：tmImage ： 图像数据
*  函数返回：无
*  修改时间：2022年3月29日
*  备    注：  GetOSTU(mt9v03x_image);//大津法阈值
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代 1
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像
的比例w0，        并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背
景像素)  * 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7) 将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
************************************************************************/
short GetOSTU (unsigned char tmImage[MT9V03X_H][MT9V03X_W])
{
  signed short i, j;
  unsigned long Amount = 0;
  unsigned long PixelBack = 0;
  unsigned long PixelshortegralBack = 0;
  unsigned long Pixelshortegral = 0;
  signed long PixelshortegralFore = 0;
  signed long PixelFore = 0;
  float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
  signed short MinValue, MaxValue;
  signed short Threshold = 0;
  unsigned char HistoGram[256];
  unsigned long colorcount=0;//灰度个数

  for (j = 0; j < 256; j++)
    HistoGram[j] = 0; //初始化灰度直方图

  for (j = 0; j < MT9V03X_H; j++)
  {
    for (i = 0; i < MT9V03X_W; i++)
    {
      HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    }
  }
  for(i=0;i<256;i++)
  {
    if(HistoGram[i]>0)
    {
      colorcount++;
    }
  }
      
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MaxValue] == 0; MaxValue--); //获取最大灰度的值

  if (MaxValue == MinValue)
    return MaxValue;         // 图像中只有一个颜色
  if (MaxValue-MinValue <=5 &&colorcount==2)
    return (MinValue+MaxValue)/2;        // 图像中只有二个颜色

  for (j = MinValue; j <= MaxValue; j++)
    Amount += HistoGram[j];        //  像素总数

  Pixelshortegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    Pixelshortegral += HistoGram[j] * j;        //灰度值总数
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];     //前景像素点数
    PixelFore = Amount - PixelBack;           //背景像素点数
    OmegaBack = (float) PixelBack / Amount;  //前景像素百分比
    OmegaFore = (float) PixelFore / Amount;  //背景像素百分比
    PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
    PixelshortegralFore = Pixelshortegral - PixelshortegralBack;   //背景灰度值
    MicroBack = (float) PixelshortegralBack / PixelBack;          //前景灰度百分比
    MicroFore = (float) PixelshortegralFore / PixelFore;          //背景灰度百分比
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差===前景像素个数*背景像素个数*（前景灰度比-背景灰度值比）的平方。
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //返回最佳阈值;
}


/*************************************************************************
*  函数名称：lq_sobel
*  功能说明：基于soble边沿检测算子的一种边沿检测
* @param    imageIn    输入数组
*           imageOut   输出数组      保存的二值化后的边沿信息
*           Threshold  阈值
*  函数返回：void
*  修改时间：2020年3月10日
*  备    注：无
*************************************************************************/
void lq_sobel (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold)
{
  //* 卷积核大小 // 
  short KERNEL_SIZE = 3;
  short xStart = KERNEL_SIZE / 2;
  short xEnd = MT9V03X_W - KERNEL_SIZE / 2;
  short yStart = KERNEL_SIZE / 2;
  short yEnd = MT9V03X_H - KERNEL_SIZE / 2;
  short i, j, k;
  short temp[4];
  for (i = yStart; i < yEnd; i++)
  {
    for (j = xStart; j < xEnd; j++)
    {
      // 计算不同方向梯度幅值  //
      temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
        - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]                   // {-1, 0, 1},
          - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];        // {-1, 0, 1};

      temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //-1, -1, -1,
        - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]                   // 0,  0,  0,
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];       //  1,  1,  1;

      temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
        - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]             //  1,  0, -1
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1]; //  1,  1,  0

      temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
        - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]              // -1,  0,  1
          - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];  //  0,  1,  1

      temp[0] = abs(temp[0]);
      temp[1] = abs(temp[1]);
      temp[2] = abs(temp[2]);
      temp[3] = abs(temp[3]);

      // 找出梯度幅值最大值  //
      for (k = 1; k < 4; k++)
      {
        if (temp[0] < temp[k])
        {
          temp[0] = temp[k];
        }
      }

      if (temp[0] > Threshold)
      {
        imageOut[i][j] = 255;//1
      }
      else
      {
        imageOut[i][j] = 0;
      }
    }
  }
}

/*************************************************************************
*  函数名称：lq_sobel
*  功能说明：基于soble边沿检测算子的一种自动阈值边沿检测
* @param    imageIn    输入数组
*           imageOut   输出数组      保存的二值化后的边沿信息 *
*  函数返回：void
*  修改时间：2020年3月10日
*  备    注：注意需要使用  带与非门版（白色）转接座，或者新版摄像头模块
*************************************************************************/
void lq_sobelAutoThreshold (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W])
{
  //* 卷积核大小 //
  short KERNEL_SIZE = 3;
  short xStart = KERNEL_SIZE / 2;
  short xEnd   = MT9V03X_W - KERNEL_SIZE / 2;
  short yStart = KERNEL_SIZE / 2;
  short yEnd   = MT9V03X_H - KERNEL_SIZE / 2;
  short i, j, k;
  short temp[4];
  for (i = yStart; i < yEnd; i++)
  {
    for (j = xStart; j < xEnd; j++)
    {
      // 计算不同方向梯度幅值  //
      temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
        - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]       // {-1, 0, 1},
          - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

      temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
        - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};

      temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
        - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0

      temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
        - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
          - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1

      temp[0] = abs(temp[0]);
      temp[1] = abs(temp[1]);
      temp[2] = abs(temp[2]);
      temp[3] = abs(temp[3]);

      // 找出梯度幅值最大值  //
      for (k = 1; k < 4; k++)
      {
        if (temp[0] < temp[k])
        {
          temp[0] = temp[k];
        }
      }

      // 使用像素点邻域内像素点之和的一定比例    作为阈值  //
      temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
        + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
          + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

      if (temp[0] > temp[3] / 12.0f)
      {
        imageOut[i][j] = 1;
      }
      else
      {
        imageOut[i][j] = 0;
      }

    }
  }
}

void erosion_operation(unsigned char imageIn[MT9V03X_H][MT9V03X_W])
{
   int i;
   int j;
   for(i=1;i<MT9V03X_H-1;i++){
      for(j=1;j<MT9V03X_W-1;j++)
      {
        if(imageIn[i-1][j]+imageIn[i][j+1]+imageIn[i+1][j]+imageIn[i][j-1]==0)
        {
          imageIn[i][j]=0;
        }
       
      }
     
   }
}

/* =====================================================================================
 * Inverse Perspective Mapping (IPM / bird-eye transform)
 * -------------------------------------------------------------------------------------
 * Maps a trapezoidal road region (perspective view) into a rectangle (top-down view)
 * using a 4-point homography.  The 8 homography coefficients are solved once in
 * ipm_init(), then every output pixel is inverse-mapped back to the source in ipm_warp()
 * (nearest neighbour).  Source pixels that fall outside the image are set to 0 (black).
 *
 * CALIBRATION: adjust the four IPM_SRC_* corners below to the trapezoid the road/field
 * occupies in YOUR camera image.  DST is the bird-eye rectangle (defaults to the full
 * output frame).  Tip: put the car on a straight road, place 4 markers at the corners of
 * a known rectangle on the ground, read their pixel coordinates and fill them in below.
 * ===================================================================================== */

/* source trapezoid corners, camera image pixel coordinates (px) -- CALIBRATE THESE */
#define IPM_SRC_TL_X   35    /* top-left    x */
#define IPM_SRC_TL_Y   40    /* top-left    y */
#define IPM_SRC_TR_X   153   /* top-right   x */
#define IPM_SRC_TR_Y   40    /* top-right   y */
#define IPM_SRC_BR_X   184   /* bottom-right x */
#define IPM_SRC_BR_Y   119   /* bottom-right y */
#define IPM_SRC_BL_X   4    /* bottom-left  x */
#define IPM_SRC_BL_Y   119   /* bottom-left  y */

uint8 ipm_image[IPM_H][IPM_W];
static float ipm_H[9];

/* Solve an 8x8 linear system A*x = b with Gaussian elimination (partial pivoting). */
static void gauss8(float a[8][8], float b[8], float x[8])
{
    int k, i, j, p;
    float t, f, ai, ap;
    for (k = 0; k < 8; k++)
    {
        p = k;
        for (i = k + 1; i < 8; i++)
        {
            ai = a[i][k]; if (ai < 0.0f) ai = -ai;
            ap = a[p][k]; if (ap < 0.0f) ap = -ap;
            if (ai > ap) p = i;
        }
        if (p != k)
        {
            for (j = k; j < 8; j++) { t = a[k][j]; a[k][j] = a[p][j]; a[p][j] = t; }
            t = b[k]; b[k] = b[p]; b[p] = t;
        }
        for (i = k + 1; i < 8; i++)
        {
            f = a[i][k] / a[k][k];
            for (j = k; j < 8; j++) a[i][j] -= f * a[k][j];
            b[i] -= f * b[k];
        }
    }
    for (k = 7; k >= 0; k--)
    {
        x[k] = b[k];
        for (j = k + 1; j < 8; j++) x[k] -= a[k][j] * x[j];
        x[k] /= a[k][k];
    }
}

/* Compute the 3x3 homography (H[8]=1) mapping dst points -> src points (4 pairs). */
static void compute_homography(float src[4][2], float dst[4][2], float H[9])
{
    float A[8][8] = {{0}};
    float b[8] = {0};
    float h[8];
    int i;
    for (i = 0; i < 4; i++)
    {
        float u = dst[i][0], v = dst[i][1];
        float x = src[i][0], y = src[i][1];
        A[2*i][0] = u; A[2*i][1] = v; A[2*i][2] = 1.0f;
        A[2*i][6] = -x * u; A[2*i][7] = -x * v; b[2*i] = x;
        A[2*i+1][3] = u; A[2*i+1][4] = v; A[2*i+1][5] = 1.0f;
        A[2*i+1][6] = -y * u; A[2*i+1][7] = -y * v; b[2*i+1] = y;
    }
    gauss8(A, b, h);
    H[0] = h[0]; H[1] = h[1]; H[2] = h[2];
    H[3] = h[3]; H[4] = h[4]; H[5] = h[5];
    H[6] = h[6]; H[7] = h[7]; H[8] = 1.0f;
}

/* Compute the inverse-perspective homography once.  Call at startup. */
void ipm_init(void)
{
    float src[4][2] =
    {
        { (float)IPM_SRC_TL_X, (float)IPM_SRC_TL_Y },
        { (float)IPM_SRC_TR_X, (float)IPM_SRC_TR_Y },
        { (float)IPM_SRC_BR_X, (float)IPM_SRC_BR_Y },
        { (float)IPM_SRC_BL_X, (float)IPM_SRC_BL_Y }
    };
    float dst[4][2] =
    {
        { 0.0f,                0.0f                },
        { (float)(IPM_W - 1),  0.0f                },
        { (float)(IPM_W - 1),  (float)(IPM_H - 1)  },
        { 0.0f,                (float)(IPM_H - 1)  }
    };
    compute_homography(src, dst, ipm_H);
}

/* Warp src (perspective) into dst (bird-eye).  dst size = IPM_H x IPM_W. */
void ipm_warp(unsigned char src[MT9V03X_H][MT9V03X_W], unsigned char dst[IPM_H][IPM_W])
{
    int u, v;
    for (v = 0; v < IPM_H; v++)
    {
        float dv   = (float)v;
        float nx_v = ipm_H[1] * dv + ipm_H[2];   /* numerator x at u = 0 */
        float ny_v = ipm_H[4] * dv + ipm_H[5];   /* numerator y at u = 0 */
        float dn_v = ipm_H[7] * dv + 1.0f;       /* denominator  at u = 0 */
        for (u = 0; u < IPM_W; u++)
        {
            float du  = (float)u;
            float den = ipm_H[6] * du + dn_v;
            float xf  = (ipm_H[0] * du + nx_v) / den;
            float yf  = (ipm_H[3] * du + ny_v) / den;
            int xi = (int)(xf + 0.5f);
            int yi = (int)(yf + 0.5f);
            if (xi >= 0 && xi < MT9V03X_W && yi >= 0 && yi < MT9V03X_H)
                dst[v][u] = src[yi][xi];
            else
                dst[v][u] = 0;
        }
    }
}

/* Draw the 4 IPM calibration corners (colored dots) and the trapezoid outline on the
 * bottom raw image (which is shown at (0,160) scaled to 240x160).  Use it to line the
 * trapezoid up with the track visually, then adjust IPM_SRC_* in the section above.
 *   TL = YELLOW, TR = CYAN, BR = RED, BL = GREEN, outline = WHITE */
void ipm_draw_calibration_points(void)
{
    const uint8  sx[4] = { IPM_SRC_TL_X, IPM_SRC_TR_X, IPM_SRC_BR_X, IPM_SRC_BL_X };
    const uint8  sy[4] = { IPM_SRC_TL_Y, IPM_SRC_TR_Y, IPM_SRC_BR_Y, IPM_SRC_BL_Y };
    const uint16 sc[4] = { RGB565_YELLOW, RGB565_CYAN, RGB565_RED, RGB565_GREEN };
    int i, j, a, b, x, y;
    int dx[4], dy[4];

    for (i = 0; i < 4; i++)
    {
        dx[i] = sx[i] * 240 / MT9V03X_W;
        dy[i] = 160 + sy[i] * 160 / MT9V03X_H;
    }
    for (i = 0; i < 4; i++)                     /* outline (TL-TR-BR-BL-TL) */
    {
        j = (i + 1) % 4;
        ips200_draw_line((uint16)dx[i], (uint16)dy[i], (uint16)dx[j], (uint16)dy[j], RGB565_WHITE);
    }
    for (i = 0; i < 4; i++)                     /* corner dots (3x3) */
    {
        for (a = -1; a <= 1; a++)
        {
            for (b = -1; b <= 1; b++)
            {
                x = dx[i] + a; if (x < 0) x = 0; if (x > 239) x = 239;
                y = dy[i] + b; if (y < 0) y = 0; if (y > 319) y = 319;
                ips200_draw_point((uint16)x, (uint16)y, sc[i]);
            }
        }
    }
}
