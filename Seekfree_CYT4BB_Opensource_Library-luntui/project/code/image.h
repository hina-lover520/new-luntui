/*
 * image.h
 *
 *  Created on: 2025年3月5日
 *      Author: dell
 */
#include "zf_common_headfile.h"
#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

extern uint8 base_image[MT9V03X_H][MT9V03X_W];//原始图像（用mecpy)
extern uint8 img_threshold;//输出阈值
short GetOSTU(unsigned char tmImage[MT9V03X_H][MT9V03X_W]);
void lq_sobel(unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold);
void lq_sobelAutoThreshold(unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W]);
extern unsigned short Threshold;
extern volatile uint8_t dotcnt;
void quanzhong_light(void);
int find_max(void);
void quanzhong_light_panduan(void);
void Get_mt9v03x_image_BandW (unsigned char mode);
void erosion_operation(unsigned char imageIn[MT9V03X_H][MT9V03X_W]);

/* ==================== Inverse Perspective Mapping (IPM / bird-eye) ====================
 * Warps the perspective camera image into a top-down (bird-eye) view so that lane lines
 * become straight & parallel, which makes edge/midline extraction much more robust.
 *
 * Usage:
 *   ipm_init();                                  // once at startup
 *   ipm_warp(mt9v03x_image, ipm_image);          // per frame, result in ipm_image[IPM_H][IPM_W]
 *
 * Output buffer ipm_image has the same size as the camera frame by default (IPM_W x IPM_H).
 * Calibration constants (the source trapezoid) live at the top of the IPM section in image.c.
 */
#define IPM_W   MT9V03X_W
#define IPM_H   MT9V03X_H
extern uint8 ipm_image[IPM_H][IPM_W];
void ipm_init(void);
void ipm_warp(unsigned char src[MT9V03X_H][MT9V03X_W], unsigned char dst[IPM_H][IPM_W]);

void ipm_draw_calibration_points(void);

#endif /* CODE_IMAGE_H_ */
