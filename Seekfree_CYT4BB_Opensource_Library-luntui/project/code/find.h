/*
 * find.h
 *
 *  Created on: 2025??4??26??
 *      Author: 39653
 */
#include "zf_common_headfile.h"
#ifndef CODE_FIND_H_
#define CODE_FIND_H
extern float screen_x;
extern float screen_y;
extern int xx;
extern int yy;
extern int center_x;  // ???0.5f?????????2
extern int center_y;
extern uint16_t areamax;

extern short Connected_xmin[100];//??С??????
extern short Connected_ymin[100];//??С??????
extern short Connected_xmax[100];//????????
extern short Connected_ymax[100];//?????????

extern uint8_t labelBuffer[MT9V03X_W*MT9V03X_H];
extern uint8_t labelcount;
void Seek_Beacon (void);
void ConnectedComponentLabeling(unsigned char *bitmap, unsigned char * labelmap);
extern uint8 mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W];
extern uint8 display_image[MT9V03X_H][MT9V03X_W];
extern uint8 mt9v03x_image_project[MT9V03X_H][MT9V03X_W];
void Set_project(void);
void find_amax(unsigned char imageIn[MT9V03X_H][MT9V03X_W]);
void find_amax_raw(void);
void find_mid_raw(void);
void find_LR(unsigned char imageIn[MT9V03X_H][MT9V03X_W], uint8 get_max);
void show_track_line(void);
void arrive_cross(void);
int16_t track_steer_error(void);   /* steer error(px): centre col (raw longest-white-col / cross centroid) - image center, >0 = track on right */
uint8_t track_state(void);         /* track state: 0 lost / 1 normal / 2 cross */
void binarize(unsigned char imageIn[MT9V03X_H][MT9V03X_W]);
extern int16_t g_left_g;   /* 十字：十字行的左边界 */
extern int16_t g_right_g;  /* 十字：十字行的右边界 */
extern int16_t g_hight_g;  /* 十字：十字所在行(靠车最近) */
extern uint8 max;
void Set_image_twovalues(uint8 value);  //????????
void Set_image_two(void);
void Bin_Image_Filter (void);//???????
//???????,????-?????????仯,???????β??????????仯
typedef struct Queue //????????
{
 struct QNode* first;
 struct QNode* last;
}Queue;//???????????????β
void PushQueue(Queue *queue,int data);
int PopQueue(Queue *queue);
typedef struct QNode
{//???н??????
 int data;//??????
 struct QNode *next;//???????????
}QNode;
extern int16_t OFFSET0;    // ????????????????????????
extern int16_t OFFSET1;    // ?????
extern int16_t OFFSET2;    // ???????????
void eye_show (void);
float Square_detection (int16_t x,int16_t y);
#endif /* CODE_FIND_H_ */
