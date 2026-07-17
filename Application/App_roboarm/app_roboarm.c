//
// Created by gaoxu on 2026/7/14.
//
#include "math.h"
#include "app_roboarm.h"
#include "Emm_V5/Emm_V5.h"
//所有的单位都是mm
#define LEN_OFFEST 45  //臂向补偿
#define LEN_BIG 120 //大臂长度
#define LEN_SMALL 120 //小臂长度
#define HEIGHT 118.6 //底座高度
volatile static float len_3 = 0;//斜边长度
volatile static float len_4 = 0;//垂直投影长度
//单位：弧度
volatile static float angle_1 = 0;//大臂角1,单位：弧度
volatile static float angle_2 = 0;//大臂角2,单位：弧度
volatile static float Angle_1 = 0;//大臂角,单位：弧度  与大臂电机的角度相关，90度对应电机零点

volatile static float Angle_2 = 0;//小臂角,单位：弧度

/**
 * @brief:用来计算，并更新各个变量
 * @property ：x -> 对应x坐标  单位：mm
 * @property ： y -> 对应y坐标 单位：mm
 * @property ： z -> 对应z坐标 单位：mm
 */
void Robo_Compute(float x,float y,float z) {
    len_4 = sqrt(x*x + y*y) - LEN_OFFEST;//垂直投影
    len_3 = sqrt(pow((z-HEIGHT),2) + pow(len_4,2));//斜边长度
    float cos_angle_1 = (LEN_BIG * LEN_BIG + len_3 * len_3 - LEN_SMALL * LEN_SMALL) / (2 * LEN_BIG * len_3);
    //大臂角
    angle_1 = acos(cos_angle_1);
    if (z > HEIGHT) {
        angle_2 = atan((z - HEIGHT) / len_4);
        Angle_1 = angle_1 + angle_2 ;//单位：弧度
    }else if (z < HEIGHT) {
        angle_2 = atan(len_4 / (HEIGHT - z)) - 1.5708;
        Angle_1 = angle_1 + angle_2;//单位：弧度
    }else if (z == HEIGHT) {
        angle_2 = 0;
        Angle_1 = angle_1 + angle_2;
    }
    //小臂角
    float cos_angle_2 = (LEN_BIG * LEN_BIG + LEN_SMALL * LEN_SMALL - len_3 * len_3) / (2 * LEN_BIG * LEN_SMALL);
    Angle_2 = acos(cos_angle_2) - 1.5708;
}

/**
 * @brief:用来设置yaw轴角度
 * @property ：yaw -> 设置的角度  单位：度
 */
void Robo_Set_YawDeg(float deg) {
    Emm_V5_SetPositionDeg(1, deg * 4.5, 100);
}

/**
 * @brief:用来设置yaw轴角度
 * @property ：yaw -> 设置的角度  单位：度
 */
void Robo_Set_YawRad(float rad) {
    Emm_V5_SetPositionRad(1, rad * 4.5, 100);
}