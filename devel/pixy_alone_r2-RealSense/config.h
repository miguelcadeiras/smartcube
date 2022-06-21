#ifndef CONFIG_H
#define CONFIG_H

#define P_SOFT_TX   A0
#define P_SOFT_RX   A1

#define PID_P_GAIN  2.0     //2
#define PID_I_GAIN  0.1     //0.1
#define PID_D_GAIN  0.4     //0.4

#define PID_RS_P_GAIN  2.0
#define PID_RS_I_GAIN  0
#define PID_RS_D_GAIN  0

#define DRIVE_MODE_PIXY           0
#define DRIVE_MODE_REALSENSE      1
#define DRIVE_MODE_PIXY_ONLY      2


#define PIXY_FEATURES_LINE_VECTOR         1
#define PIXY_FEATURES_LINE_INTERSECTION   2
#define PIXY_FEATURES_LINE_BARCODE        4

#define NO_LINE_TIMEOUT         4000
#define PIXY_INTERVAL           25

#endif
