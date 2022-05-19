#ifndef CONFIG_H
#define CONFIG_H

#define VERSION "dt_r6.1"

// STEPPER AND MOTION VALUES
#define STEPPER_DEFAULT_SPEED 2350
#define STEPPER_DEFAULT_ACCEL 2000
#define STEPPER_LOW_ACCEL 1000

#define MOTION_RESUME_DELAY 2000

#define MOTION_TURN_RIGHT 1
#define MOTION_TURN_LEFT -1

#define MOTION_SETSPEED_MAX 5000
#define MOTION_SETSPEED_MIN 500

// SENSORS SETTINGS (US & REFLECTIVE)
#define REFLECTIVE_READ_INTERVAL    100       // Milliseconds
#define REFLECTIVE_AVERAGE_SAMPLES    5       // Milliseconds


// PIXY DRIVE SETTINGS
#define PIXY_DRIVE_INTERVAL         35        // Intervalo para consultar los pines (milisegundos)
#define PIXY_DRIVE_INTERSECT_TIME_STRAIGHT   500       // tiempo que fuerza el avance en una interseccion (milisegundos)
#define PIXY_DRIVE_INTERSECT_TIME_TURN       3000       // tiempo que fuerza el avance en una interseccion (milisegundos)

#define PIXY_SERIAL                 Serial1

// ENCODER SETTINGS
#define ENCODER_STEPS_PER_REV       600
#define ENCODER_WHEEL_PERIMETER     31.41    // (Cm.)
#define ENCODER_STEPS_PER_5CM       95         // (ENCODER_STEPS_PER_REV / ENCODER_WHEEL_PERIMETER * 5)
#define ENCODER_NOT_MOVING_TIMEOUT  3000

// HW PIN DEFINITIONS

#define PIN_STEPPER_RIGHT_STEP    6
#define PIN_STEPPER_RIGHT_DIR     4
#define PIN_STEPPER_RIGHT_ENABLE  5

#define PIN_STEPPER_LEFT_STEP     7
#define PIN_STEPPER_LEFT_DIR      10
#define PIN_STEPPER_LEFT_ENABLE   11

#define PIN_STATUS_LED            13
#define PIN_BUZZER                9

#define PIN_US1_TRIGGER         A8
#define PIN_US1_ECHO            A9      // PCINT17
#define PIN_US1_ECHO_INT        PCINT17

#define PIN_US2_TRIGGER         A10
#define PIN_US2_ECHO            A11     // PCINT19
#define PIN_US2_ECHO_INT        PCINT19

#define PIN_VCC36               A0
#define PIN_VCC24               A1

#define PIN_EXTRA_IO1           A2
#define PIN_EXTRA_IO2           A3
#define PIN_EXTRA_IO3           A4
#define PIN_EXTRA_IO4           A5

#define PIN_AUX1_A                16        // TX2
#define PIN_AUX1_B                17        // RX2
    
#define PIN_AUX2_A                14        // TX3
#define PIN_AUX2_B                15        // RX3

#define PIN_AUX3_A                26
#define PIN_AUX3_B                28
#define PIN_AUX3_C                30


#define PIN_REFLECTIVE_SENSOR   A4

#define PIN_PIXY_STRAIGHT       29  
#define PIN_PIXY_LEFT           31
#define PIN_PIXY_RIGHT          33
#define PIN_PIXY_LEFT90         35
#define PIN_PIXY_RIGHT90        37
#define PIN_PIXY_STOP           39
#define PIN_PIXY_INTERSECT      41
#define PIN_PIXY_AUX1           43
#define PIN_PIXY_AUX2           45

#define PIN_ENCODER_A           3
#define PIN_ENCODER_B           2

#define DRIVE_MANUAL            0
#define DRIVE_AUTO              1


#define DRIVE_NEXT_INTERSECT_STRAIGHT 0
#define DRIVE_NEXT_INTERSECT_RIGHT    1
#define DRIVE_NEXT_INTERSECT_LEFT     2




#define US_MIN_DISTANCE         20        // Centimetros
#define US_SENSORS_COUNT        2         // Cantidad de sensores US


#define TILT_ACCELEROMETER_UPDATE_INTERVAL  50
#define TILT_MAX_ANGLE_X        4
#define TILT_MAX_ANGLE_Y        4

#endif
