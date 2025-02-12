#pragma once
// Last Update: 1/12/2025 - 1:31 PM
// https://wiki.penfieldrobotics.com/wiki/index.php?title=2025:Robot_IO_Map



/*
    ██╗ ██████╗     ███╗   ███╗ █████╗ ██████╗ 
    ██║██╔═══██╗    ████╗ ████║██╔══██╗██╔══██╗
    ██║██║   ██║    ██╔████╔██║███████║██████╔╝
    ██║██║   ██║    ██║╚██╔╝██║██╔══██║██╔═══╝ 
    ██║╚██████╔╝    ██║ ╚═╝ ██║██║  ██║██║     
    ╚═╝ ╚═════╝     ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     
*/
#define CAN_DO_NOT_USE 1

#define CAN_SWERVE_DRIVE_FL 1
#define CAN_SWERVE_ROTATION_FL 2
#define CAN_SWERVE_CANCODER_FL 3

#define CAN_SWERVE_DRIVE_FR 4
#define CAN_SWERVE_ROTATION_FR 5
#define CAN_SWERVE_CANCODER_FR 6

#define CAN_SWERVE_DRIVE_BR 7
#define CAN_SWERVE_ROTATION_BR 8
#define CAN_SWERVE_CANCODER_BR 9

#define CAN_SWERVE_DRIVE_BL 10
#define CAN_SWERVE_ROTATION_BL 11
#define CAN_SWERVE_CANCODER_BL 12

#define CAN_PIGEON 14


// #define IS_TESTBOARD
#ifdef IS_TESTBOARD

    #define CAN_LEFT_ELEVATOR 3
    #define CAN_RIGHT_ELEVATOR 2
    #define CAN_LEFT_CALGAE 1
    #define CAN_RIGHT_CALGAE 4

#else

    #define CAN_LEFT_ELEVATOR 15
    #define CAN_RIGHT_ELEVATOR 16

#endif



#define CAN_SLOT_17 17
#define CAN_SLOT_18 18
#define CAN_SLOT_19 19
#define CAN_HANG 20
#define CAN_SLOT_21 21

#define PWM_SLOT_0 0
#define PWM_SLOT_1 1
#define PWM_SLOT_2 2
#define PWM_SLOT_3 3
#define PWM_SLOT_4 4
#define PWM_SLOT_5 5
#define PWM_SLOT_6 6
#define PWM_SLOT_7 7
#define PWM_SLOT_8 8
#define PWM_SLOT_9 9

#define DIO_ALGAE_RETROREFLECTIVE 0
#define DIO_CORAL_RETROREFLECTIVE 7
#define DIO_ELEVATOR_TOP_LIMITSWITCH 1 // 1 on test board
#define DIO_ELEVATOR_BOTTOM_LIMITSWITCH 2 // 2 on test board
#define DIO_SLOT_4 4
#define DIO_HANG_SOLENOID_UP 5
#define DIO_HANG_HUNG 6
#define DIO_SLOT_7 7
#define DIO_SLOT_8 8
#define DIO_SLOT_9 9

#define RELAY_HANG 0
#define RELAY_SLOT_1 1
#define RELAY_SLOT_2 2
#define RELAY_SLOT_3 3

#define ANALOG_SLOT_0 0
#define ANALOG_SLOT_1 1
#define ANALOG_SLOT_2 2
#define ANALOG_SLOT_3 3