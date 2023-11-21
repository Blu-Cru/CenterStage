package org.firstinspires.ftc.teamcode.BluCru;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double sens                     = 0.1;
    public static double triggerSens              = 0.5;

    public static double manualServoDelta         = .01;

// drive variables
    public static double driveSpeedIntake         = 0.5;
    public static double driveSpeedMoving         = 0.75;
    public static double driveSpeedPreOuttake     = 0.6;
    public static double driveSpeedOuttake        = 0.4;
    public static double driveSpeedEject          = 0.4;

// slider variables
    public static double sliderP                  = 0.01;
    public static double sliderI                  = 0;
    public static double sliderD                  = 0.0005;
    public static double sliderF                  = 0.08;
    public static int sliderBasePos               = 0;
    public static int sliderIntakePos             = 50;
    public static int sliderLowPos                = 1000;
    public static int sliderMedPos                = 1500;
    public static int sliderHighPos               = 2000;
    public static int sliderIntakeDelta           = 50;
    public static int sliderOuttakeDelta          = 150;
    public static int sliderMaxPos                = 1800;
    public static int sliderMinPos                = 0;
    public static int sliderUpWristClearPos       = 200;

// outtake servo variables
    public static double outtakeServoIntakePower = 1;
    public static double outtakeServoOuttakePower = -1;

// intake wrist variables
    public static double intakeWristRetractPos    = 0.5;
    public static double intakeWristDownPos       = 0.5;

// outtake wrist variables
    public static double outtakeWristIntakePos    = 0.52;
    public static double outtakeWristOuttakePos   = 0.6;

    /* timer variables */
    public static double slideDownDelay           = 1000;
    public static double slideStallDelay          = 3000; //milliseconds
    public static double wristGrabDelay           = 0.3;
    public static double wristTurretTurnDelay     = 0.35;

// test hardware names
    public static String motorTestName            = "intakeMotor";
    public static String servoTestName            = "intakeServo";
    public static String CRServoTestName          = "intakeCRServo";
}
