package org.firstinspires.ftc.teamcode.BluCru;

public class Constants {
    public static double sens                     = 0.1;
    public static double triggerSens              = 0.5;

    public static double manualServoDelta         = .01;

    /* servo variables */
    public static double driveSpeedIntake         = 0.5;
    public static double driveSpeedMoving         = 0.75;
    public static double driveSpeedPreOuttake     = 0.6;
    public static double driveSpeedOuttake        = 0.4;
    public static double driveSpeedEject          = 0.4;

    public static double slideHighPower           = 0.8;
    public static double slideMedPower            = 0.8;
    public static double slideLowPower            = 0.8;
    public static double slideBasePower           = 0.8;
    public static double slideDropConePower       = 0.3;

    public static double wheelSpeedIntake         = 0.5;
    public static double wheelSpeedOuttake        = -0.5;
    public static double wheelSpeedStop           = 0;

    public static double wristMovingPos           = 0.05;
    public static double wristDownPos             = 0.78;
    public static double wristPreOuttakePos       = 0.97;
    public static double wristOuttakePos          = 0.97;

    /* encoder tick variables */
    public static int sliderBasePos               = 0;
    public static int sliderLowPos                = 425;
    public static int sliderMedPos                = 815;
    public static int sliderHighPos               = 1115;
    public static int sliderDunkDelta             = 100;
    public static int sliderIntakeDelta           = 50;
    public static int sliderOuttakeDelta          = 150;
    public static int sliderMaxPos                = 1350;
    public static int sliderMinPos                = 0;
    public static int coneClearDelta              = 180;
    public static int sliderTurretClearPos        = 320;

    /* timer variables */
    public static double slideDownDelay           = 0.15;
    public static double slideStallDelay          = 3;
    public static double wristGrabDelay           = 0.3;
    public static double wristTurretTurnDelay     = 0.35;
}
