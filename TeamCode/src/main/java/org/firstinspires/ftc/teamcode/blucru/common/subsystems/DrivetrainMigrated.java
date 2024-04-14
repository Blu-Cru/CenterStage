package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.blucru.common.states.Initialization;
import org.firstinspires.ftc.teamcode.blucru.common.util.DrivetrainTranslationPID;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

@Config
public class DrivetrainMigrated extends MecanumDrive implements Subsystem {
    public static double
            MAX_ACCEL_DRIVE_DELTA = 3.5,
            MAX_DECEL_DRIVE_DELTA = 10.0, // magnitude per second at power 1 for slew rate limiter

    HEADING_DECELERATION = 12, // radians per second squared, for calculating new target heading after turning
            HEADING_P = 1.5, HEADING_I = 0, HEADING_D = 0.07, // PID constants for heading
            HEADING_PID_TOLERANCE = 0.05, // radians

    DISTANCE_P = 0.15, DISTANCE_I = 0, DISTANCE_D = 0.04, // PID constants for distance sensors
            DISTANCE_PID_ANGLE_TOLERANCE = 0.5, // radians
            OUTTAKE_DISTANCE = 3.6, // correct distance for outtake for distance PID

    TRANSLATION_P = 0.3, TRANSLATION_I = 0, TRANSLATION_D = 0.05, TRANSLATION_TOLERANCE = 0.4, // PID constants for translation

    STATIC_TRANSLATION_VELOCITY_TOLERANCE = 15.0, // inches per second
            STATIC_HEADING_VELOCITY_TOLERANCE = Math.toRadians(100), // radians per second
            STRAFE_kStatic = 0.08, FORWARD_kStatic = 0.05, // feedforward constants for static friction

    TRAJECTORY_FOLLOWER_ERROR_TOLERANCE = 12.0; // inches to shut down auto

    enum State {
        TELEOP,
        DRIVE_TO_POSITION,
        FOLLOWING_TRAJECTORY
    }

    State state;
    boolean isTeleOp,
            intakingInAuto = false;
    public double drivePower = 0.5;
    double dt, lastTime;
    PoseVelocity2d velocity;

    DrivetrainTranslationPID translationPID;
    PIDController headingPID;
    double targetHeading = 0;
    double heading;
    public boolean fieldCentric;

    Vector2d lastDriveVector;
    double lastRotate;

    public DrivetrainMigrated(HardwareMap hardwareMap, boolean isTeleOp) {
        super(hardwareMap, Initialization.POSE);
        this.state = isTeleOp ? State.TELEOP : State.DRIVE_TO_POSITION;
        this.isTeleOp = isTeleOp;
        this.intakingInAuto = false;
        fieldCentric = true;
        headingPID = new PIDController(HEADING_P, HEADING_I, HEADING_D);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);

        translationPID = new DrivetrainTranslationPID(DISTANCE_P, DISTANCE_I, DISTANCE_D, DISTANCE_PID_ANGLE_TOLERANCE);

        lastDriveVector = new Vector2d(0, 0);
        lastRotate = 0;

        velocity = new PoseVelocity2d(new Vector2d(0,0), 0);
    }

    public void init() {
        heading = pose.heading.toDouble();
    }

    public void read() {
        dt = System.currentTimeMillis() - lastTime;
        lastTime = System.currentTimeMillis();
    }

    public void write() {

    }

    public void telemetry(Telemetry telemetry) {

    }
}
