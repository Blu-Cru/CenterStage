package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.blucru.common.states.DrivetrainState;
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

        velocity = updatePoseEstimate();
        heading = pose.heading.toDouble();
    }

    public void write() {
        switch(state) {
            case TELEOP:
                break;
            case DRIVE_TO_POSITION:
                driveToPosition(targetPose);
                break;
            case FOLLOWING_TRAJECTORY:
                break;
        }
    }


    public void teleOpDrive(double x, double y, double rotate) {
        boolean turning = Math.abs(rotate) > 0.02;
        boolean wasJustTurning = Math.abs(lastRotate) > 0.02;
        boolean movingTranslation = new Vector2d(x, y).norm() > 0.05;
        boolean stopped = lastDriveVector.norm() < 0.05 && !movingTranslation && velocity.linearVel.norm() < 10.0;

        if(turning) // if driver is turning, drive with turning normally
            drive(x, y, rotate);
        else if(wasJustTurning) // if driver just stopped turning, drive to new target heading
            driveToHeading(x, y, calculateNewTargetHeading());
        else if(stopped) // if drivetrain is stopped, drive to current heading
            driveToHeading(0, 0, heading);
        else // drive, turning to target heading
            driveToHeading(x, y, targetHeading);

        state = State.TELEOP;

        // recording last turn input
        lastRotate = rotate;
    }

    public void drive(double x, double y, double rotate) {
        Vector2d driveVector = calculateDriveVector(new Vector2d(x, y));

        Pose2d drivePose = processDrivePower(new Pose2d(driveVector, rotate));
        Pose2d staticDrivePose = processStaticFriction(drivePose);

        setDrivePowers(new PoseVelocity2d(staticDrivePose.position, staticDrivePose.heading.toDouble());
    }

    public void driveToHeading(double x, double y, double targetHeading) {
        this.targetHeading = targetHeading;
        double rotate = getPIDRotate(heading, targetHeading);

        drive(x, y, rotate);
    }

    // apply slew rate limiter to drive vector
    public Vector2d calculateDriveVector(Vector2d input) {
        if (fieldCentric)
            input = Rotation2d.fromDouble(-heading).times(input);
//            input = input.rotated(-heading); // rotate input vector to match robot heading if field centric
        else
            input = Rotation2d.fromDouble(-Math.PI/2).times(input);
//            input = input.rotated(Math.toRadians(-90)); // rotate to match robot coordinates (x forward, y left)

        // scale acceleration to match drive power
        // maximum change in the drive vector per second at the drive power
        double scaledMaxAccelVectorDelta = MAX_ACCEL_DRIVE_DELTA / drivePower;
        double scaledMaxDecelVectorDelta = MAX_DECEL_DRIVE_DELTA / drivePower;

        // calculate the change between the last drive vector and the current drive vector
        Vector2d driveVectorDelta = input.minus(lastDriveVector);

        double limitedDriveVectorDeltaMagnitude;
        boolean decelerating = input.norm() < lastDriveVector.norm();

        if(decelerating) {
            // if we are decelerating, limit the delta to the max decel delta
            limitedDriveVectorDeltaMagnitude = Range.clip(driveVectorDelta.norm(), 0, (scaledMaxDecelVectorDelta * dt / 1000.0));
        } else {
            // otherwise, limit the delta to the max accel delta
            limitedDriveVectorDeltaMagnitude = Range.clip(driveVectorDelta.norm(), 0, (scaledMaxAccelVectorDelta * dt / 1000.0));
        }

        // scale the drive vector delta to the limited magnitude
        Vector2d scaledDriveVectorDelta = driveVectorDelta.div(driveVectorDelta.norm()).times(limitedDriveVectorDeltaMagnitude);

        Vector2d driveVector;
        if(driveVectorDelta.norm() == 0) // catch divide by zero
            driveVector = lastDriveVector;
        else
            // add the scaled change in drive vector to the last drive vector
            driveVector = lastDriveVector.plus(scaledDriveVectorDelta);

        // record the drive vector for the next loop
        lastDriveVector = driveVector;

        return driveVector; // return the new drive vector
    }

    public Pose2d processStaticFriction(Pose2d drivePose) {
        Vector2d driveVector = drivePose.position;
        boolean robotStopped = velocity.linearVel.norm() < STATIC_TRANSLATION_VELOCITY_TOLERANCE && Math.abs(velocity.angVel) < STATIC_HEADING_VELOCITY_TOLERANCE;

        if(robotStopped && driveVector.norm() != 0) {
            double angle = driveVector.angleCast().toDouble();
            double staticMinMagnitude =
                    STRAFE_kStatic * FORWARD_kStatic
                            /
                            Math.sqrt(STRAFE_kStatic * Math.cos(angle) * STRAFE_kStatic * Math.cos(angle) + FORWARD_kStatic * Math.sin(angle) * FORWARD_kStatic * Math.sin(angle));
            double newDriveMagnitude = staticMinMagnitude + (1-staticMinMagnitude) * driveVector.norm();
            return new Pose2d(driveVector.div(driveVector.norm()).times(newDriveMagnitude), drivePose.heading);
        } else return drivePose;
    }

    public Pose2d processDrivePower(Pose2d drivePose) {
        return new Pose2d(drivePose.position.times(drivePower), drivePose.heading.toDouble() * drivePower);
    }

    public void driveToPosition(Pose2d targetPosition) {
        translationPID.setTargetPosition(targetPosition.position);
        Vector2d rawDriveVector = translationPID.calculate(pose.position);

        driveToHeading(rawDriveVector.component1(), rawDriveVector.component2(), targetPosition.heading.toDouble());
    }

    public void telemetry(Telemetry telemetry) {

    }
}
