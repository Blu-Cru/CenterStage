package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain;


import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.DrivetrainState;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization.FusedLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.util.DrivetrainTranslationPID;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class Drivetrain extends SampleMecanumDrive implements Subsystem {
    public static double
            MAX_ACCEL_DRIVE_DELTA = 3.5,
            MAX_DECEL_DRIVE_DELTA = 10.0, // magnitude per second at power 1 for slew rate limiter

            HEADING_DECELERATION = 12, // radians per second squared, for calculating new target heading after turning
            HEADING_P = 1.5, HEADING_I = 0, HEADING_D = 0.07, // PID constants for heading
            HEADING_PID_TOLERANCE = 0.05, // radians

            DISTANCE_P = 0.15, DISTANCE_I = 0, DISTANCE_D = 0.04, // PID constants for distance sensors
            DISTANCE_PID_ANGLE_TOLERANCE = 0.5, // radians
            OUTTAKE_DISTANCE = 3.6, // correct distance for outtake for distance PID

            TRANSLATION_P = 0.28, TRANSLATION_I = 0, TRANSLATION_D = 0.03, TRANSLATION_TOLERANCE = 0.4, // PID constants for translation

            STATIC_TRANSLATION_VELOCITY_TOLERANCE = 15.0, // inches per second
            STATIC_HEADING_VELOCITY_TOLERANCE = Math.toRadians(100), // radians per second
            STRAFE_kStatic = 0.08, FORWARD_kStatic = 0.05, // feedforward constants for static friction

            TRAJECTORY_FOLLOWER_ERROR_TOLERANCE = 12.0; // inches to shut down auto

    public DrivetrainState drivetrainState;
    boolean isTeleOp;
    boolean intakingInAuto;
    public double drivePower = 0.5;
    double dt;
    public Pose2d pose;
    Pose2d lastPose;
    public Pose2d velocity;
    double lastTime;

    public DrivetrainTranslationPID translationPID;
    public Pose2d targetPose;
    public FusedLocalizer fusedLocalizer;

    PIDController headingPID;
    double targetHeading = 0;
    double heading; // estimated field heading (0 is facing right, positive is counterclockwise)
    double imuHeading; // heading retrieved from IMU
    double odoHeading; // heading retrieved from odometry
    public boolean fieldCentric; // whether the robot is field centric or robot centric

    Vector2d lastDriveVector; // drive vector in previous loop
    double lastRotateInput; // rotate input in previous loop

    public Drivetrain(HardwareMap hardwareMap, boolean isTeleOp) {
        super(hardwareMap);
        this.drivetrainState = DrivetrainState.TELEOP;
        this.isTeleOp = isTeleOp;
        this.intakingInAuto = false;
        headingPID = new PIDController(HEADING_P, HEADING_I, HEADING_D);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);

        translationPID = new DrivetrainTranslationPID(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D, TRANSLATION_TOLERANCE);
        fusedLocalizer = new FusedLocalizer(getLocalizer(), hardwareMap);
        pose = new Pose2d(0,0,0);
        lastPose = Globals.START_POSE;
        lastDriveVector = new Vector2d(0,0);
        velocity = new Pose2d(0,0,0);
        fieldCentric = true;
        lastRotateInput = 0;
        drivetrainState = DrivetrainState.TELEOP;
    }

    public void init() {
        lastTime = System.currentTimeMillis();
        heading = getOdoHeading();

        if(isTeleOp) {
            initializePose();

//            headingMotionProfile = new MotionProfile(heading, heading);
        }

        pose = this.getPoseEstimate();
        fusedLocalizer.init();
    }

    public void read() {
        dt = System.currentTimeMillis() - lastTime;
        lastTime = System.currentTimeMillis();

        fusedLocalizer.update();

        lastPose = pose;
        pose = this.getPoseEstimate();
        velocity = getPoseVelocity();
//        velocity = getPoseVelocity();
        heading = getOdoHeading();
    }

    public void write() {
        switch(drivetrainState) {
            case TELEOP:
                break;
            case DRIVE_TO_POSITION:
                driveToPosition(targetPose);
                break;
            case FOLLOWING_TRAJECTORY:
                updateTrajectory();
                break;
        }
    }

    public void teleOpDrive(double x, double y, double rotate) {
        boolean turning = Math.abs(rotate) > 0.02;
        boolean wasJustTurning = Math.abs(lastRotateInput) > 0.02;
        boolean movingTranslation = new Vector2d(x, y).norm() > 0.05;
        boolean stopped = lastDriveVector.norm() < 0.05 && !movingTranslation && velocity.vec().norm() < 10.0;

        if(turning) // if driver is turning, drive with turning normally
            driveScaled(x, y, rotate);
        else if(wasJustTurning) // if driver just stopped turning, drive to new target heading
            driveToHeadingScaled(x, y, calculateNewTargetHeading());
        else if(stopped) // if drivetrain is stopped, drive to current heading
            driveToHeadingScaled(0, 0, heading);
        else // drive, turning to target heading
            driveToHeadingScaled(x, y, targetHeading);

        drivetrainState = DrivetrainState.TELEOP;

        // recording last turn input
        lastRotateInput = rotate;
    }

    public void driveScaled(double x, double y, double rotate) {
        Vector2d driveVector = calculateDriveVector(new Vector2d(x, y));

        Pose2d drivePose = scaleByDrivePower(new Pose2d(driveVector, rotate));
        Pose2d staticDrivePose = processStaticFriction(drivePose);

        setWeightedDrivePower(staticDrivePose);
    }

    public void driveClip(double x, double y, double rotate) {
        Vector2d driveVector = calculateDriveVector(new Vector2d(x, y));

        Pose2d drivePose = clipByDrivePower(new Pose2d(driveVector, rotate));
        Pose2d staticDrivePose = processStaticFriction(drivePose);

        setWeightedDrivePower(staticDrivePose);
    }

    public void driveToHeadingScaled(double x, double y, double targetHeading) {
        this.targetHeading = targetHeading;
        double rotate = getPIDRotate(heading, targetHeading);

        Vector2d driveVector = calculateDriveVector(new Vector2d(x, y));

        Pose2d drivePose = new Pose2d(driveVector.times(drivePower), Range.clip(rotate, -drivePower, drivePower));
        Pose2d staticDrivePose = processStaticFriction(drivePose);

        setWeightedDrivePower(staticDrivePose);
    }

    public void driveToHeadingClip(double x, double y, double targetHeading) {
        this.targetHeading = targetHeading;
        double rotate = getPIDRotate(heading, targetHeading);
        
        driveClip(x, y, rotate);
    }

    // apply slew rate limiter to drive vector
    private Vector2d calculateDriveVector(Vector2d input) {
        if (fieldCentric)
            input = input.rotated(-heading); // rotate input vector to match robot heading if field centric
        else
            input = input.rotated(Math.toRadians(-90)); // rotate to match robot coordinates (x forward, y left)

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

    private Pose2d processStaticFriction(Pose2d drivePose) {
        Vector2d driveVector = drivePose.vec();
        boolean robotStopped = velocity.vec().norm() < STATIC_TRANSLATION_VELOCITY_TOLERANCE && Math.abs(velocity.getHeading()) < STATIC_HEADING_VELOCITY_TOLERANCE;

        if(robotStopped && driveVector.norm() != 0) {
            double angle = driveVector.angle();
            double staticMinMagnitude =
                    STRAFE_kStatic * FORWARD_kStatic
                            /
                    Math.sqrt(STRAFE_kStatic * Math.cos(angle) * STRAFE_kStatic * Math.cos(angle) + FORWARD_kStatic * Math.sin(angle) * FORWARD_kStatic * Math.sin(angle));
            double newDriveMagnitude = staticMinMagnitude + (1-staticMinMagnitude) * driveVector.norm();
            return new Pose2d(driveVector.div(driveVector.norm()).times(newDriveMagnitude), drivePose.getHeading());
        } else return drivePose;
    }

    private Pose2d scaleByDrivePower(Pose2d drivePose) {
        return drivePose.times(drivePower);
    }

    private Pose2d clipByDrivePower(Pose2d drivePose) {
        double newX = Range.clip(drivePose.getX(), -drivePower, drivePower);
        double newY = Range.clip(drivePose.getY(), -drivePower, drivePower);
        double newHeading = Range.clip(drivePose.getHeading(), -drivePower, drivePower);
        return new Pose2d(newX, newY, newHeading);
    }

    public void driveToPosition(Pose2d targetPosition) {
        translationPID.setTargetPosition(targetPosition.vec());
        Vector2d rawDriveVector = translationPID.calculate(pose.vec());

        driveToHeadingClip(rawDriveVector.getX(), rawDriveVector.getY(), targetPosition.getHeading());
    }

    // set the component of a vector in a direction
    private Vector2d setComponent(Vector2d vector, double component, double angle) {
        vector = vector.rotated(-angle); // rotate so the component is in the x direction
        vector = new Vector2d(component, vector.getY()); // set the x component
        return vector.rotated(angle); // rotate back
    }

    public double calculateNewTargetHeading() {
        // vf^2 = vi^2 + 2a(xf - xi)
        // 0 = velocity * velocity + 2 * -HEADING_DECELERATION * (targetHeading - heading)
        // velocity * velocity = 2 * HEADING_DECELERATION * (targetHeading - heading)
        // target heading = heading + 0.5 * velocity * velocity / HEADING_DECELERATION
        // use sign of velocity to determine to add or subtract

        return Angle.norm(heading + Math.signum(velocity.getHeading()) * 0.5 * velocity.getHeading() * velocity.getHeading() / HEADING_DECELERATION);
    }

    public boolean followerIsWithinTolerance() {
        return getTrajectoryFollowerError() < TRAJECTORY_FOLLOWER_ERROR_TOLERANCE;
    }

    public double getTrajectoryFollowerError() {
        Pose2d lastError = getLastError();
        return lastError.vec().norm();
    }

    public void idle() {
        drivetrainState = DrivetrainState.TELEOP;
    }

    public void lockTo(Pose2d pose) {
        fieldCentric = true;
        drivetrainState = DrivetrainState.DRIVE_TO_POSITION;
        setTargetPose(pose);
    }

    public void setDrivePower(double power) {
        drivePower = Range.clip(power, 0.15, 1.0);
    }

//    public void setDistancePID(double p, double i, double d) {
//        distancePID.setPID(p, i, d);
//    }

    public void setTurnPID(double p, double i, double d) {
        headingPID.setPID(p, i, d);
    }

    private double getPIDRotate(double heading, double target) {
        if(heading - target < -Math.PI) heading += 2*Math.PI;
        else if(heading - target > Math.PI) heading -= 2 * Math.PI;

        if(Math.abs(heading - target) < HEADING_PID_TOLERANCE) return 0;
        else return Range.clip(headingPID.calculate(heading, target), -1, 1);
    }

    public double getOdoHeading() {
        double heading = getPoseEstimate().getHeading();
        if(heading > Math.PI) {
            heading -= 2*Math.PI;
        } else if(heading < -Math.PI) {
            heading += 2*Math.PI;
        }
        return heading;
    }

//    public double getIMUHeading() {
//        double heading = getExternalHeading();
//        if(heading > Math.PI) {
//            heading -= 2*Math.PI;
//        } else if(heading < -Math.PI) {
//            heading += 2*Math.PI;
//        }
//        return heading;
//    }

    public void setTeleDrivePower(RobotState robotState, Gamepad gamepad) {
        boolean slow = gamepad.left_trigger > 0.1,
                fast = gamepad.right_trigger > 0.1;

        double slowPower, fastPower, normalPower;
        switch (robotState) {
            case RETRACT: slowPower = 0.4; fastPower = 1.0; normalPower = 0.9; break;
            case INTAKING: slowPower = 0.3; fastPower = 1.0; normalPower = 0.85; break;
            case LIFTING: slowPower = 0.3; fastPower = 0.7; normalPower = 0.6; break;
            case OUTTAKING: slowPower = 0.25; fastPower = 0.8; normalPower = 0.5; break;
            case OUTTAKE_WRIST_RETRACTED: slowPower = 0.3; fastPower = 0.7; normalPower = 0.7; break;
            case RETRACTING: slowPower = 0.3; fastPower = 0.8; normalPower = 0.75; break;
            default: slowPower = 0.3; fastPower = 0.8; normalPower = 0.5; break;
        }

        if(slow && fast) setDrivePower(normalPower);
        else if(slow) setDrivePower(slowPower);
        else if(fast) setDrivePower(fastPower);
        else setDrivePower(normalPower);
    }

    // resets heading
    public void resetHeading(double heading) {
//        resetIMU(heading);
        fusedLocalizer.resetHeading(heading);
    }

    // set initial pose from auto
    public void initializePose() {
        setPoseEstimate(Globals.START_POSE);
        setExternalHeading(Globals.START_POSE.getHeading());
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        this.targetHeading = targetPose.getHeading();
    }

    public boolean isAtTargetPose() {
        boolean translationAtTarget = pose.vec().distTo(targetPose.vec()) < TRANSLATION_TOLERANCE && Math.abs(heading - targetHeading) < HEADING_PID_TOLERANCE;
        boolean velocityAtTarget = velocity.vec().norm() < 7.0;
        return translationAtTarget && velocityAtTarget;
    }

    public void updateAprilTags(AprilTagProcessor processor) {
        try {
            fusedLocalizer.updateAprilTags(processor);

        } catch (Exception e) {
            Log.e("Drivetrain", "Error updating April tags: " + e.getMessage());
        }
    }

//    public void startReadingDistance() {
//        readingDistance = true;
//        errors.clear();
//    }
//
//    public void stopReadingDistance() {
//        readingDistance = false;
//    }

    public double getHeading() {return heading;}

    public boolean isStopped() {
        Pose2d poseVelocity = getPoseVelocity();
        if(poseVelocity == null) return true;
        else return poseVelocity.vec().norm() < 0.1 && Math.abs(poseVelocity.getHeading()) < 0.1;
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return super.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    idle();
                }); // idle the drivetrain before building a trajectory
    }

    public void ftcDashDrawPose(Pose2d pose) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay()
                        .setStroke("#1d38cf");
        DashboardUtil.drawRobot(fieldOverlay, pose);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void ftcDashDrawCurrentPose() {
        ftcDashDrawPose(pose);
    }

    public void telemetry(Telemetry telemetry) {
        if(isTeleOp) {
            telemetry.addData("drive power", drivePower);
            telemetry.addData("field centric", fieldCentric);
            telemetry.addData("target heading", targetHeading);
        }

        telemetry.addData("DRIVETRAIN STATE:", drivetrainState);
        telemetry.addData("heading", heading);
        telemetry.addData("pose x", pose.getX());
        telemetry.addData("pose y", pose.getY());
        telemetry.addData("velocity x", velocity.getX());
        telemetry.addData("velocity y", velocity.getY());
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("dt", dt);
        telemetry.addData("last drive vector", lastDriveVector);
        telemetry.addData("last drive vector magnitude", lastDriveVector.norm());
        telemetry.addData("odo heading", odoHeading);
        telemetry.addData("imu heading", imuHeading);
//        distanceSensors.telemetry(telemetry);
    }
}
