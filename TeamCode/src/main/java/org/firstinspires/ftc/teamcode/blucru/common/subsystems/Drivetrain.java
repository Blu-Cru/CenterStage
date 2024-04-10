package org.firstinspires.ftc.teamcode.blucru.common.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.DrivetrainState;
import org.firstinspires.ftc.teamcode.blucru.common.states.Initialization;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.util.DrivetrainTranslationPID;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

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

            TRANSLATION_P = 0.3, TRANSLATION_I = 0, TRANSLATION_D = 0.002, TRANSLATION_TOLERANCE = 0.4, // PID constants for translation

            STATIC_TRANSLATION_VELOCITY_TOLERANCE = 10.0, // inches per second
            STATIC_HEADING_VELOCITY_TOLERANCE = 0.3, // radians per second
            kStaticX = 0.7, kStaticY = 0.1, // feedforward constants for static friction

            TRAJECTORY_FOLLOWER_ERROR_TOLERANCE = 12.0; // inches to shut down auto

    public DrivetrainState drivetrainState;
    boolean isTeleOp;
    boolean intakingInAuto;
    public double drivePower = 0.5;
    double dt;
    Pose2d pose;
    Pose2d velocity;
    double lastTime;

//    boolean readingDistance;
//    ArrayList<Double> errors;

    public DrivetrainTranslationPID translationPID;
    public Pose2d targetPose;

    MotionProfile headingMotionProfile;
    PIDController headingPID;
    double targetHeading = 0;
    double heading; // estimated field heading (0 is facing right, positive is counterclockwise)
    double imuHeading; // heading retrieved from IMU
    double odoHeading; // heading retrieved from odometry
    public boolean fieldCentric; // whether the robot is field centric or robot centric

    Vector2d lastDriveVector; // drive vector in previous loop
    double lastRotate; // rotate input in previous loop

//    public DistanceSensors distanceSensors;
    PIDController distancePID;

    public Drivetrain(HardwareMap hardwareMap, boolean isTeleOp) {
        super(hardwareMap);
        this.drivetrainState = DrivetrainState.IDLE;
        this.isTeleOp = isTeleOp;
        this.intakingInAuto = false;
        headingPID = new PIDController(HEADING_P, HEADING_I, HEADING_D);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);

        translationPID = new DrivetrainTranslationPID(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D, TRANSLATION_TOLERANCE);
    }

    public void init() {
        lastDriveVector = new Vector2d(0,0);

        if(isTeleOp) {
            fieldCentric = true;
            initializePose();
            velocity = new Pose2d(0,0,0);

            lastRotate = 0;
            lastTime = System.currentTimeMillis();
            heading = getOdoHeading();

            drivetrainState = DrivetrainState.IDLE;
            pose = this.getPoseEstimate();
//            headingMotionProfile = new MotionProfile(heading, heading);
        }
    }

    public void read() {
        if(isTeleOp) {
            updatePoseEstimate(); //only update pose in teleop because pose is updated in follower in auto
            dt = System.currentTimeMillis() - lastTime;
            lastTime = System.currentTimeMillis();


            velocity = getPoseVelocity();
        }

        pose = this.getPoseEstimate();
        heading = getOdoHeading();
//        if(readingDistance) {
//            distanceSensors.read(heading);
//        }
    }

    public void write() {
        switch(drivetrainState) {
            case IDLE:
                break;
            case DRIVE_TO_POSITION:
                driveToPosition(targetPose);
                break;
        }
    }

    public void teleOpDrive(double x, double y, double rotate) {
        boolean turning = Math.abs(rotate) > 0.05;
        boolean wasJustTurning = Math.abs(lastRotate) > 0.05;
        boolean driving = lastDriveVector.norm() > 0.05 || new Vector2d(x, y).norm() > 0.05;

        if(turning) // if driver is turning, drive with turning normally
            drive(x, y, rotate);
        else if(wasJustTurning) // if driver just stopped turning, drive to new target heading
            driveToHeading(x, y, calculateNewTargetHeading());
        else if(!driving) // if driver is not moving, drive with turning normally
            driveToHeading(x, y, heading);
        else // drive, turning to target heading
            driveToHeading(x, y, targetHeading);

        // recording last turn input
        lastRotate = rotate;
    }

    public Pose2d processStaticFriction(Pose2d drivePose) {
        Vector2d driveVector = drivePose.vec();
        boolean robotStopped = velocity.vec().norm() < STATIC_TRANSLATION_VELOCITY_TOLERANCE && Math.abs(velocity.getHeading()) < STATIC_HEADING_VELOCITY_TOLERANCE;

        if(robotStopped && driveVector.norm() != 0) {
            double angle = driveVector.angle();
            double staticMinMagnitude =
                    kStaticX * kStaticY
                        /
                    Math.sqrt(kStaticX * Math.cos(angle) * kStaticX * Math.cos(angle) + kStaticY * Math.sin(angle) * kStaticY * Math.sin(angle));
            double newDriveMagnitude = staticMinMagnitude + (1-staticMinMagnitude) * driveVector.norm();
            return new Pose2d(driveVector.div(driveVector.norm()).times(newDriveMagnitude), drivePose.getHeading());
        } else return drivePose;
    }

    public Pose2d processDrivePower(Pose2d drivePose) {
        return new Pose2d(drivePose.vec().times(drivePower), drivePose.getHeading() * drivePower);
    }

    public void drive(double x, double y, double rotate) {
//        drivetrainState = DrivetrainState.IDLE;
        Vector2d driveVector = calculateDriveVector(new Vector2d(x, y));

        Pose2d drivePose = processDrivePower(new Pose2d(driveVector, rotate));

        setWeightedDrivePower(drivePose);
    }

    public void driveStaticFriction(double x, double y, double rotate) {
        Vector2d driveVector = calculateDriveVector(new Vector2d(x, y));

        Pose2d drivePose = processDrivePower(new Pose2d(driveVector, rotate));
        Pose2d staticDrivePose = processStaticFriction(drivePose);


        setWeightedDrivePower(staticDrivePose);
    }

    public void driveToHeading(double x, double y, double targetHeading) {
        this.targetHeading = targetHeading;
        double rotate = getPIDRotate(heading, targetHeading);

        drive(x, y, rotate);
    }

    // rotate for field centric
    // apply slew rate limiter to drive vector
    public Vector2d calculateDriveVector(Vector2d input) {
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
            // add the scaled delta to the last drive vector
            driveVector = lastDriveVector.plus(scaledDriveVectorDelta);

        // record the drive vector for the next loop
        lastDriveVector = driveVector;
        
        return driveVector;
    }

//    public boolean imuAccurate() {
//        double correctedOdoHeading = correctHeading(odoHeading);
//        double correctedImuHeading = correctHeading(imuHeading);
//        return Math.abs(correctedOdoHeading - correctedImuHeading) < HEADING_ANGLE_TOLERANCE;
//    }

//    public double correctHeading(double heading) {
//        double correctedHeading = heading - Math.PI/2;
//
//        if(correctedHeading > Math.PI)
//            correctedHeading -= 2 * Math.PI;
//        else if (correctedHeading < -Math.PI)
//            correctedHeading += 2 * Math.PI;
//
//        return correctedHeading + Math.PI/2;
//    }

//    public void driveToDistanceToHeading(double x, double y, double targetDistance, double targetHeading) {
//        distanceSensors.read(heading);
//        Vector2d distanceVector = new Vector2d(x,y);
//
//        double component;
//        if(Math.abs(distanceSensors.getAngleError(heading - targetHeading)) < DISTANCE_PID_ANGLE_TOLERANCE && distanceSensors.sensing) {
//            component = Range.clip(distancePID.calculate(distanceSensors.distanceFromWall, targetDistance), -drivePower, drivePower);
//            // set component in direction opposite target heading
//            distanceVector = setComponent(distanceVector, component, -(heading - targetHeading));
//        }
//
//        x = distanceVector.getX();
//        y = distanceVector.getY();
//
//        driveToHeading(x, y, targetHeading);
//    }

    public void driveToPosition(Pose2d targetPosition) {
        translationPID.setTargetPosition(targetPosition.vec());
        Vector2d rawDriveVector = translationPID.calculate(pose.vec());
//        double rotate = getPIDRotate(heading, targetPosition.getHeading());
//
//        setWeightedDrivePower(new Pose2d(rawDriveVector.getX(), rawDriveVector.getY(), rotate));
        driveToHeading(rawDriveVector.getX(), rawDriveVector.getY(), targetPosition.getHeading());
    }

    // set the component of a vector in a direction
    public Vector2d setComponent(Vector2d vector, double component, double angle) {
        vector = vector.rotated(-angle); // rotate so the component is in the x direction
        vector = new Vector2d(component, vector.getY()); // set the x component
        return vector.rotated(angle); // rotate back
    }

//    public double getDistanceSensorAngleError(double targetHeading) {
//        double error = heading - targetHeading - distanceSensors.angle;
//        if(error > Math.PI) {
//            error -= 2 * Math.PI;
//        } else if (error < -Math.PI) {
//            error += 2 * Math.PI;
//        }
//
//        return Math.abs(error);
//    }

    public double calculateNewTargetHeading() {
        // vf^2 = vi^2 + 2a(xf - xi)
        // 0 = velocity * velocity + 2 * -HEADING_DECELERATION * (targetHeading - heading)
        // velocity * velocity = 2 * HEADING_DECELERATION * (targetHeading - heading)
        // target heading = heading + 0.5 * velocity * velocity / HEADING_DECELERATION
        if (velocity.getHeading() > 0) { // if velocity is positive, add to heading
            return heading + 0.5 * velocity.getHeading() * velocity.getHeading() / HEADING_DECELERATION;
        } else { // if velocity is negative, subtract to heading
            return heading - 0.5 * velocity.getHeading() * velocity.getHeading() / HEADING_DECELERATION;
        }
    }

    public boolean followerIsWithinTolerance() {
        return getTrajectoryFollowerError() < TRAJECTORY_FOLLOWER_ERROR_TOLERANCE;
    }

    public double getTrajectoryFollowerError() {
        Pose2d lastError = getLastError();
        double xError = lastError.getX();
        double yError = lastError.getY();
        return Math.sqrt(xError * xError + yError * yError);
    }

    public void idle() {
        drivetrainState = DrivetrainState.IDLE;
    }

    public void lockTo(Pose2d pose) {
        drivetrainState = DrivetrainState.DRIVE_TO_POSITION;
        setTargetPose(pose);
    }

    public void setDrivePower(double power) {
        drivePower = Range.clip(power, 0.15, 1.0);
    }

    public void setDistancePID(double p, double i, double d) {
        distancePID.setPID(p, i, d);
    }

    public void setTurnPID(double p, double i, double d) {
        headingPID.setPID(p, i, d);
    }

    public double getPIDRotate(double heading, double target) {
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

    public void setDrivePower(RobotState robotState, Gamepad gamepad) {
        boolean slow = false;
        boolean fast = false;
        if(gamepad.left_trigger > 0.1) slow = true;
        if (gamepad.right_trigger > 0.1) fast = true;

        double slowPower = 0.3;
        double fastPower = 0.8;
        double normalPower = 0.5;
        switch (robotState) {
            case RETRACT:
                slowPower = 0.4;
                fastPower = 1.0;
                normalPower = 0.9;
                break;
            case INTAKING:
                slowPower = 0.4;
                fastPower = 1.0;
                normalPower = 0.85;
                break;
            case LIFTING:
                slowPower = 0.3;
                fastPower = 0.7;
                normalPower = 0.6;
                break;
            case OUTTAKING:
                slowPower = 0.25;
                fastPower = 0.8;
                normalPower = 0.5;
                break;
            case OUTTAKE_WRIST_RETRACTED:
                slowPower = 0.3;
                fastPower = 0.7;
                normalPower = 0.7;
                break;
            case RETRACTING:
                slowPower = 0.3;
                fastPower = 0.8;
                normalPower = 0.75;
                break;
            default:
                slowPower = 0.3;
                fastPower = 0.8;
                normalPower = 0.5;
                break;
        }

        if(slow && fast) setDrivePower(normalPower);
        else if(slow) setDrivePower(slowPower);
        else if(fast) setDrivePower(fastPower);
        else setDrivePower(normalPower);
    }

    // resets heading
    public void resetHeading(double heading) {
//        resetIMU(heading);
        setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),heading));
    }

    // set initial pose from auto
    public void initializePose() {
        setPoseEstimate(Initialization.POSE);
        setExternalHeading(Initialization.POSE.getHeading());
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        this.targetHeading = targetPose.getHeading();
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
        return poseVelocity.vec().norm() < 0.1 && Math.abs(poseVelocity.getHeading()) < 0.1;
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return super.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    idle();
                }); // idle the drivetrain before building a trajectory
    }

    public void telemetry(Telemetry telemetry) {
        if(isTeleOp) {
            telemetry.addData("drive power", drivePower);
            telemetry.addData("field centric", fieldCentric);
            telemetry.addData("target heading", targetHeading);
        } else {
//            telemetry.addData("reading distance", readingDistance);
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
        telemetry.addData("odo heading", odoHeading);
        telemetry.addData("imu heading", imuHeading);
//        distanceSensors.telemetry(telemetry);
    }
}
