package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@TeleOp(name = "Drive Translational Decel test", group = "test")
public class DriveTranslationDecelTest extends BCLinearOpMode {
    enum State {
        DRIVING,
        STOPPING,
        STOPPED
    }

    Pose2d startPose = new Pose2d(0, 0, 0);
    Pose2d startVelocity = new Pose2d(0, 0, 0);
    Pose2d robotVelocity = new Pose2d(0, 0, 0);
    Pose2d endPose = new Pose2d(0, 0, 0);
    Pose2d modelSymmetricalStopPose = new Pose2d(0, 0, 0);
    Pose2d modelAsymmetricalStopPose = new Pose2d(0, 0, 0);
    Pose2d deltaPose = new Pose2d(0, 0, 0);
    double modelSymmetricalDecel = 0;
    double modelStrafeDecel = 0;
    double modelForwardDecel = 0;
    double symmetricalDecel = 0;
    double strafeDecel = 0;
    double forwardDecel = 0;
    double startTimeSecs = 0;

    State state = State.DRIVING;

    @Override
    public void initialize() {
        addDrivetrain(true);
        drivetrain.fieldCentric = false;
    }

    @Override
    public void periodic() {
        switch(state) {
            case DRIVING:
                double horz = gamepad1.left_stick_x;
                double vert = -gamepad1.left_stick_y;
                double rot = -gamepad1.right_stick_x;

                if(gamepad1.right_stick_button) {
                    drivetrain.resetHeading(Math.toRadians(90));
                }

                if(gamepad1.a) {
                    state = State.STOPPING;
                    startTimeSecs = currentSecs();
                    startPose = drivetrain.getPoseEstimate();
                    startVelocity = drivetrain.getPoseVelocity();
                }


                drivetrain.teleOpDrive(horz, vert, rot);
                break;
            case STOPPING:
                if(drivetrain.isStopped() && secsSince(startTimeSecs) > 1.5) {
                    state = State.STOPPED;
                    endPose = drivetrain.getPoseEstimate();
                    calculate();
                    gamepad1.rumble(100); // rumble the controller to indicate the end of deceleration
                }
                drivetrain.setWeightedDrivePower(new Pose2d(0, 0, 0));
                break;
            case STOPPED:
                if(gamepad1.b) {
                    state = State.DRIVING; // reset the op mode
                    gamepad1.rumble(100); // rumble the controller to indicate the reset
                }

                if(gamepad1.right_stick_button) {
                    modelSymmetricalDecel = symmetricalDecel;
                    modelStrafeDecel = strafeDecel;
                    modelForwardDecel = forwardDecel;
                    gamepad1.rumble(100); // rumble the controller to indicate the data has been saved
                }
                break;
        }
    }

    public void telemetry() {
        switch(state) {
            case DRIVING:
                telemetry.addData("Robot is driving, press a to start test", "");
                break;
            case STOPPING:
                telemetry.addData("Robot is decelerating", "");
                break;
            case STOPPED:
                telemetry.addData("Robot has stopped, press b to reset", "");
                telemetry.addData("Press right stick button to save data", "");
                telemetry.addData("SYMMETRICAL DECELERATION:", symmetricalDecel);
                telemetry.addData("STRAFE DECELERATION:", strafeDecel);
                telemetry.addData("FORWARD DECELERATION:", forwardDecel);
                telemetry.addData("ROBOT VELOCITY:", robotVelocity);
                telemetry.addData("DELTA POSE:", deltaPose);
                break;

        }
        telemetry.addData("state:", state);
        telemetry.addData("start pose:", startPose);
        telemetry.addData("start velocity:", startVelocity);
        telemetry.addData("end pose:", endPose);
        telemetry.addData("model symmetrical stop pose:", modelSymmetricalStopPose);
        telemetry.addData("model asymmetrical stop pose:", modelAsymmetricalStopPose);
        telemetry.addData("model symmetrical deceleration:", modelSymmetricalDecel);
        telemetry.addData("model strafe deceleration:", modelStrafeDecel);
        telemetry.addData("model forward deceleration:", modelForwardDecel);
    }

    public void calculate() {
        robotVelocity = new Pose2d(startVelocity.vec().rotated(-startPose.getHeading()), startVelocity.getHeading());
        deltaPose = endPose.minus(startPose);
        symmetricalDecel = getSymmetricalDeceleration();
        strafeDecel = getStrafeDeceleration();
        forwardDecel = getForwardDeceleration();

        modelSymmetricalStopPose = getModelSymmetricalStopPose();
        modelAsymmetricalStopPose = getModelAsymmetricalStopPose();
    }

    public Pose2d getModelSymmetricalStopPose() {
        Vector2d initialVelocity = startVelocity.vec();
        // x = vi^2 / 2a
        double x = Math.pow(initialVelocity.norm(), 2) / (2 * modelSymmetricalDecel);
        return startPose.plus(new Pose2d(x * Math.cos(startPose.getHeading()), x * Math.sin(startPose.getHeading()), drivetrain.calculateNewTargetHeading()));
    }

    public Pose2d getModelAsymmetricalStopPose() {
        Vector2d initialVelocity = robotVelocity.vec();
        double strafeCoast = Math.pow(initialVelocity.getY(), 2) / (2 * modelStrafeDecel) * Math.signum(initialVelocity.getY());
        double forwardCoast = Math.pow(initialVelocity.getX(), 2) / (2 * modelForwardDecel) * Math.signum(initialVelocity.getX());
        Vector2d coast = new Vector2d(forwardCoast, strafeCoast);
        return new Pose2d(startPose.vec().plus(coast.rotated(-startPose.getHeading())), drivetrain.calculateNewTargetHeading());
    }

    public double getStrafeDeceleration() {
        // vf^2 = vi^2 + 2a(xf - xi)
        // 0 = vi^2 + 2a(xf - xi)
        // -vi^2 = 2a(xf - xi)
        // -vi^2 / 2(xf - xi) = a
        return Math.pow(robotVelocity.getY(), 2) / (2 * deltaPose.vec().rotated(-startPose.getHeading()).getY());
    }

    public double getForwardDeceleration() {
        // vf^2 = vi^2 + 2a(xf - xi)
        // 0 = vi^2 + 2a(xf - xi)
        // -vi^2 = 2a(xf - xi)
        // -vi^2 / 2(xf - xi) = a
        return Math.pow(robotVelocity.getX(), 2) / (2 * deltaPose.vec().rotated(-startPose.getHeading()).getX());
    }

    public double getSymmetricalDeceleration() {
        // vf^2 = vi^2 + 2a(xf - xi)
        // 0 = vi^2 + 2a(xf - xi)
        // -vi^2 = 2a(xf - xi)
        // -vi^2 / 2(xf - xi) = a
        return Math.pow(startVelocity.vec().norm(), 2) / (2 * deltaPose.vec().norm());
    }
}
