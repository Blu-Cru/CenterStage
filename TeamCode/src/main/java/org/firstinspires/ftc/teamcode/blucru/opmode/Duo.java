package org.firstinspires.ftc.teamcode.blucru.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.Initialization;
import org.firstinspires.ftc.teamcode.blucru.common.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Plane;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;


/*
Controls:

release everything: retracted state
a : release cone/intake
b : low height
x : medium height
y : high height
left joystick : strafe (field centric)
right joystick : turn

 */
@Config
@TeleOp(name = "Main TeleOp", group = "1")
public class Duo extends LinearOpMode {
    public static double OUTTAKE_DELAY_SECONDS = 0.5;

    Alliance alliance;

    Robot robot;
    Drivetrain drivetrain;
    Outtake outtake;
    Intake intake;
    Hanger hanger;
    Plane plane;

    private RobotState robotState;

    ElapsedTime totalTimer;
    ElapsedTime outtakeTimer;
    double lastTime, deltaTime;
    int loop;
    double boardHeading;

    boolean lastDown2 = false;
    boolean lastA2 = false;
    boolean lastRSUp2 = false;
    boolean lastRSDown2 = false;
    boolean lastUp1 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while(opModeInInit()) {
            telemetry.addData("PICK UP UR CONTROLELRS", "");
            telemetry.update();
        }

        waitForStart();


        while(opModeIsActive()) {
            deltaTime = totalTimer.milliseconds() - lastTime;
            lastTime = totalTimer.milliseconds();
            // updates states based on gamepad input
            read();

            // loop time: current time - time at start of loop

            // data for feedback
            write();
        }
    }

    public void initialize() {
        robotState = RobotState.RETRACT;
        robot = new Robot(hardwareMap);

        drivetrain = robot.addDrivetrain(true);
        outtake = robot.addOuttake();
        intake = robot.addIntake();
        hanger = robot.addHanger();
        plane = robot.addPlane();

        totalTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();

        robot.init();

        // set initial pose from auto
        drivetrain.setPoseEstimate(Initialization.POSE);
        drivetrain.setExternalHeading(Initialization.POSE.getHeading());
        alliance = Initialization.alliance;
        boardHeading = alliance == Alliance.RED ? Math.toRadians(180) : 0;
    }

    public void read() {
        robot.read();

        // DRIVING
        drivetrain.setDrivePower(robotState, gamepad1);

        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rotate = -gamepad1.right_stick_x;

        // resets heading offset (face forwards)
        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(boardHeading);
            gamepad1.rumble(100);
        }
        if(gamepad1.b) {
            if(gamepad1.left_bumper)
                drivetrain.driveToDistanceToHeading(horz, vert, Drivetrain.OUTTAKE_DISTANCE, boardHeading);
            else
                drivetrain.driveToHeading(horz, vert, boardHeading);
        } else if(gamepad1.x) {
            if(gamepad1.left_bumper)
                drivetrain.driveToDistanceToHeading(horz, vert, Drivetrain.OUTTAKE_DISTANCE, boardHeading - Math.PI);
            else
                drivetrain.driveToHeading(horz, vert, boardHeading - Math.PI);
        } else
            drivetrain.drive(horz, vert, rotate);

        // INTAKE
        if(gamepad2.left_bumper && outtake.liftIntakeReady()) {
            intake.setIntakePower(1);
            outtake.unlock();
        } else if(gamepad2.right_bumper) {
            intake.setIntakePower(-1);
            outtake.lock();
        } else {
            intake.setIntakePower(0);
            if (gamepad2.dpad_left) outtake.lockBack();
            else if(gamepad2.dpad_right) outtake.unlock();
            else outtake.lock();
        }

        // toggle intake wrist
        if(gamepad2.a && outtake.liftIntakeReady())
            intake.downIntakeWrist();
        else
            intake.retractIntakeWrist();

        switch(robotState) {
            case RETRACT:
                outtake.outtaking = false;

                if(gamepad2.x) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad2.y) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad2.b) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }
                break;
            case LIFTING:
                outtake.outtaking = true;
                if(outtake.lift.getCurrentPos() > Outtake.LIFT_WRIST_CLEAR_POS) {
                    outtake.wristRetracted = false;
                    robotState = RobotState.OUTTAKE;
                    outtakeTimer.reset();
                }

                if(gamepad2.x) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad2.y) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad2.b) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }

                if(gamepad2.a && !lastA2) {
                    robotState = RobotState.RETRACT;
                    outtake.outtaking = false;
                    outtake.lift.setMotionProfileTargetPos(0);
                }
                lastA2 = gamepad2.a;
                break;
            case OUTTAKE:
                outtake.outtaking = true;

                // TURRET CONTROL
                if(outtakeTimer.seconds() > OUTTAKE_DELAY_SECONDS) {
                    if(!outtake.wristRetracted) {
                        if (gamepad2.left_trigger > 0.1) outtake.setTurretAngle(-gamepad2.left_trigger * 60 + 270);
                        else if (gamepad2.right_trigger > 0.1) outtake.setTurretAngle(gamepad2.right_trigger * 60 + 270);
                        else outtake.setTurretAngle(270);
                    } else outtake.setTurretAngle(270);
                } else outtake.setTurretAngle(270);

                // WRIST CONTROL
                if(Math.abs(outtake.getTurretAngle() - 270) < 5) {
                    if(gamepad2.dpad_down && !lastDown2) outtake.toggleWrist();
                } else outtake.extendWrist();
                lastDown2 = gamepad2.dpad_down;

                // Change height
                if(gamepad2.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad2.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad2.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

                // increment height by one pixel
                if(gamepad2.right_stick_y < -0.3 && !lastRSUp2 && !gamepad2.right_stick_button) outtake.incrementTargetHeight(1);
                lastRSUp2 = gamepad2.right_stick_y < -0.3;
                if(gamepad2.right_stick_y > 0.3 && !lastRSDown2 && !gamepad2.right_stick_button) outtake.incrementTargetHeight(-1);
                lastRSDown2 = gamepad2.right_stick_y > 0.3;

                // retract
                if(gamepad2.a && !lastA2 && outtake.wristRetracted) {
                    robotState = RobotState.RETRACT;
                    outtake.outtaking = false;
                    outtake.lift.setMotionProfileTargetPos(0);
                }
                lastA2 = gamepad2.a;
                break;
        }

        // MANUAL SLIDE
        if(Math.abs(gamepad2.right_stick_y) > 0.1 && gamepad2.right_stick_button) {
            outtake.lift.liftState = LiftState.MANUAL;
            outtake.setManualSlidePower(-gamepad2.right_stick_y + Lift.kF);
        }

        // RESET SLIDES
        if(gamepad2.share) {
            outtake.lift.resetEncoder();
        }

        // MANUAL HANG
        if(Math.abs(gamepad2.left_stick_y) > 0.2 && gamepad2.left_stick_button)
            hanger.setPower(-gamepad2.left_stick_y);
        else
            hanger.setPower(0);

        // PLANE
        if(gamepad1.dpad_up && !lastUp1)
            plane.togglePlane();
        lastUp1 = gamepad1.dpad_up;
    }

    public void write() {
        robot.write();

        loop++;
        if(loop > 10) {
            telemetry.addData("robot state", robotState);
            robot.telemetry(telemetry);
            telemetry.addData("loop time", deltaTime);
            telemetry.addData("alliance", alliance);
            telemetry.update();
        }
    }
}
