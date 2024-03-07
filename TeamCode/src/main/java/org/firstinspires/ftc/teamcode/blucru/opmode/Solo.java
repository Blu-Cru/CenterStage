package org.firstinspires.ftc.teamcode.blucru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.Initialization;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Plane;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

@TeleOp(name = "Solo", group = "2")
public class Solo extends LinearOpMode {
    public static double OUTTAKE_DELAY_SECONDS = 0.3;
    Robot robot;
    Drivetrain drivetrain;
    Intake intake;
    Outtake outtake;
    Hanger hanger;
    Plane plane;
    Alliance alliance;
    RobotState robotState;
    ElapsedTime totalTimer;
    ElapsedTime outtakeTimer;
    ElapsedTime retractTimer;
    double boardHeading, lastTime, deltaTime, loop;
    boolean retractRequested = false;

    // timer variables
    double stopIntakeTimeSeconds = -5;

    // gamepad variables
    double lastLT = 0;
    boolean lastA1;
    boolean lastDown1;

    public void runOpMode()  throws InterruptedException {
        initialize();
        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            read();
            robot.read();
            write();
        }
    }

    public void read() {
        switch (robotState) {
            case RETRACT:
                outtake.outtaking = false;
                // drop down
                if(gamepad1.a && gamepad1.left_bumper) intake.dropToStack(3);
                else if(gamepad1.a) intake.dropToGround();
                else intake.retractIntakeWrist();

                // intake/outtake
                if(gamepad1.left_trigger > 0.3) {
                    intake.setIntakePower(gamepad1.left_trigger);
                    outtake.unlock();
                } else if(gamepad1.right_trigger > 0.3) {
                    intake.setIntakePower(-gamepad1.right_trigger);
                    outtake.lock();
                } else if(timeSince(stopIntakeTimeSeconds) < 1.0) {
                    intake.setIntakePower(-1);
                    outtake.lock();
                } else {
                    intake.setIntakePower(0);
                    outtake.lock();
                }

                // if LT was released, start timer
                if(lastLT > 0.3 && !(gamepad1.left_trigger > 0.3))  stopIntakeTimeSeconds = totalTimer.seconds();
                lastLT = gamepad1.left_trigger;

                if(gamepad1.x) {
                    outtake.outtaking = true;
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad1.y) {
                    outtake.outtaking = true;
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad1.b) {
                    outtake.outtaking = true;
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }

                break;
            case LIFTING:
                outtake.outtaking = true;

                if(outtake.lift.getCurrentPos() > Outtake.LIFT_WRIST_CLEAR_POS) {
                    outtake.wristRetracted = false;
                    robotState = RobotState.OUTTAKE_WRIST_UP;
                    outtakeTimer.reset();
                }

                if(gamepad1.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad1.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad1.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

// reverse intake
                if(gamepad1.right_trigger > 0.3 && gamepad1.left_trigger > 0.3) intake.setIntakePower(-(gamepad1.right_trigger + gamepad1.left_trigger)/2);
                else intake.setIntakePower(0);

                if(gamepad1.a && !lastA1) {
                    robotState = RobotState.RETRACT;
                    retractRequested = false;
                    outtake.outtaking = false;
                    outtake.lift.setMotionProfileTargetPos(0);
                }
                lastA1 = gamepad1.a;

                break;
            case OUTTAKE_WRIST_UP:
                outtake.outtaking = true;

                // TURRET CONTROL
                if(outtakeTimer.seconds() > OUTTAKE_DELAY_SECONDS) {
                    if(!outtake.wristRetracted) {
                        if (gamepad1.left_trigger > 0.1) outtake.setTurretAngle(-gamepad1.left_trigger * 60 + 270);
                        else if (gamepad1.right_trigger > 0.1) outtake.setTurretAngle(gamepad1.right_trigger * 60 + 270);
                        else outtake.setTurretAngle(270);
                    } else outtake.setTurretAngle(270);
                } else outtake.setTurretAngle(270);

                // retract wrist
                if(outtake.turret.isCentered() && gamepad1.dpad_down && !lastDown1) {
                    outtake.retractWrist();
                    robotState = RobotState.OUTTAKE_WRIST_RETRACT;
                }
                lastDown1 = gamepad1.dpad_down;

                // Change height
                if(gamepad1.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad1.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad1.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);


// reverse intake
                if(gamepad1.right_trigger > 0.3 && gamepad1.left_trigger > 0.3) intake.setIntakePower(-(gamepad1.right_trigger + gamepad1.left_trigger)/2);
                else intake.setIntakePower(0);

// lock/unlock for depositing
                if(gamepad1.left_bumper) outtake.unlockFrontLockBack();
                else if(gamepad1.right_bumper) outtake.unlock();
                else outtake.lock();

                // retract
                if(gamepad1.a && !lastA1 && outtake.turret.isCentered()) {
                    retractRequested = true;
                    retractTimer.reset();
                    outtake.retractWrist();
                    robotState = RobotState.OUTTAKE_WRIST_RETRACT;
                }
                lastA1 = gamepad1.a;
                break;
            case OUTTAKE_WRIST_RETRACT:
                // retract
                if((gamepad1.a && !lastA1) || (retractRequested && retractTimer.seconds() > 0.35)) {
                    robotState = RobotState.RETRACT;
                    retractRequested = false;
                    outtake.outtaking = false;
                    outtake.lift.setMotionProfileTargetPos(0);
                }

                // extend wrist
                if(gamepad1.dpad_down && !lastDown1) {
                    retractRequested = false;
                    outtake.extendWrist();
                    robotState = RobotState.OUTTAKE_WRIST_UP;
                    outtakeTimer.reset();
                }
                lastDown1 = gamepad1.dpad_down;

                // Change height
                if(gamepad1.x) {
                    retractRequested = false;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad1.y) {
                    retractRequested = false;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad1.b) {
                    retractRequested = false;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }
                break;
        }


// DRIVING

        drivetrain.setDrivePower(robotState, gamepad1);

        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rotate = -gamepad1.right_stick_x;

        if (gamepad1.left_stick_button) drivetrain.driveToHeading(horz, vert, boardHeading);
        else drivetrain.driveMaintainHeading(horz, vert, rotate);

        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(boardHeading);
            gamepad1.rumble(100);
        }
    }

    public void initialize() {
        robot = new Robot(hardwareMap);
        drivetrain = robot.addDrivetrain(true);
        intake = robot.addIntake();
        outtake = robot.addOuttake();
        hanger = robot.addHanger();
        plane = robot.addPlane();

        alliance = Initialization.ALLIANCE;
        drivetrain.setPoseEstimate(Initialization.POSE);

        robotState = RobotState.RETRACT;
        totalTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();
        retractTimer = new ElapsedTime();
        boardHeading = alliance == Alliance.RED ? Math.toRadians(180) : 0;
        lastTime = System.currentTimeMillis();
        deltaTime = 0;
    }

    public void write() {
        robot.write();

        deltaTime = totalTimer.milliseconds() - lastTime;
        lastTime = totalTimer.milliseconds();

        // write telemetry only some loops to reduce loop times
        loop++;
        if(loop > 50) {
            telemetry.addData("robot state", robotState);
            robot.telemetry(telemetry);
            telemetry.addData("loop time", deltaTime);
            telemetry.addData("alliance", alliance);
            telemetry.update();
            loop = 0; // reset loop counter
        }
    }

    // seconds
    public double timeSince(double time) {
        return totalTimer.seconds() - time;
    }
}
