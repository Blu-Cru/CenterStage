package org.firstinspires.ftc.teamcode.blucru.opmode;

import com.acmerobotics.dashboard.config.Config;
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
    public static double OUTTAKE_DELAY_SECONDS = 0.3;

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
    ElapsedTime retractTimer;
    double lastTime, deltaTime;
    int loop;
    double boardHeading;

    boolean retractRequested = false;

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
            // updates states based on gamepad input
            read();
            // writes to hardware
            write();
        }
    }

    public void initialize() {
        robotState = RobotState.RETRACT;
        robot = new Robot(hardwareMap); // initialize robot

        // add subsystems
        drivetrain = robot.addDrivetrain(true);
        outtake = robot.addOuttake();
        intake = robot.addIntake();
        hanger = robot.addHanger();
        plane = robot.addPlane();

        totalTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();
        retractTimer = new ElapsedTime();

        robot.init();

        alliance = Initialization.alliance; // set alliance based on auto
        boardHeading = alliance == Alliance.RED ? Math.toRadians(180) : 0; // set board heading based on alliance
    }

    public void read() {
        robot.read();

        // DRIVING
        drivetrain.setDrivePower(robotState, gamepad1);

        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rotate = -gamepad1.right_stick_x;

        // resets heading offset (face away from board)
        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(boardHeading);
            gamepad1.rumble(100);
        }
        if(gamepad1.b) drivetrain.driveToHeading(horz, vert, boardHeading); // drive to outtake heading
        else if(gamepad1.x) drivetrain.driveToHeading(horz, vert, boardHeading - Math.PI); // drive to opposite outtake heading
        else drivetrain.drive(horz, vert, rotate); // drive normally

        // INTAKE
        if(gamepad2.left_bumper && outtake.liftIntakeReady()) {
            intake.setIntakePower(1);
            outtake.unlock();
        } else if(gamepad2.right_bumper) {
            intake.setIntakePower(-1);
            outtake.lock();
        } else {
            intake.setIntakePower(0);
            if (gamepad2.dpad_left) outtake.unlockFrontLockBack();
            else if(gamepad2.dpad_right) outtake.unlock();
            else outtake.lock();
        }

        // toggle intake wrist
        if(gamepad2.a && outtake.liftIntakeReady()) intake.dropToGround();
        else if (gamepad2.dpad_down && outtake.liftIntakeReady()) intake.dropToStack(2);
        else intake.retractIntakeWrist();

        switch(robotState) {
            case RETRACT:
                outtake.outtaking = false;

                if(gamepad2.x) {
                    outtake.outtaking = true;
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad2.y) {
                    outtake.outtaking = true;
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad2.b) {
                    outtake.outtaking = true;
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }
                break;
            case LIFTING:
                if(outtake.lift.getCurrentPos() > Outtake.LIFT_WRIST_CLEAR_POS) {
                    outtake.wristRetracted = false;
                    robotState = RobotState.OUTTAKE_WRIST_UP;
                    outtakeTimer.reset();
                }

                if(gamepad2.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad2.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad2.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

                if(gamepad2.a && !lastA2) {
                    robotState = RobotState.RETRACT;
                    retractRequested = false;
                    outtake.outtaking = false;
                    outtake.lift.setMotionProfileTargetPos(0);
                }
                lastA2 = gamepad2.a;
                break;
            case OUTTAKE_WRIST_UP:
                // TURRET CONTROL
                if(outtakeTimer.seconds() > OUTTAKE_DELAY_SECONDS) {
                    if(!outtake.wristRetracted) {
                        if (gamepad2.left_trigger > 0.1) outtake.setTurretAngle(-gamepad2.left_trigger * 60 + 270);
                        else if (gamepad2.right_trigger > 0.1) outtake.setTurretAngle(gamepad2.right_trigger * 60 + 270);
                        else outtake.setTurretAngle(270);
                    } else outtake.setTurretAngle(270);
                } else outtake.setTurretAngle(270);

                // retract wrist
                if(outtake.turret.isCentered() && gamepad2.dpad_down && !lastDown2) {
                    outtake.retractWrist();
                    robotState = RobotState.OUTTAKE_WRIST_RETRACT;
                }
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
                if(gamepad2.a && !lastA2 && outtake.turret.isCentered()) {
                    retractRequested = true;
                    retractTimer.reset();
                    outtake.retractWrist();
                    robotState = RobotState.OUTTAKE_WRIST_RETRACT;
//                    outtake.outtaking = false;
//                    outtake.lift.setMotionProfileTargetPos(0);
                }
                lastA2 = gamepad2.a;
                break;
            case OUTTAKE_WRIST_RETRACT:
                // retract
                if((gamepad2.a && !lastA2) || (retractRequested && retractTimer.seconds() > 0.5)) {
                    robotState = RobotState.RETRACT;
                    retractRequested = false;
                    outtake.outtaking = false;
                    outtake.lift.setMotionProfileTargetPos(0);
                }

                // extend wrist
                if(gamepad2.dpad_down && !lastDown2) {
                    outtake.extendWrist();
                    robotState = RobotState.OUTTAKE_WRIST_UP;
                    outtakeTimer.reset();
                }
                lastDown2 = gamepad2.dpad_down;

                // Change height
                if(gamepad2.x) {
                    retractRequested = false;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad2.y) {
                    retractRequested = false;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad2.b) {
                    retractRequested = false;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }

                // increment height by one pixel
                if(gamepad2.right_stick_y < -0.3 && !lastRSUp2 && !gamepad2.right_stick_button) {
                    retractRequested = false;
                    outtake.incrementTargetHeight(1);
                }
                lastRSUp2 = gamepad2.right_stick_y < -0.3;
                if(gamepad2.right_stick_y > 0.3 && !lastRSDown2 && !gamepad2.right_stick_button) {
                    retractRequested = false;
                    outtake.incrementTargetHeight(-1);
                }
                lastRSDown2 = gamepad2.right_stick_y > 0.3;
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
        if(Math.abs(gamepad2.left_stick_y) > 0.2)
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

        deltaTime = totalTimer.milliseconds() - lastTime;
        lastTime = totalTimer.milliseconds();

        loop++;
        if(loop > 10) {
            telemetry.addData("robot state", robotState);
            robot.telemetry(telemetry);
            telemetry.addData("loop time", deltaTime);
            telemetry.addData("alliance", alliance);
            telemetry.update();
            loop = 0;
        }
    }
}
