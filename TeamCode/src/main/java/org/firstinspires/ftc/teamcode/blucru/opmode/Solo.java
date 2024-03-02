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
    Robot robot;
    Drivetrain drivetrain;
    Intake intake;
    Outtake outtake;
    Hanger hanger;
    Plane plane;
    Alliance alliance;
    RobotState robotState;
    ElapsedTime totalTimer;
    double boardHeading, lastTime, deltaTime, loop;

    // timer variables
    double stopIntakeTimeSeconds = 0;

    // gamepad variables
    double lastLT = 0;

    public void runOpMode()  throws InterruptedException {
        initialize();

        while (opModeIsActive()) {
            read();
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
                    outtake.locks.unlockAll();
                } else if(gamepad1.right_trigger > 0.3) {
                    intake.setIntakePower(-gamepad1.right_trigger);
                    outtake.locks.lockAll();
                } else if(timeSince(stopIntakeTimeSeconds) < 0.5) {
                    intake.setIntakePower(-1);
                    outtake.locks.lockAll();
                } else {
                    intake.setIntakePower(0);
                    outtake.locks.lockAll();
                }

                // if LT was released, start timer
                if(lastLT > 0.3 && !(gamepad1.left_trigger > 0.3))  stopIntakeTimeSeconds = totalTimer.seconds();
                lastLT = gamepad1.left_trigger;

                break;
            case LIFTING:
                outtake.outtaking = true;

// reverse intake
                if(gamepad1.right_trigger > 0.3 && gamepad1.left_trigger > 0.3) intake.setIntakePower(-(gamepad1.right_trigger + gamepad1.left_trigger)/2);

                break;
            case OUTTAKE:
                outtake.outtaking = true;
// reverse intake
                if(gamepad1.right_trigger > 0.3 && gamepad1.left_trigger > 0.3) intake.setIntakePower(-(gamepad1.right_trigger + gamepad1.left_trigger)/2);
// lock/unlock for depositing
                if(gamepad1.left_bumper) outtake.locks.unlockFrontLockBack();
                else if(gamepad1.right_bumper) outtake.locks.unlockAll();
                else outtake.locks.lockAll();

                break;
        }

// DRIVING
        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rotate = -gamepad1.right_stick_x;

        if(gamepad1.left_stick_button) {
            drivetrain.resetHeading(boardHeading);
            gamepad1.rumble(100);
        }
        if (gamepad1.right_stick_button) drivetrain.driveToHeading(horz, vert, boardHeading);
        else drivetrain.drive(horz, vert, rotate);
    }

    public void initialize() {
        robot = new Robot(hardwareMap);
        drivetrain = robot.addDrivetrain(true);
        intake = robot.addIntake();
        outtake = robot.addOuttake();
        hanger = robot.addHanger();
        plane = robot.addPlane();
        alliance = Initialization.alliance;
        robotState = RobotState.RETRACT;
        totalTimer = new ElapsedTime();
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
