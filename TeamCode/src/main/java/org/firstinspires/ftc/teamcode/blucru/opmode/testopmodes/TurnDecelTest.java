package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

/*
This is a test op mode to calculate the deceleration of the robot when turning
The robot's turning is under driver control until the driver presses a to calculate the deceleration

There are two theories I want to test:
1. The robot's deceleration is constant
2. The robot's change in angle is proportional to the robot's kinetic energy

To test these theories, I will calculate the robot's deceleration by measuring the robot's heading and heading velocity
at the start of the deceleration and at the end of the deceleration. I then calculate the theoretical deceleration and
theoretical coefficient to angular velocity squared. Doing multiple tests at different velocities will hopefully reveal
which theory is correct.
*/

@Disabled
@TeleOp(name = "Turn Decel Test", group = "test")
public class TurnDecelTest extends LinearOpMode {
    Robot robot;
    Drivetrain drivetrain;
    Intake intake;
    ElapsedTime runtime;

    private enum State {
        TURNING,
        DECEL,
        STOP
    }
    State state;
    double startDecelHeading = 0;
    double startDecelVelocity = 0;
    double startDecelTime = 0;
    double endHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = Robot.getInstance();
        drivetrain = robot.addDrivetrain(true);
        intake = robot.addIntake();
        state = State.TURNING;
        drivetrain.setDrivePower(1);

        robot.init();

        waitForStart();
        runtime = new ElapsedTime(); // start timer
        while(opModeIsActive()) {
            robot.read();
            telemetry.addData("STATE:", state);
            switch (state) {
                case TURNING:
                    double horz = gamepad1.left_stick_x;
                    double vert = -gamepad1.left_stick_y;
                    double rotate = -gamepad1.right_stick_x;
                    drivetrain.driveScaled(horz, vert, rotate);

                    // press dpad up to calculate deceleration
                    if (gamepad1.dpad_up) {
                        state = State.DECEL; // change op mode state
                        drivetrain.driveScaled(0, 0, 0); // stop the robot
                        startDecelHeading = drivetrain.getHeading(); // get the current heading
                        startDecelVelocity = drivetrain.getPoseVelocity().getHeading(); // get the current heading velocity
                        startDecelTime = runtime.seconds(); // get the current time
                        gamepad1.rumble(100); // rumble the controller to indicate the start of deceleration
                    }

                    if(gamepad1.right_stick_button) {
                        drivetrain.resetHeading(90);
                        gamepad1.rumble(200);
                    }

                    telemetry.addLine("Use right stick to turn robot");
                    telemetry.addLine("Press a to calculate deceleration");
                    telemetry.addData("heading:", drivetrain.getHeading());
                    break;
                case DECEL:
                    if(drivetrain.isStopped() && timeSinceSecs(startDecelTime) > 1.5) {
                        state = State.STOP;
                        endHeading = drivetrain.getHeading();
                        gamepad1.rumble(100); // rumble the controller to indicate the end of deceleration
                    }
                    telemetry.addLine("Robot is decelerating");
                    break;
                case STOP:
                    drivetrain.driveScaled(0, 0, 0); // stop the robot

                    if(gamepad1.b) {
                        state = State.TURNING; // reset the op mode
                        gamepad1.rumble(100); // rumble the controller to indicate the reset
                    }

                    telemetry.addLine("Robot has stopped");
                    telemetry.addData("Start Heading", startDecelHeading);
                    telemetry.addData("End Heading", endHeading);
                    telemetry.addData("Start Velocity", startDecelVelocity);
                    telemetry.addData("OBSERVED DECELERATION:", getAngularDecel());
                    telemetry.addLine("Press b to reset");
                    break;
            }
            robot.write();
            telemetry.update();
        }
    }

    public double timeSinceSecs(double time) {
        return runtime.seconds() - time;
    }

    // this method gets the theoretical angular deceleration of the robot, assuming the robot's deceleration is constant
    public double getAngularDecel() {
        // vf^2 = vi^2 + 2a(xf - xi)
        // vf = 0, xi = startDecelHeading, xf = endHeading, vi = startDecelVelocity
        // 0 = startDecelVelocity^2 + 2a(endHeading - startDecelHeading)
        // a = -startDecelVelocity^2 / (2(endHeading - startDecelHeading))
        return -startDecelVelocity * startDecelVelocity / (2 * (endHeading - startDecelHeading));
    }
}
