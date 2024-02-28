package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TurnMotionProfileTest extends LinearOpMode {
    Robot robot;
    Drivetrain drivetrain;
    IntakeWrist intakeWrist;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drivetrain = robot.addDrivetrain(true);
        intakeWrist = robot.addIntakeWrist();
        robot.init();
        waitForStart();
        while(opModeIsActive()) {
            robot.read();

            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rot = -gamepad1.right_stick_x;

            drivetrain.drive(horz, vert, rot);

            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading(90);
                gamepad1.rumble(150);
            }

            robot.write();
            robot.telemetry(telemetry);
            telemetry.update();
        }
    }
}
