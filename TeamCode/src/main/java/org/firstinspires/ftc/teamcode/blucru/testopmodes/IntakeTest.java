package org.firstinspires.ftc.teamcode.blucru.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.Constants;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Intake;

@TeleOp(name = "intake test", group = "TeleOp")
public class IntakeTest extends LinearOpMode {
    Intake intake;
    Drivetrain drivetrain;
    double horz, vert, rotate;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        intake.init();
        drivetrain.init();
        waitForStart();
        while(opModeIsActive()) {
            horz = gamepad1.left_stick_x;
            vert = -gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;

            if(gamepad1.left_trigger > 0.1) {
                intake.setOuttakeRollersPower(gamepad1.left_trigger);
                intake.stopOuttakeWrist();
            } else if (gamepad1.right_trigger > 0.1) {
                intake.setOuttakeRollersPower(-gamepad1.right_trigger);
                intake.stopOuttakeWrist();
            } else {
                intake.setOuttakeRollersPower(0);
                intake.setOuttakeWristPosition(Constants.outtakeWristRetractPos);
            }

            intake.write();
            drivetrain.drive(horz, vert, rotate);

            telemetry.addData("left trigger", gamepad1.left_trigger);
            telemetry.addData("intake rollers power", intake.getIntakeRollersPower());
            telemetry.update();
        }
    }
}
