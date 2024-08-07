package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Turret ivk test", group = "test")
public class TurretIVKTest extends BluLinearOpMode {
    public static double TARGET_Y = 36;
    boolean retracted;
    @Override
    public void initialize() {
        addDrivetrain(true);
        addOuttake();
        addCVMaster();
    }

    @Override
    public void onStart() {
        cvMaster.detectTag();
    }

    @Override
    public void periodic() {
        double vert = -gamepad1.left_stick_y;
        double horz = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(Math.toRadians(90));
            gamepad1.rumble(150);
        }
        if(stickyG1.b) {
            retracted = false;
            CommandScheduler.getInstance().schedule(new OuttakeExtendCommand(1));
        }
        if(stickyG1.a) {
            retracted = true;
            CommandScheduler.getInstance().schedule(new OuttakeRetractCommand());
        }

        if(!retracted) outtake.setTurretGlobalY(TARGET_Y);

        drivetrain.updateAprilTags(cvMaster.tagDetector);

        drivetrain.driveScaled(horz, vert, rotate);
    }
}
