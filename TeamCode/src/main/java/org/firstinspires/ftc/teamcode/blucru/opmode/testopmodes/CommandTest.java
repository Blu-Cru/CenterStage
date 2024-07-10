package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.OuttakeWristBackstageCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.TurretGlobalYCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.opmode.KLinearOpMode;

//@Disabled
@Config
@TeleOp(name = "Command test", group = "test")
public class CommandTest extends KLinearOpMode {
    public static double intakeTime = 150;
    boolean lastA = false,
            lastB = false,
            lastY = false,
            lastX = false;
    @Override
    public void initialize() {
        addOuttake();
        addIntake();
        addDrivetrain(true);
        drivetrain.fieldCentric = false;
    }

    public void periodic() {

        drivetrain.teleOpDrive(gamepad1);

        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(Math.toRadians(90));
        }

        if(gamepad1.b && !lastB) {
            CommandScheduler.getInstance().schedule(new IntakeStopCommand());
        }
        lastB = gamepad1.b;

        if(gamepad1.x && !lastX) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new OuttakeExtendCommand(1),
                            new TurretGlobalYCommand(0)
                    )
            );
        }
        lastX = gamepad1.x;

        if(gamepad1.y && !lastY) {
            CommandScheduler.getInstance().schedule(new OuttakeRetractCommand());
        }
        lastY = gamepad1.y;

        if(stickyG1.dpad_up) {
            new SequentialCommandGroup(
                    new LockCommand(),
                    new InstantCommand(
                            () -> Robot.getInstance().outtake.setTargetPixelHeight(-1)
                    ),
                    new WaitCommand(150),
                    new OuttakeWristBackstageCommand()
            ).schedule();
        }

        if(stickyG1.dpad_down) {
            new SequentialCommandGroup(
                    new LockReleaseCommand(2),
                    new WaitCommand(150),
                    new OuttakeRetractCommand()
            ).schedule();
        }
    }
}
