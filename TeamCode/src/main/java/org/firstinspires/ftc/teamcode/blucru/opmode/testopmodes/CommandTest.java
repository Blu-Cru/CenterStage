package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@Disabled
@TeleOp(name = "Command test", group = "test")
public class CommandTest extends BCLinearOpMode {
    boolean lastA = false,
            lastB = false,
            lastY = false,
            lastX = false;
    @Override
    public void initialize() {
        addOuttake();
        addIntake();
    }

    public void periodic() {
        if(gamepad1.a && !lastA) {
            CommandScheduler.getInstance().schedule(new IntakeCommand(0, 1));
        }
        lastA = gamepad1.a;

        if(gamepad1.b && !lastB) {
            CommandScheduler.getInstance().schedule(new StopIntakeCommand());
        }
        lastB = gamepad1.b;

        if(gamepad1.x && !lastX) {
            CommandScheduler.getInstance().schedule(new OuttakeExtendCommand(1));
        }
        lastX = gamepad1.x;

        if(gamepad1.y && !lastY) {
            CommandScheduler.getInstance().schedule(new OuttakeRetractCommand());
        }
        lastY = gamepad1.y;

        CommandScheduler.getInstance().run();
    }
}
