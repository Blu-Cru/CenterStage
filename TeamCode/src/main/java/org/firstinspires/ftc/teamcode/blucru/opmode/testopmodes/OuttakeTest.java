package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@Config
@TeleOp(name = "outtake test", group = "test")
public class OuttakeTest extends BCLinearOpMode {
    @Override
    public void periodic() {
        if(stickyG1.b) {
            CommandScheduler.getInstance().schedule(new OuttakeExtendCommand(0));
        }

        if(stickyG1.a) {
            CommandScheduler.getInstance().schedule(new OuttakeRetractCommand());
        }

        outtake.setDunkHeight(gamepad1.left_trigger);
    }

    public void initialize() {
        addOuttake();
        enableFTCDashboard();
    }
}
