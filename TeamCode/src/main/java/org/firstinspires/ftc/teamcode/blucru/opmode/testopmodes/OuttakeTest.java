package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "outtake test", group = "test")
public class OuttakeTest extends BluLinearOpMode {
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
