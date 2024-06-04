package org.firstinspires.ftc.teamcode.blucru.opmode.tele;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

public class MTITele extends BCLinearOpMode {
    enum State {
        INTAKING,
        RETRACT,
        OUTTAKING,
        RETRACTING
    }

    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.RETRACT)
            .transition(() -> !intake.isFull() && outtake.liftIntakeReady() && stickyG2.right_bumper, State.INTAKING, () -> {
                new IntakeCommand(0).schedule();
            })
            .transition(() -> !intake.isFull() && outtake.liftIntakeReady() && stickyG2.a, State.INTAKING, () -> {
                new IntakeCommand(2).schedule();
            })
            .transition(() -> stickyG2.b, State.OUTTAKING, () -> {
                new OuttakeExtendCommand(Outtake.HIGH_HEIGHT).schedule();
            })
            .transition(() -> stickyG2.y, State.OUTTAKING, () -> {
                new OuttakeExtendCommand(Outtake.MID_HEIGHT).schedule();
            })
            .transition(() -> stickyG2.x, State.OUTTAKING, () -> {
                new OuttakeExtendCommand(Outtake.LOW_HEIGHT).schedule();
            })
            .build();
}
