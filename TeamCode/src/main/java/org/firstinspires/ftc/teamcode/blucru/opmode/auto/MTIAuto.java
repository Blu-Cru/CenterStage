package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

public class MTIAuto extends BCLinearOpMode {
    private enum State {
        CONFIG,
        BUILD,
        DETECTION,
        RUNNING,
        END
    }
    AutoConfig config;
    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.CONFIG)
            .loop(() -> {
                if(stickyG1.a) Globals.alliance = Globals.alliance.flip();
                if(stickyG1.b) Globals.side = Globals.side.flip();
                if(stickyG1.x) Globals.autoType = Globals.autoType.cycle();
                if(stickyG1.y) Globals.parkType = Globals.parkType.cycle();
            })
            .transition(() -> stickyG1.right_stick_button, State.DETECTION, () -> {
                telemetry.addLine("Building Paths . . .");
                telemetry.update();
            })
            .build();

    @Override
    public void initialize() {
        addDrivetrain(false);
        addIntake();
        addOuttake();
        addCVMaster();
        addPurplePixelHolder();
    }

    @Override
    public void initLoop() {

    }

    public void buildConfig() {

    }
}
