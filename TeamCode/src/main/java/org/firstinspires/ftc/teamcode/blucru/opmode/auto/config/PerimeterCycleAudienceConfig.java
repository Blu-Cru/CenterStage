package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class PerimeterCycleAudienceConfig extends AutoConfig {
    enum State {

    }

    Path currentPath;
    StateMachine stateMachine;
    ElapsedTime runtime;
    Robot robot;

    public void build() {

    }

    public PerimeterCycleAudienceConfig() {

    }

    public void run() {

    }

    public void start(Randomization randomization) {

    }

    public void telemetry(Telemetry tele) {
        tele.addLine("Audience Perimeter Cycle");
        tele.addData("State", stateMachine.getState());
    }

    public void logTransitionTo(Enum to) {
        Log.i("PerimeterCycleAudienceConfig", "Transitioning to " + to);
    }
}
