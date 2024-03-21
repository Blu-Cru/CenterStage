package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;

public class IntakeTrajectories {
    public static double
            DROP_INTAKE_TIME = -1.0,
            START_INTAKE_TIME = -0.5,
            INTAKE_LENGTH = 0.0,
            INTAKE_HYPOTENUSE = INTAKE_LENGTH * INTAKE_LENGTH;

    double reflect;
    public IntakeTrajectories(double reflect) {
        reflect = reflect;
    }
}
