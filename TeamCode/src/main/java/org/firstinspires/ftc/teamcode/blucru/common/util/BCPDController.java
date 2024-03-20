package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.arcrobotics.ftclib.controller.PIDController;

public class BCPDController extends PIDController {
    public BCPDController(double kP, double kD) {
        super(kP, 0, kD);
    }

    public double calculate(double currentPos, double targetPos, double currentVelocity, double targetVelocity) {
        return getP() * (targetPos - currentPos) + getD() * (targetVelocity - currentVelocity);
    }
}
