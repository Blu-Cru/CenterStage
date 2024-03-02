package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.arcrobotics.ftclib.controller.PIDController;

public class PDController extends PIDController {
    public PDController(double kP, double kD) {
        super(kP, 0, kD);
    }

    public double calculate(double currentPos, double targetPos, double targetVelocity) {
        return calculate(currentPos, targetPos) + targetVelocity * getD();
    }
}
