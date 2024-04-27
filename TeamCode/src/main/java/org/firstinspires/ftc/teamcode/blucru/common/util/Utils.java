package org.firstinspires.ftc.teamcode.blucru.common.util;

public final class Utils {
    public static double headingClip(double value) {
        while(value >= Math.PI) {
            value -= 2*Math.PI;
        }
        while(value <= -Math.PI) {
            value += 2*Math.PI;
        }
        return value;
    }
}
