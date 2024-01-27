package org.firstinspires.ftc.teamcode.blucru.common.util;

public class GHFilter {
    double G;
    double H;
    double xEst;
    double xPredict;
    double dX;

    double lastTime;

    public GHFilter(double g, double h, double initValue) {
        xEst = initValue;
        G = g;
        H = h;
        dX = 0;
    }

    public double predict(double z, double dt) {
        xPredict = xEst + dX * dt;
        double residual = z - xPredict;

        dX = dX + H * residual / dt;
        xEst = xPredict + G * residual;

        return xEst;
    }

    public void update(double measurement) {
        double dt = System.currentTimeMillis() - lastTime;
        lastTime = System.currentTimeMillis();
        predict(measurement, dt);
    }
}
