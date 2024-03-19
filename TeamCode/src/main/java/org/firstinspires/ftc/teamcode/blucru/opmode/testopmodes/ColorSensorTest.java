package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

@TeleOp(name = "color sensor test", group = "test")
public class ColorSensorTest extends BCLinearOpMode {
    boolean lastA = false;

    View relativeLayout;
    public void periodic() {
        if(gamepad1.a && !lastA) {
            intakeColorSensors.toggleReading();
        }
        lastA = gamepad1.a;

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(intakeColorSensors.getFrontHSV()));
            }
        });
    }

    public void initialize() {
        addIntakeColorSensors();
        intakeColorSensors.startReading();

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    }

    public void end() {
        relativeLayout.setBackgroundColor(Color.CYAN);
    }
}
