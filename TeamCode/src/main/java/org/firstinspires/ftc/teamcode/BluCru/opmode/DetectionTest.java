package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@TeleOp(name = "Detection Test", group = "Teleop")
public class DetectionTest extends LinearOpMode {
    DistanceSensor zero;
    DistanceSensor one;
    double sum, avg, loopTime;
    double pastTime = 0;
    ElapsedTime time;
    @Override
    public void runOpMode() throws InterruptedException {
        zero = hardwareMap.get(DistanceSensor.class, "zero");
        one = hardwareMap.get(DistanceSensor.class, "one");
        time = new ElapsedTime();
        ArrayList<Double> distances = new ArrayList<Double>();
        telemetry.addData("Status", "Initialized");
        while(opModeInInit()) {
            loopTime = time.seconds() - pastTime;
            pastTime = time.seconds();

            if(distances.size() >= 10) {
                distances.remove(0);
                distances.add(zero.getDistance(DistanceUnit.INCH));
            }

            telemetry.addData("Distances", distances);
            telemetry.update();
        }
        waitForStart();

        for(int i = 0; i < distances.size(); i++) {
            sum += distances.get(i);
        }

        avg = sum/distances.size();

        while (opModeIsActive()) {
            telemetry.addData("zero: ", zero.getDistance(DistanceUnit.INCH));
            telemetry.addData("one: ", one.getDistance(DistanceUnit.INCH));
            telemetry.addData("avg: ", avg);
            telemetry.addData("sum: ", sum);
            telemetry.addData("size: ", distances.size());


            telemetry.update();
        }
    }
}
