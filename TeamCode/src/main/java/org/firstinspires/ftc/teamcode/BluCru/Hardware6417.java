package org.firstinspires.ftc.teamcode.BluCru;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Vector;

public class Hardware6417 {
    public DcMotorEx slider, auxSlider;
    public Servo wrist;
    public CRServo intake;
    public MecanumDrive drive;

    public Hardware6417(HardwareMap hwMap) {
        initSlides(hwMap);
        initIntake(hwMap);
    }

    public void initSlides(HardwareMap hwMap) {
        slider  = hwMap.get(DcMotorEx.class, "Slider");
        auxSlider = hwMap.get(DcMotorEx.class, "AuxSlider");

        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        auxSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        //set all motors to zero power
        slider.setPower(0);
        auxSlider.setPower(0);

        slider.setTargetPosition(0);
        auxSlider.setTargetPosition(0);

        //set brake behavior
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        auxSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        auxSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initIntake(HardwareMap hwMap) {
        wrist       = hwMap.get(Servo.class, "Wrist");
    }

    public void initDrive(HardwareMap hwMap, Pose2d pose) {
        drive = new MecanumDrive(hwMap, pose);
    }

    public void autoWrist(double position) {
        if(wrist.getPosition() != position) {
            wrist.setPosition(position);
        }
    }

    public boolean slideOuttakeReady() {
        return Math.abs(slider.getTargetPosition() - slider.getCurrentPosition()) < Constants.sliderOuttakeDelta;
    }

    public void autoSlide(int position, double power) {
        if(slider.getCurrentPosition() != position) {
            slider.setTargetPosition(position);
            auxSlider.setTargetPosition(position);

            if(slider.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                auxSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            slider.setPower(power);
            auxSlider.setPower(power);
        }
    }

    public void resetSliders() {
        slider.setPower(0);
        auxSlider.setPower(0);

        slider.setTargetPosition(0);
        auxSlider.setTargetPosition(0);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        auxSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean sliderIntakeReady() {
        return slider.getCurrentPosition() < Constants.sliderIntakeDelta;
    }

    public void holonomicDrive(double vert, double horz, double rotate, double driveSpeed, double heading) {
        Vector2d input = new Vector2d(horz, vert);
        drive.setDrivePowers(new PoseVelocity2d(input, rotate*driveSpeed));
    }

    /*public void holonomicDrive(double vert, double horz, double rotate, double driveSpeed, double heading) {
        Vector2d input = new Vector2d(horz, vert).rotated(-heading - Math.toRadians(90));
        setWeightedDrivePower(new Pose2d(input.getX() * driveSpeed, input.getY() * driveSpeed, rotate * driveSpeed));
    }*/

    public void telemetry(Telemetry tele) {
        tele.addData("slider position: ", slider.getCurrentPosition());
        tele.addData("slider target pos: ", slider.getTargetPosition());
    }
}