package org.firstinspires.ftc.teamcode.BluCru;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Hardware6417 {
    public DcMotorEx slider, auxSlider, frontLeft, frontRight, backLeft, backRight;
    public Servo turret, wrist, twister, grabber, leftRetract, rightRetract;

    public Hardware6417(HardwareMap hwMap) {
        initSlides(hwMap);
        initIntake(hwMap);
        initRetract(hwMap);
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
        turret      = hwMap.get(Servo.class, "Turret");
        grabber     = hwMap.get(Servo.class, "Grabber");
        twister     = hwMap.get(Servo.class, "Twister");
        wrist       = hwMap.get(Servo.class, "Wrist");
    }

    public void initRetract(HardwareMap hwMap) {
        leftRetract = hwMap.get(Servo.class, "LeftRetract");
        rightRetract = hwMap.get(Servo.class, "RightRetract");
    }
    public void autoTurret(double position) {
        if(turret.getPosition() != position) {
            turret.setPosition(position);
        }
    }

    public void autoTwister(double position) {
        if(twister.getPosition() != position) {
            twister.setPosition(position);
        }
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

    public boolean turretClear() {return slider.getCurrentPosition() > Constants.sliderTurretClearPos;}

    public void closeGrabber() {grabber.setPosition(Constants.grabberClosePos);}
    public void openGrabber() {grabber.setPosition(Constants.grabberOpenPos);}

    public void retractOdo() {
        leftRetract.setPosition(Constants.leftOdoRetractPos);
        rightRetract.setPosition(Constants.rightOdoRetractPos);
    }
    public void dropOdo() {
        leftRetract.setPosition(Constants.leftOdoDropPos);
        rightRetract.setPosition(Constants.rightOdoDropPos);
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