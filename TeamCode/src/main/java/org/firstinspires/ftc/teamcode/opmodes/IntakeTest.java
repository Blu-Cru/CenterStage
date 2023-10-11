//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.Hardware6417;
//
//
///*
//Controls:
//
//release everything: retracted state
//a : release cone/intake
//b : low height
//x : medium height
//y : high height
//right trigger (hold) : turret to the right, then wrist down
//left trigger (hold) : turret to the left, then wrist down
//left joystick : strafe (field centric)
//right joystick : turn
//options : cone righting + snail drive
//
// */
//@TeleOp(name = "IntakeTest", group = "TeleOp")
//public class IntakeTest extends LinearOpMode {
//    public DcMotorEx left, right;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        left = hardwareMap.get(DcMotorEx.class, "left");
//        right = hardwareMap.get(DcMotorEx.class, "right");
//        waitForStart();
//        while (opModeIsActive()) {
//            if(gamepad1.a) {
//                left.setPower(0);
//                right.setPower(0);
//            }
//            if(gamepad1.b) {
//                left.setPower(0.5);
//                right.setPower(-0.5);
//            }
//            if(gamepad1.x) {
//                left.setPower(-0.5);
//                right.setPower(0.5);
//            }
//        }
//    }
//}
