package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="IntakePowerOneMotorWithArm", group="Robot")
public class IntakePowerOneMotor extends LinearOpMode {

    private DcMotor intake;
    private Servo armservo;
    private DcMotor bootkicker;



    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        armservo = hardwareMap.get(Servo.class, "armservo");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");
        //double curPos = 0.5;
        waitForStart();
        //distance sensor

        while (opModeIsActive()) {
          //  curPos = armservo.getPosition();
//            if(gamepad1.dpad_down) {
//                curPos-=0.0001;
//                curPos = Math.max(curPos, 0);
//                armservo.setPosition(curPos);
//            }
//            else if(gamepad1.dpad_up) {
//                curPos+=0.0001;
//                curPos = Math.min(curPos, 1);
//                armservo.setPosition(curPos);
//            }
            if (gamepad1.dpad_right) {
                armservo.setPosition(0.0475);
            }
            if(gamepad1.dpad_left) {
                armservo.setPosition(0.042);
            }
            if(gamepad1.dpad_down) {
                armservo.setPosition(0.04);
            }
            if (gamepad1.dpad_up) {
                armservo.setPosition(0.043);
            }
            if (gamepad1.right_bumper) {
                armservo.setPosition(0.041);
            }

            if (gamepad1.a) {
                bootkicker.setPower(-0.5);
            }

            if (gamepad1.x) {
                bootkicker.setPower(-0.3);
            }
            if (gamepad1.b) {
                intake.setPower(-0.5);
            }
            if (gamepad1.y){
                intake.setPower(-0.9);
            }



//            if (gamepad1.dpad_up) {
//                armservo.setPosition(0.6);
//            }
//            if(gamepad1.dpad_down) {
//                armservo.setPosition(0.4);
//            }




            telemetry.addData("armservo Pos", armservo.getPosition());
            telemetry.addData("intake Power", intake.getPower());
            telemetry.addData("Bootkicker power", bootkicker.getPower());
            telemetry.update();
        }

    }}




