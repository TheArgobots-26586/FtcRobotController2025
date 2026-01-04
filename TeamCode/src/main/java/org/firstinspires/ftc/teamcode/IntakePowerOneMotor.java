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

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        armservo = hardwareMap.get(Servo.class, "armservo");

        waitForStart();
        //distance sensor

        while (opModeIsActive()) {


            if (gamepad1.a) {
                intake.setPower(-0.5);
            }

            if (gamepad1.x) {
                intake.setPower(0.5);
            }


            if (gamepad1.dpad_down) {
                armservo.setPosition(0.1);
            }
            if (gamepad1.dpad_up) {
                armservo.setPosition(0.2);
            }
            if (gamepad1.dpad_right) {
                armservo.setPosition(0.7);
            }
            if (gamepad1.dpad_left) {
                armservo.setPosition(0.9);
            }


        }
            telemetry.addData("armservo Pos", armservo.getPosition());
            telemetry.addData("intake Power", intake.getPower());
            // telemetry.addData("armservo Pos", armservo.get());
            telemetry.update();
        }

    }




