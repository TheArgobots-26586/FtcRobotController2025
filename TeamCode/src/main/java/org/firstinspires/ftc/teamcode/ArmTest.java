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


@TeleOp(name="ArmTest", group="Robot")
public class ArmTest extends LinearOpMode {

    private Servo armservo;



    @Override
    public void runOpMode() {

        armservo = hardwareMap.get(Servo.class, "armservo");


        // --- Initialize intake/shooter ---


        // --- Initialize Limelight / rotator ---


        //  rotator.setPosition(0.3);

        // --- Initialize IMU ---



        waitForStart();
        //distance sensor

        while (opModeIsActive()) {





            if (gamepad1.dpad_down) {
                armservo.setPosition(0.1);//lower pos
            }
            if (gamepad1.dpad_right) {
                armservo.setPosition(0.05);
            }
            if (gamepad1.dpad_left) {
                armservo.setPosition(0);
            }
            if (gamepad1.dpad_up) {
                armservo.setPosition(0.2);//higher pos
            }


            telemetry.addData("armservo Pos", armservo.getPosition());
           // telemetry.addData("armservo Pos", armservo.get());


            telemetry.update();
        }

    }
}



