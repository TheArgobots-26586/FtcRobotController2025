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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="kicker", group="Robot")
public class kicker extends LinearOpMode {

    private Servo kicker;



    @Override
    public void runOpMode() {

        kicker = hardwareMap.get(Servo.class, "kicker");


        // --- Initialize intake/shooter ---


        // --- Initialize Limelight / rotator ---


        //  rotator.setPosition(0.3);

        // --- Initialize IMU ---



        waitForStart();
        //distance sensor

        while (opModeIsActive()) {



            if (gamepad1.x) {
                kicker.setPosition(0.6);//higher position
            }

            if (gamepad1.dpad_down) {
                kicker.setPosition(0.33);
            }
            if (gamepad1.dpad_right) {
                kicker.setPosition(0.25);
            }
         //   if (gamepad1.dpad_left) {
           //     kicker.setPosition(0.9);
           // }


            telemetry.addData("armservo Pos", kicker.getPosition());
           // telemetry.addData("armservo Pos", armservo.get());


            telemetry.update();
        }

    }
}



