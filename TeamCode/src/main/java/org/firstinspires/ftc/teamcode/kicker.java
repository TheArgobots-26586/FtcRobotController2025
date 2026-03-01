package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="kicker", group="Robot")
public class kicker extends LinearOpMode {

    private Servo kicker;
    private RevColorSensorV3 distanceSensor;



    @Override
    public void runOpMode() {

        kicker = hardwareMap.get(Servo.class, "kicker");
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");

        // --- Initialize intake/shooter ---


        // --- Initialize Limelight / rotator ---


        //  rotator.setPosition(0.3);

        // --- Initialize IMU ---


        distanceSensor.setGain(8);
        waitForStart();
        //distance sensor

        while (opModeIsActive()) {



            if (gamepad1.x) {
                kicker.setPosition(0.36);//higher position
            }

            if (gamepad1.dpad_down) {
                kicker.setPosition(0.75);
            }
            if (gamepad1.dpad_right) {
                kicker.setPosition(0.37);
            }
            if (gamepad1.dpad_left) {
                kicker.setPosition(0.375);
            }
            if (gamepad1.dpad_up) {
                kicker.setPosition(0.38);
            }
            if (gamepad1.a) {
                kicker.setPosition(0.385);
            }
            //   if (gamepad1.dpad_left) {
            //     kicker.setPosition(0.9);
            // }


            telemetry.addData("kicker Pos", kicker.getPosition());
            telemetry.addData("Distance Sensor Value: ", distanceSensor.getDistance(DistanceUnit.CM));


            telemetry.update();
        }

    }
}