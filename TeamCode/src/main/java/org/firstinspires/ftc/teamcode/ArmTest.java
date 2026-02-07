package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="ArmTest", group="Robot")
public class ArmTest extends LinearOpMode {

    private Servo armservo;
    private DcMotor intake;
    private ColorRangeSensor distanceSensor;



    @Override
    public void runOpMode() {

        armservo = hardwareMap.get(Servo.class, "armservo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");


        // --- Initialize intake/shooter ---


        // --- Initialize Limelight / rotator ---


        //  rotator.setPosition(0.3);

        // --- Initialize IMU ---



        waitForStart();
        //distance sensor

        while (opModeIsActive()) {
            distanceSensor.getDistance(DistanceUnit.CM);
            if (gamepad1.dpad_down) {
                armservo.setPosition(0.29);//lower pos
            }
            if (gamepad1.dpad_right) {
                armservo.setPosition(0.28);
            }
            if (gamepad1.b) {
                armservo.setPosition(0.26);
            }
            if (gamepad1.dpad_left) {
                armservo.setPosition(0.24);
            }
            if (gamepad1.dpad_up) {
                armservo.setPosition(0.22);//higher pos
            }
            if(gamepad1.y) {
                armservo.setPosition(0.25);
            }
            if (gamepad1.a) {
                intake.setPower(-0.9);
            }
            if (gamepad1.x) {
                intake.setPower(0.9);
            }



            telemetry.addData("armservo Pos", armservo.getPosition());
            telemetry.addData("intake Pow", intake.getPower());
            telemetry.addData("Ditance", distanceSensor.getDistance(DistanceUnit.CM));
           // telemetry.addData("armservo Pos", armservo.get());


            telemetry.update();
        }

    }
}



