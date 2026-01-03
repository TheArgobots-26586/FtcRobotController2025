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


@TeleOp(name="IntakePowerOneMotor", group="Robot")
public class intake extends LinearOpMode {

    private DcMotor intake;
    private Servo armservo;



    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        armservo = hardwareMap.get(Servo.class, "arm");


        // --- Initialize intake/shooter ---


        // --- Initialize Limelight / rotator ---


        //  rotator.setPosition(0.3);

        // --- Initialize IMU ---



        waitForStart();
        //distance sensor

        while (opModeIsActive()) {



            if (gamepad1.x) {
                intake.setPower(-1);
            }

            if (gamepad1.x) {
                intake.setPower(-0.9);
            }
            if (gamepad1.x) {
                intake.setPower(-0.8);
            }
            if (gamepad1.x) {
                intake.setPower(-1);
            }
            if (gamepad1.dpad_down) {
                armservo.setPosition(0.5);
            }



            telemetry.addData("armservo Pos", intake.getPower());
            // telemetry.addData("armservo Pos", armservo.get());


            telemetry.update();
        }

    }
}