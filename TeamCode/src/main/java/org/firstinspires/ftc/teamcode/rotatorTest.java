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


@TeleOp(name="rotatorTest", group="Robot")
public class rotatorTest extends LinearOpMode {

    private Servo rotator;




    @Override
    public void runOpMode() {

        rotator = hardwareMap.get(Servo.class, "rotator");
       // intake = hardwareMap.get(DcMotor.class, "intake");


        // --- Initialize intake/shooter ---


        // --- Initialize Limelight / rotator ---


        //  rotator.setPosition(0.3);

        // --- Initialize IMU ---



        waitForStart();
        //distance sensor

        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                rotator.setPosition(0.5);//lower pos
            }
            if (gamepad1.dpad_right) {
                rotator.setPosition(0.15);
            }




            telemetry.addData("armservo Pos", rotator.getPosition());

            // telemetry.addData("armservo Pos", armservo.get());


            telemetry.update();
        }

    }
}



