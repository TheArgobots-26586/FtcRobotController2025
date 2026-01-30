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
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_left) {
                rotator.setPosition(0.5);//lower pos
            }
            if (gamepad1.dpad_right) {
                rotator.setPosition(0.85);
            }
            if (gamepad1.dpad_down) {
                rotator.setPosition(0.9);
            }
            if(gamepad1.dpad_up) {
                rotator.setPosition(1.0);
            }
            if(gamepad1.x) {
                rotator.setPosition(0.8);
            }
            telemetry.addData("rotator Pos", rotator.getPosition());
            telemetry.update();
        }

    }
}



