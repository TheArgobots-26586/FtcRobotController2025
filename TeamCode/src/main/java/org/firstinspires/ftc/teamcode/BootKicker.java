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


@TeleOp(name="BootKicker-1.0 - BootKicker", group="Robot")
public class BootKicker extends LinearOpMode {

    private DcMotor bootkicker;



    @Override
    public void runOpMode() {

        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");

        waitForStart();
        //distance sensor

        while (opModeIsActive()) {



            if (gamepad1.x) {
                bootkicker.setPower(0.35);
            }

            if (gamepad1.x) {
                bootkicker.setPower(0.55);
            }
            if (gamepad1.x) {
                bootkicker.setPower(0.75);
            }
            if (gamepad1.x) {
                bootkicker.setPower(1);
            }


            telemetry.addData("bootkicker", bootkicker.getPower());
            telemetry.update();
        }

    }
}