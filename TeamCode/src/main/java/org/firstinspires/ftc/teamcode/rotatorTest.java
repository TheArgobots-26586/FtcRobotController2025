package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="rotatorTest", group="Robot")
public class rotatorTest extends LinearOpMode {
    private Servo rotator;
    private AnalogInput turretinput;
    double MAXV = 0.47,MINV = 2.79;

    @Override
    public void runOpMode() {

        rotator = hardwareMap.get(Servo.class, "rotator");
        turretinput = hardwareMap.get(AnalogInput.class,"turretinput");
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_left) {
                rotator.setPosition(0.778);//lower pos
            }
            if (gamepad1.dpad_right) {
                rotator.setPosition(0.578);
            }
            if (gamepad1.dpad_down) {
                rotator.setPosition(0);
            }
            if(gamepad1.dpad_up) {
                rotator.setPosition(1);
            }
            if(gamepad1.x) {
                rotator.setPosition(0.87);
            }
            double curV = turretinput.getVoltage();
            double turrpos = (curV-MINV) / (MAXV-MINV);
            telemetry.addData("cur Voltage", curV);
            telemetry.addData("turret exact pose", turrpos);
            telemetry.addData("rotator Pos", rotator.getPosition());
            telemetry.update();
        }

    }
}



