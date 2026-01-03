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

@TeleOp(name="SampleFieldCentric", group="Robot")
public class SampleFieldCentric extends LinearOpMode {


    private DcMotor leftFront, leftBack, rightFront, rightBack;

    private IMU imu;
    boolean lastOptions = false;


    @Override
    public void runOpMode() {



        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "RightBack");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {
            boolean options = gamepad1.options;

            if (options && !lastOptions) {
                imu.resetYaw();
            }
            lastOptions = options;

            // DRIVETRAIN
            double y  = -gamepad1.left_stick_y / 1.5;
            double x  =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFront.setPower((rotY + rotX + rx) / denom);
            leftBack.setPower((rotY - rotX + rx) / denom);
            rightFront.setPower((rotY - rotX - rx) / denom);
            rightBack.setPower((rotY + rotX - rx) / denom);

            // STATE MACHINE








            telemetry.update();
        }
    }
}


//Gamepad 1 (Driver)
//
//Left stick → Drive (field-centric)
//
//Right stick X → Turn
//
//Gamepad 2 (Operator)
//
//A → Toggle COLLECT mode
//
//B → Toggle SHOOT mode
//
//D-pad Right → Start auto-fire (shoot all balls that are in)

//
//COLLECT mode active → Intake + bootkicker run
//
//SHOOT mode active → Shooter spins up
//
//No balls detected → Shooting stops automatically