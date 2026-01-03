package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="DECODE_teleopV2", group="Robot")
public class DECODE_TeleoperatedV2 extends LinearOpMode {

    // ---------------- DRIVETRAIN ----------------
    private DcMotor leftFrontDrive  = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightBackDrive  = null;

    public Servo kicker = null;
    public DcMotorEx shooter = null;
    public DcMotor intake = null;

    private static final double TARGET_VELO0 = 0;
    private static final double TARGET_VELO1 = 1450; // FAST
    private static final double TARGET_VELO2 = 1000; // MED / SLOW

    // Field-centric toggle
    private final int ROBOT_OR_FIELD_CENTRIC = 1;

    @Override
    public void runOpMode() {


        leftFrontDrive  = hardwareMap.dcMotor.get("LeftFront");
        leftBackDrive   = hardwareMap.dcMotor.get("LeftBack");
        rightFrontDrive = hardwareMap.dcMotor.get("RightFront");
        rightBackDrive  = hardwareMap.dcMotor.get("RightBack");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kicker = hardwareMap.get(Servo.class, "kicker");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        kicker.setPosition(0);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setPower(0);


        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);

        double headingOffset =
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.addLine("Robot Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y / 1.5;
            double x  =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;

            if (gamepad1.options) imu.resetYaw();

            double botHeading =
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
                            - headingOffset;

            double rotX, rotY;

            if (ROBOT_OR_FIELD_CENTRIC == 1) {
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX *= 1.1;
            } else {
                rotX = x;
                rotY = y;
            }

            double denominator =
                    Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            leftFrontDrive.setPower((rotY + rotX + rx) / denominator);
            leftBackDrive.setPower((rotY - rotX + rx) / denominator);
            rightFrontDrive.setPower((rotY - rotX - rx) / denominator);
            rightBackDrive.setPower((rotY + rotX - rx) / denominator);

            if (gamepad2.x) {
                kicker.setPosition(0);
            }
            if (gamepad2.y) {
                kicker.setPosition(0.8);
                sleep(1000);
                kicker.setPosition(0.33);
            }

            if (gamepad2.b) {
                shooter.setVelocity(TARGET_VELO1); // FAST
            }
            else if (gamepad2.dpad_right) {
                shooter.setVelocity(TARGET_VELO2); // MED
            }
            else if (gamepad2.dpad_left) {
                shooter.setVelocity(TARGET_VELO2); // SLOW (same)
            }
            else if (gamepad2.a) {
                shooter.setVelocity(TARGET_VELO0); // STOP
            }

            if (gamepad2.dpad_down) {
                intake.setPower(0.75);
            }
            else if (gamepad2.dpad_up) {
                intake.setPower(0);
            }

            telemetry.addData("Target Velo", shooter.getVelocity());
            telemetry.addData("Heading (rad)", botHeading);
            telemetry.update();
        }
    }
}
