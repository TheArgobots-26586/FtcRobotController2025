package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="DECODE_TeleoperatedV6", group="R  obot")
public class DECODE_TeleoperatedV4V6 extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    public Servo kicker = null;
    public DcMotorEx shooter = null;

    // ===== ROTATOR + LIMELIGHT =====
    private Servo rotator;
    private Limelight3A limelight;


    final double minPos = 0.45;
    final double maxPos = 1.0;
    final double centerPos = 0.725;

    final double SERVO_GAIN = 0.0067;
    double lastAlignedPos = centerPos;

    private static final double TARGET_VELO0 = 0;
    private static final double TARGET_VELO1 = 1450;
    private static final double TARGET_VELO2 = 1000;

    // ===== CAMERA CONSTANTS =====
    private final double CAMERA_HEIGHT_INCHES = 16;
    private final double TAG_HEIGHT_INCHES = 34.7;
    private final double CAMERA_PITCH_DEGREES = 10;

    // Field-centric toggle
    private final int ROBOT_OR_FIELD_CENTRIC = 1;

    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.dcMotor.get("LeftFront");
        leftBackDrive = hardwareMap.dcMotor.get("LeftBack");
        rightFrontDrive = hardwareMap.dcMotor.get("RightFront");
        rightBackDrive = hardwareMap.dcMotor.get("RightBack");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kicker = hardwareMap.get(Servo.class, "kicker");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        kicker.setPosition(0);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== ROTATOR + LIMELIGHT INIT =====
        rotator = hardwareMap.get(Servo.class, "decodetesting");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        // Start rotator safely inside limits
        rotator.setPosition(centerPos);

        // ===== IMU =====
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

            // ===== DRIVETRAIN =====
            double y = -gamepad1.left_stick_y / 1.5;
            double x = gamepad1.left_stick_x / 1.5;
            double rx = gamepad1.right_stick_x / 1.5;

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

            // ===== SHOOTER =====
            if (gamepad2.b) {
                shooter.setVelocity(TARGET_VELO1);
            } else if (gamepad2.dpad_right || gamepad2.dpad_left) {
                shooter.setVelocity(TARGET_VELO2);
            } else if (gamepad2.a) {
                shooter.setVelocity(TARGET_VELO0);
            }

            if (gamepad2.y) {
                kicker.setPosition(0.8);
                sleep(1000);
                kicker.setPosition(0);
            }


            LLResult r = limelight.getLatestResult();

            if (r != null && r.isValid()) {

                double tx = r.getTx();
                double ty = r.getTy();

                final double TX_DEADZONE_DEG = 0.8;

                telemetry.addLine("APRIL TAG DETECTED");

                if (Math.abs(tx) >= TX_DEADZONE_DEG) {
                    double servoOffset = tx * SERVO_GAIN;
                    double targetPos = centerPos + servoOffset;


                    targetPos = Math.max(minPos, Math.min(maxPos, targetPos));

                    lastAlignedPos = targetPos;
                    rotator.setPosition(lastAlignedPos);
                }

                double totalVerticalAngle = CAMERA_PITCH_DEGREES + ty;
                double totalVerticalRadians = Math.toRadians(totalVerticalAngle);

                double distanceInches =
                        (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES)
                                / Math.sin(totalVerticalRadians);

                telemetry.addData("Distance (in)", distanceInches);

            } else {

                rotator.setPosition(lastAlignedPos);
            }

            telemetry.update();
        }
    }
}