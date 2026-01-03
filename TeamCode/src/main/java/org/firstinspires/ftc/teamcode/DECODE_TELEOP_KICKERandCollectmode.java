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

@TeleOp(name="DECODE_TeleOp_Final", group="Robot")
public class DECODE_TELEOP_KICKERandCollectmode extends LinearOpMode {

    // ---------------- Hardware ----------------
    private DcMotor intake;
    private Servo armservo;
    private DcMotorEx shooter;
    private Servo kicker;
    private DcMotor bootkicker;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private RevColorSensorV3 distanceSensor;

    private final int ROBOT_OR_FIELD_CENTRIC = 1;

    // ---------------- Auto-shoot state ----------------
    private boolean autoShootEnabled = false;
    private boolean kickerExtended = false;
    private long kickerTimer = 0;

    // ---------------- Robot Modes ----------------
    enum RobotMode {
        COLLECTING,
        SHOOTING,
        IDLE
    }

    RobotMode currentMode = RobotMode.COLLECTING;

    // ---------------- Double-click variables ----------------
    long lastAPressTime = 0;
    static final long DOUBLE_CLICK_TIME = 300; // ms

    @Override
    public void runOpMode() {

        // ---------------- Init Hardware ----------------
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");

        leftFrontDrive  = hardwareMap.dcMotor.get("LeftFront");
        leftBackDrive   = hardwareMap.dcMotor.get("LeftBack");
        rightFrontDrive = hardwareMap.dcMotor.get("RightFront");
        rightBackDrive  = hardwareMap.dcMotor.get("RightBack");

        intake     = hardwareMap.get(DcMotor.class, "intake");
        armservo   = hardwareMap.get(Servo.class, "armservo");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");
        kicker     = hardwareMap.get(Servo.class, "kicker");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kicker.setPosition(0.33);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---------------- IMU ----------------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        double headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive()) {

            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
            long now = System.currentTimeMillis();

            // ===== FIELD-CENTRIC DRIVE =====
            double y  = -gamepad1.left_stick_y / 1.5;
            double x  =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;

            if (gamepad1.options) imu.resetYaw();

            double botHeading =
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;

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

            // ===== DOUBLE-CLICK IDLE MODE =====
            if (gamepad1.a) {
                if (now - lastAPressTime < DOUBLE_CLICK_TIME) {
                    currentMode = (currentMode != RobotMode.IDLE) ? RobotMode.IDLE : RobotMode.COLLECTING;
                }
                lastAPressTime = now;
            }

            // ===== MODE LOGIC =====
            switch (currentMode) {

                // ---------------- IDLE ----------------
                case IDLE:
                    intake.setPower(0);
                    shooter.setVelocity(0);
                    bootkicker.setPower(0);
                    kicker.setPosition(0.33);

                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);

                    telemetry.addData("Mode", "IDLE");
                    break;

                // ---------------- COLLECTING ----------------
                case COLLECTING:
                    autoShootEnabled = false;
                    shooter.setVelocity(0);
                    kicker.setPosition(0.33);

                    // Intake control
                    if (gamepad2.dpad_up) {
                        intake.setPower(-0.9);
                        bootkicker.setPower(1.0); // automatically powered while intake runs
                    } else {
                        intake.setPower(0);
                        bootkicker.setPower(0);
                    }
                    break;

                // ---------------- SHOOTING ----------------
                case SHOOTING:
                    intake.setPower(0);
                    bootkicker.setPower(0);

                    if (gamepad2.a) shooter.setVelocity(700);
                    if (gamepad2.b)  shooter.setVelocity(900);

                    // Auto-shoot control
                    if (gamepad2.y) autoShootEnabled = true;
                    if (gamepad2.x) autoShootEnabled = false;

                    boolean ballPresent = distanceCM < 7.3;

                    if (autoShootEnabled && ballPresent) {

                        if (!kickerExtended) {
                            kicker.setPosition(0.8);
                            kickerExtended = true;
                            kickerTimer = now;
                        } else if (kickerExtended && now - kickerTimer > 300) {
                            kicker.setPosition(0.33);
                            kickerExtended = false;
                            kickerTimer = now;
                        }

                    } else {
                        kicker.setPosition(0.33);
                        kickerExtended = false;
                    }
                    break;
            }

            telemetry.addData("Mode", currentMode);
            telemetry.addData("Auto Shoot", autoShootEnabled);
            telemetry.addData("Ball Distance (cm)", distanceCM);
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.update();
        }
    }
}
