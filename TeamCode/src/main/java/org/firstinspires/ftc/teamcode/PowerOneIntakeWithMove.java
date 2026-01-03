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


@TeleOp(name="PowerIntakeWithMoving", group="Robot")
public class PowerOneIntakeWithMove extends LinearOpMode {

    private DcMotor intake;
    private Servo armservo;
    public DcMotorEx shooter;
    public Servo kicker;
    public DcMotor bootkicker;
    // public Servo rotator;
    RevColorSensorV3 distanceSensor;
    private static final double VELO_CLOSE = 700;//1530
    private static final double VELO_FAR = 900;//1660
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private final int ROBOT_OR_FIELD_CENTRIC = 1;


    @Override
    public void runOpMode() {

        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        telemetry.addData("Status of Distance sensor", "Initialized");
        leftFrontDrive = hardwareMap.dcMotor.get("LeftFront");
        leftBackDrive = hardwareMap.dcMotor.get("LeftBack");
        rightFrontDrive = hardwareMap.dcMotor.get("RightFront");
        rightBackDrive = hardwareMap.dcMotor.get("RightBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        armservo = hardwareMap.get(Servo.class, "armservo");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Initialize intake/shooter ---
        kicker = hardwareMap.get(Servo.class, "kicker");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        //  intake = hardwareMap.get(DcMotor.class, "intake");


        kicker.setPosition(0.25);//starting kicker value
        //  intake.setPower(0);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Initialize Limelight / rotator ---


        //  rotator.setPosition(0.3);

        // --- Initialize IMU ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);

        double headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();
        //distance sensor

        while (opModeIsActive()) {
            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
            // --- Drivetrain field-centric ---
            double y = -gamepad1.left_stick_y / 1.5;
            double x = gamepad1.left_stick_x / 1.5;
            double rx = gamepad1.right_stick_x / 1.5;

            if (gamepad1.options) imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;

            double rotX, rotY;
            if (ROBOT_OR_FIELD_CENTRIC == 1) {
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX *= 1.1;
            } else {
                rotX = x;
                rotY = y;
            }

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFrontDrive.setPower((rotY + rotX + rx) / denominator);
            leftBackDrive.setPower((rotY - rotX + rx) / denominator);
            rightFrontDrive.setPower((rotY - rotX - rx) / denominator);
            rightBackDrive.setPower((rotY + rotX - rx) / denominator);


//// Arm servo (independent)
//            if (gamepad1.dpad_right) {
//                armservo.setPosition(0.55);
//            }
//             else if (gamepad1.dpad_left) {
//                armservo.setPosition(0.8);
//            }
//             else if (gamepad1.b) {
//                armservo.setPosition(0.65);//around middle
//            } else if (gamepad1.y) {
//                armservo.setPosition(0.6);
//            }
            if (gamepad2.dpad_down) {
                bootkicker.setPower(-0.4);
            }

            if (gamepad2.y) {
                if (distanceCM < 7) {
                    kicker.setPosition(0.6);
                    sleep(600);
                    kicker.setPosition(0.25);
                }
            }
            if (gamepad2.x && distanceCM <7) {
                kicker.setPosition(0.6);
                sleep(600);
                kicker.setPosition(0.25);
            }

            if (gamepad2.a) {
                shooter.setVelocity(VELO_CLOSE);
            }
            if (gamepad2.b) {
                shooter.setVelocity(VELO_FAR);
            }
//0.6-0.7
            if (gamepad2.dpad_up) {
                intake.setPower(-0.9);
            }
            if (gamepad2.dpad_left) {
                armservo.setPosition(0.67);//lower position
            }
            if (gamepad2.dpad_right) {
                armservo.setPosition(0.7);//higher position
            }
            telemetry.addData("Rotator Power", intake.getPower());
            telemetry.addData("armservo Pos", armservo.getPosition());
            telemetry.addData("Distance", distanceCM);

            telemetry.update();
        }

    }
}



