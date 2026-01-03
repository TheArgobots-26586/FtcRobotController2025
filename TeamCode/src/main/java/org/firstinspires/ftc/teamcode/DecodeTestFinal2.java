package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="DecodeTestFinal2", group="Robot")
public class DecodeTestFinal2 extends LinearOpMode {

    // Drivetrain
    private DcMotor leftFrontDrive  = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightBackDrive  = null;
    // Shooter / Intake / Kicker
    public Servo kicker = null;
    public DcMotorEx shooter = null;
    //  public DcMotor intake = null;

    // Limelight / Rotator/distance
    private Servo rotator;
    //private Limelight3A limelight;
    RevColorSensorV3 distanceSensor; // Declare the sensor

    // Constants for servo alignment
    final double centerPos = 0.35;
    final double minPos = 0;
    final double maxPos = 1;
    final double SERVO_GAIN = 0.0067;
    double lastAlignedPos = centerPos;

    // Shooter velocity zones
    private static final double VELO_CLOSE = 1000;
    private static final double VELO_FAR   = 1450;

    // Field-centric toggle
    private final int ROBOT_OR_FIELD_CENTRIC = 1;

    // Limelight / Distance measurement constants
    private final double CAMERA_HEIGHT_INCHES = 16;
    private final double TAG_HEIGHT_INCHES    = 29.875;
    private final double CAMERA_PITCH_DEGREES = 10;

    @Override
    public void runOpMode() {

        // --- Initialize drivetrain ---
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        telemetry.addData("Status of Distance sensor", "Initialized");
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

        // --- Initialize intake/shooter ---
        kicker = hardwareMap.get(Servo.class, "kicker");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        //  intake = hardwareMap.get(DcMotor.class, "intake");
        //  rotator = hardwareMap.get(Servo.class, "rotator");
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");

        kicker.setPosition(0.3);
        //  intake.setPower(0);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Initialize Limelight / rotator ---


        //limelight.pipelineSwitch(0);
        //limelight.start();

        // rotator.setPosition(centerPos);

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
        double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
        while (opModeIsActive()) {

            // --- Drivetrain field-centric ---
            double y  = -gamepad1.left_stick_y / 1.5;
            double x  =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;

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

            // --- Intake / kicker control ---
            //       if (gamepad2.dpad_up) {
            //          intake.setPower(0.75);
            //      } else if (gamepad2.dpad_down) {
            //         intake.setPower(0);
            //     }
            distanceCM = distanceSensor.getDistance(DistanceUnit.CM);


            if (distanceCM < 6) {
                kicker.setPosition(0.8);
                sleep(600);
                kicker.setPosition(0.33);
            }
            shooter.setVelocity(VELO_CLOSE);

            if (gamepad1.a){
                shooter.setVelocity(VELO_CLOSE);
            }
            if (gamepad1.b){
                shooter.setVelocity(VELO_FAR);
            }

            // --- Limelight / Rotator ---
            //cLLResult r = limelight.getLatestResult();
            double distance3D = -1; // default if no tag
          /*  if (r != null && r.isValid()) {

                telemetry.addLine("APRIL TAG DETECTED");
                double tx = r.getTx();
                double ty = r.getTy() - CAMERA_PITCH_DEGREES;//not necessarily needed

                // servo offset based on horizontal error
                double servoOffset = tx * SERVO_GAIN;
                double targetPos = centerPos + servoOffset;
                targetPos = Math.max(minPos, Math.min(maxPos, targetPos));
                lastAlignedPos = targetPos;
                rotator.setPosition(lastAlignedPos);

                // Distance calculation
                double totalVerticalRadians = Math.toRadians(CAMERA_PITCH_DEGREES + r.getTy());
                distance3D = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.sin(totalVerticalRadians);

                // --- Shooter proportional velocity ---
                // Linear interpolation between VELO_CLOSE and VELO_FAR
                // double VELO_MIN_DIST = 70; // example close launch zone distance in inches
                //double VELO_MAX_DIST = 140; // example far launch zone distance in inches
                shooter.setVelocity(VELO_CLOSE);
            } else {
                rotator.setPosition(lastAlignedPos);
                // No tag: stop shooter
            }
            */


            // --- Telemetry ---
            telemetry.addData("Front Left Power", leftFrontDrive.getPower());
            telemetry.addData("Front Right Power", rightFrontDrive.getPower());
            telemetry.addData("Back Left Power", leftBackDrive.getPower());
            telemetry.addData("Back Right Power", rightBackDrive.getPower());
            //        telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Kicker Pos", kicker.getPosition());
            telemetry.addData("Rotator Pos", lastAlignedPos);
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.addData("Distance sensor",distanceCM );

            telemetry.addData("Distance (in)", distance3D);
            telemetry.update();
        }
    }
}