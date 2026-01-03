package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="DECODE_TeleOp_FieldCentricTanmay", group="Robot")
public class DecodeTestFinal1Tanmay extends LinearOpMode {

    // ---------------- Drivetrain ----------------
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // ---------------- Mechanisms ----------------
    private DcMotor intake;
    private DcMotor bootkicker;
    private DcMotorEx shooter;
    private Servo kicker;

    // ---------------- Sensors ----------------
    private DistanceSensor ballSensor;
    private IMU imu;

    // ---------------- Constants ----------------
    private static final double SHOOTER_VELOCITY = 1550;
    private static final double BALL_PRESENT_DISTANCE_IN = 2.5;

    private static final double KICKER_REST = 0.33;
    private static final double KICKER_FIRE = 0.8;

    // EXACT same toggle you had
    private final int ROBOT_OR_FIELD_CENTRIC = 1;

    @Override
    public void runOpMode() {

        // -------- Hardware Map --------
        leftFront  = hardwareMap.dcMotor.get("LeftFront");
        leftBack   = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack  = hardwareMap.dcMotor.get("RightBack");

        intake     = hardwareMap.dcMotor.get("intake");
        bootkicker = hardwareMap.dcMotor.get("bootkicker");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker     = hardwareMap.get(Servo.class, "kicker");

       ballSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        double DC = ballSensor.getDistance(DistanceUnit.CM);
        // -------- Motor Setup --------
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(KICKER_REST);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // -------- IMU (COPIED FROM YOUR CODE) --------
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        double headingOffset =
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.addLine("FIELD CENTRIC READY");
        telemetry.update();
        waitForStart();

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            // ---------- FIELD-CENTRIC DRIVE (UNCHANGED) ----------
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

            leftFront.setPower((rotY + rotX + rx) / denominator);
            leftBack.setPower((rotY - rotX + rx) / denominator);
            rightFront.setPower((rotY - rotX - rx) / denominator);
            rightBack.setPower((rotY + rotX - rx) / denominator);

            // ---------- BALL DETECTION ----------
            //boolean ballPresent =
                  //  distanceSensor.getDistance(DistanceUnit.INCH)
                        //    < BALL_PRESENT_DISTANCE_IN;

            // ---------- INTAKE + BOOTKICKER ----------
//            if (gamepad2.right_bumper && !ballPresent) {
//                intake.setPower(1.0);
//                bootkicker.setPower(1.0);
//            } else if (gamepad2.left_bumper) {
//                intake.setPower(0);
//                bootkicker.setPower(0);
//            }

            // ---------- SHOOTER ----------
            if (gamepad2.a) {
                shooter.setVelocity(SHOOTER_VELOCITY);
            } else if (gamepad2.b) {
                shooter.setPower(0);
            }

            // ---------- KICKER SERVO ----------
            if (gamepad2.y) {
                kicker.setPosition(KICKER_FIRE);
                sleep(400);
                kicker.setPosition(KICKER_REST);
            }

            // ---------- TELEMETRY ----------
            telemetry.addData("Heading (deg)",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
          //  telemetry.addData("Ball Present", );
            telemetry.addData("Ball Distance (in)",DC);

            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.update();
        }
    }
}