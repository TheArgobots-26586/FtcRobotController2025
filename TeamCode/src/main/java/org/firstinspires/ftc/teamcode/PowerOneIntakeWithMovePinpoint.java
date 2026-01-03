package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="PowerOneIntakeWithMovePinpoint", group="Robot")
public class PowerOneIntakeWithMovePinpoint extends LinearOpMode {

    private DcMotor intake;
    private Servo armservo;
    public DcMotorEx shooter;
    public Servo kicker;
    public DcMotor bootkicker;
    RevColorSensorV3 distanceSensor;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // --- Pinpoint Driver ---
    private GoBildaPinpointDriver odo;
    private boolean lastOptions = false;

    private static final double VELO_CLOSE = 700;
    private static final double VELO_FAR = 900;
    private final int ROBOT_OR_FIELD_CENTRIC = 1;

    @Override
    public void runOpMode() {
        // --- Hardware Mapping ---
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        leftFrontDrive = hardwareMap.dcMotor.get("LeftFront");
        leftBackDrive = hardwareMap.dcMotor.get("LeftBack");
        rightFrontDrive = hardwareMap.dcMotor.get("RightFront");
        rightBackDrive = hardwareMap.dcMotor.get("RightBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        armservo = hardwareMap.get(Servo.class, "armservo");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");
        kicker = hardwareMap.get(Servo.class, "kicker");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // --- Drive Directions ---
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Pinpoint Initialization ---
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Calibration Settings (Matches your working snippet)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(-111.0, -127.2, DistanceUnit.MM);

        // Resetting to start at 0
        odo.resetPosAndIMU();

        kicker.setPosition(0.25);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Robot Ready - Pinpoint IMU Active");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // IMPORTANT: Update() must be called to get new data
            odo.update();

            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);

            // --- Reset heading on options button press ---
            boolean options = gamepad1.options;
            if (options && !lastOptions) {
                odo.resetPosAndIMU();
            }
            lastOptions = options;

            // --- Drivetrain inputs ---
            double y = -gamepad1.left_stick_y / 1.5;
            double x = gamepad1.left_stick_x / 1.5;
            double rx = gamepad1.right_stick_x / 1.5;

            // --- HEADING LOGIC (Corrected to your snippet style) ---
            double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);

            double rotX, rotY;
            if (ROBOT_OR_FIELD_CENTRIC == 1) {
                // Rotation math using the heading from Pinpoint
                rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            } else {
                rotX = x;
                rotY = y;
            }

            // --- Mecanum Wheel Power Calculation ---
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFrontDrive.setPower((rotY + rotX + rx) / denominator);
            leftBackDrive.setPower((rotY - rotX + rx) / denominator);
            rightFrontDrive.setPower((rotY - rotX - rx) / denominator);
            rightBackDrive.setPower((rotY + rotX - rx) / denominator);

            // --- Subsystem Controls ---
            if (gamepad2.dpad_down) {
                bootkicker.setPower(-0.4);
            } else {
                bootkicker.setPower(0);
            }

            if ((gamepad2.y || gamepad2.x) && distanceCM < 7) {
                kicker.setPosition(0.25);
                sleep(600);
                kicker.setPosition(0.6);
            }

            if (gamepad2.a) shooter.setVelocity(VELO_CLOSE);
            if (gamepad2.b) shooter.setVelocity(VELO_FAR);

            // Intake toggle/hold
            if (gamepad2.dpad_up) {
                intake.setPower(-0.9);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.dpad_left) armservo.setPosition(0.67);
            if (gamepad2.dpad_right) armservo.setPosition(0.7);

            // --- Telemetry ---
            telemetry.addData("Heading (Deg)", Math.toDegrees(heading));
            telemetry.addData("Distance CM", distanceCM);
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.update();
        }
    }
}