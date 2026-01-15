package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
// Added Pinpoint Import
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="DecodeTestFinal2bPinpoint", group="Robot")
public class DecodeTestFinal2bPinpoint extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private DcMotor intake;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    private Servo kicker;
    private DcMotor bootkicker;
    private Servo armservo;
    private RevColorSensorV3 distanceSensor;

    // Replace IMU with Pinpoint
    private GoBildaPinpointDriver odo;

    private Limelight3A limelight;
    private Servo rotator;

    private static final double CENTER_POS = 0.35;
    private static final double SERVO_GAIN = 0.0067;
    private static final double ROTATOR_MIN = 0.45;
    private static final double ROTATOR_MAX = 1.0;
    private double lastAlignedPos = CENTER_POS;

    // DISTANCE CONSTANTS
    private static final double CAMERA_HEIGHT_IN = 12.0;
    private static final double CAMERA_PITCH_DEG = 10.0;
    private static final double TAG_HEIGHT_IN    = 29.5;

    // CONSTANTS
    private static final double VELO_CLOSE = 700;

    enum RobotState {
        IDLE,
        COLLECT,
        SHOOT
    }

    RobotState currentState = RobotState.IDLE;

    boolean lastA = false;
    boolean lastB = false;
    boolean lastOptions = false;

    @Override
    public void runOpMode() {

        // --- Hardware Mapping ---
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "RightBack");

        intake     = hardwareMap.get(DcMotor.class, "intake");
        kicker     = hardwareMap.get(Servo.class, "kicker");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");
        armservo   = hardwareMap.get(Servo.class, "armservo");

        rotator   = hardwareMap.get(Servo.class, "rotator");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // --- Pinpoint Initialization ---
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-111.0, -127.2, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.resetPosAndIMU();

        // --- Motor/Servo Setup ---
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(0.33);
        rotator.setPosition(CENTER_POS);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Pinpoint Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // MUST call update() to get new data from Pinpoint
            odo.update();

            // Reset heading on "Options"
            boolean options = gamepad1.options;
            if (options && !lastOptions) {
                odo.resetPosAndIMU();
            }
            lastOptions = options;

            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
            double tagDistanceIn = -1;

            // --- MODE TOGGLE ---
            boolean a = gamepad2.a;
            boolean b = gamepad2.b;

            if (a && !lastA)
                currentState = (currentState == RobotState.COLLECT) ? RobotState.IDLE : RobotState.COLLECT;
            if (b && !lastB)
                currentState = (currentState == RobotState.SHOOT) ? RobotState.IDLE : RobotState.SHOOT;

            lastA = a;
            lastB = b;

            // --- FIELD CENTRIC DRIVE (Pinpoint Driven) ---
            double y  = -gamepad1.left_stick_y / 1.5;
            double x  =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;

            // Use Pinpoint heading
            double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFront.setPower((rotY + rotX + rx) / denom);
            leftBack.setPower((rotY - rotX + rx) / denom);
            rightFront.setPower((rotY - rotX - rx) / denom);
            rightBack.setPower((rotY + rotX - rx) / denom);

            // --- LIMELIGHT CALCULATIONS ---
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();

                // Rotator alignment
                double targetPos = CENTER_POS + (tx * SERVO_GAIN);
                targetPos = Math.max(ROTATOR_MIN, Math.min(ROTATOR_MAX, targetPos));
                lastAlignedPos = targetPos;
                rotator.setPosition(lastAlignedPos);

                // Distance Trig
                double angleRad = Math.toRadians(CAMERA_PITCH_DEG + ty);
                tagDistanceIn = (TAG_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleRad);
            }

            // --- STATE MACHINE ---
            switch (currentState) {
                case IDLE:
                    shooter.setVelocity(VELO_CLOSE);
                    intake.setPower(-0.9);
                    bootkicker.setPower(-0.4);
                    kicker.setPosition(0.33);
                    break;
                case COLLECT:
                    kicker.setPosition(0.33);
                    bootkicker.setPower(-0.4);
                    intake.setPower(-1);
                    armservo.setPosition(0.17);
                    break;
                case SHOOT:
                    intake.setPower(-1);
                    bootkicker.setPower(-0.4);
                    shooter.setVelocity(VELO_CLOSE);

                    if (distanceCM < 7) {
                        // Added logic from your snippet: check tag distance before shooting
                        if (tagDistanceIn < 60 && tagDistanceIn > 45){
                            shooter.setVelocity(VELO_CLOSE);
                        }
                        sleep(300);
                        kicker.setPosition(0.8);
                        sleep(600);
                        kicker.setPosition(0.33);
                    } else {
                        kicker.setPosition(0.33);
                    }
                    break;
            }

            //  TELEMETRY
            telemetry.addData("MODE", currentState);
            telemetry.addData("Heading (Deg)", Math.toDegrees(heading));
            telemetry.addData("Tag Distance (in)", tagDistanceIn);
            telemetry.addData("Rotator Pos", lastAlignedPos);
            telemetry.addData("X-Pod Raw", odo.getEncoderX());
            telemetry.update();
        }
    }
}