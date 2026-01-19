package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;

@TeleOp(name="DECODE_FINAL_TELEOP", group="Robot")
public class DECODE_FINAL_TELEOP extends LinearOpMode {

/*================CONTROLS================

    A  - Toggle COLLECT mode. Press again to return to IDLE
    B - Toggle SHOOT mode Press again to return to IDLE
    X - Hold to align TURRET with APRIL TAG
    Joystick - BASIC driver controls

==========================================*/


// CAMERA CODE INITIALIZATION FOR APRIL TAG and TURRET SERV0 ALIGNMENT IS BELOW

    private static final double CAMERA_HEIGHT_INCHES = 12.9;   // camera lens height
    private static final double TAG_HEIGHT_INCHES    = 29.5;   // AprilTag center height
    private static final double CAMERA_PITCH_DEGREES = 5.0;    // camera upward tilt
    private static final double SERVO_CENTER = 0.9;

    // Servo position that points the mechanism straight ahead (aligned with robot center)
    private static final double SERVO_MIN = 0.8;

    // Lowest safe servo position to prevent hitting the robot or hard stops
    private static final double SERVO_MAX = 1;

    // Highest safe servo position to prevent hitting the robot or hard stops
    private static final double SERVO_GAIN = 0.005;

    // How much the servo moves per degree of horizontal error (tx); higher = faster movement
    private static final double TX_DEADBAND = 4.0;

    double distanceInches = 0;

    //================CONSTANTS================//
    public static final double KICKER_DOWN = 0.225;
    public static final double KICKER_UP = 0.6;
    public static final double ARM_SERVO_POSITION = 0.245;
    public static final double INTAKE_IDLE = -0.1;
    public static final double BOOTKICKER_IDLE = -0.1;
    public static final double INTAKE_COLLECT = -0.9;
    public static final double BOOTKICKER_COLLECT = -0.4;
    public static final double INTAKE_ABORT = 0.5;
    public static final double BOOTKICKER_ABORT = 0.5;
    public static final double ARM_ABORT = 0.3;
    public static final double INTAKE_SHOOT = -0.2;
    public static final double BOOTKICKER_SHOOT = -0.2;
    public static final double MAX_COLOR_SENSED_DISTANCE = 7;


//ELECTRONIC SETUP

    private Limelight3A limelight;
    private DcMotor intake;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    private Servo kicker;
    private DcMotor bootkicker;
    private RevColorSensorV3 distanceSensor;
    private PinpointLocalizer pinpointLocalizer;

    Range range1 = new Range(0, 55, 1100);
    Range range2 = new Range(55, 80, 1200);
    Range range3 = new Range(80, Integer.MAX_VALUE, 1400);

    //STATE MACHINE SETUP
    enum RobotState {
        IDLE,
        COLLECT,
        SHOOT,
        ABORT
    }

    RobotState currentState = RobotState.IDLE;

    boolean lastA = false;
    boolean lastB = false;

    // Latch for auto-fire
    boolean firingEnabled = false;
    boolean lastOptions = false;
    private Servo turret = null;
    public Servo armservo = null;
    double tx=0;
    double ty=0;
    double newPos;
    @Override
    public void runOpMode() {
        //HARDWARE MAP
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "RightBack");

        intake     = hardwareMap.get(DcMotor.class, "intake");
        kicker     = hardwareMap.get(Servo.class, "kicker");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");
        armservo = hardwareMap.get(Servo.class, "armservo");
        turret = hardwareMap.get(Servo.class, "rotator");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //LIMELIGHT SETUP
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        //INITIALIZATIONS

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(KICKER_DOWN);
        armservo.setPosition(ARM_SERVO_POSITION);
        turret.setPosition(0.9);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinpointLocalizer = new PinpointLocalizer(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            armservo.setPosition(ARM_SERVO_POSITION);

            pinpointLocalizer.update();
            boolean options = gamepad1.options;
            if (options && !lastOptions) {
                pinpointLocalizer.resetHeading();
            }
            lastOptions = options;

            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);

            // DRIVETRAIN
            double y  = -gamepad1.left_stick_y / 1.5;
            double x  =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;

            double heading = pinpointLocalizer.getHeading();
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFront.setPower((rotY + rotX + rx) / denom);
            leftBack.setPower((rotY - rotX + rx) / denom);
            rightFront.setPower((rotY - rotX - rx) / denom);
            rightBack.setPower((rotY + rotX - rx) / denom);

            //distance mesuring

            // APRIL TAG DETECTION
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                tx = result.getTx();
                ty = result.getTy();

                double totalVerticalAngle = CAMERA_PITCH_DEGREES + ty;

                // Update the existing distanceInches variable (no 'double' prefix)
                distanceInches = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
                        Math.tan(Math.toRadians(totalVerticalAngle));

                //telemetry.addData("Tag Seen", true);
            }

            // A(Toggle for IDLE and COLLECT) B(Toggle for SHOOT and IDLE)

            boolean a = gamepad2.a;
            boolean b = gamepad2.b;
            boolean xPressed = gamepad2.x;

            if (a && !lastA) {
                if (currentState == RobotState.COLLECT) {
                    currentState = RobotState.IDLE;
                    tx = 0;
                } else {
                    currentState = RobotState.COLLECT;
                    tx = 0;
                }
            }
            if (b && !lastB) {
                if (currentState == RobotState.SHOOT) {
                    currentState = RobotState.IDLE;
                    tx = 0;
                } else {
                    currentState = RobotState.SHOOT;
                    if (result != null && result.isValid()) {
                        double val = Math.min(Math.max(0.8, turret.getPosition()+(tx/360)), 1);
                        turret.setPosition(val);
                        sleep(1000);
                        telemetry.addData("servo target pos", val);
                        telemetry.update();
                    }

                }
            }
            if(xPressed) {
                currentState = RobotState.ABORT;
            }
            lastA = a;
            lastB = b;


            // STATE MACHINE
            switch (currentState) {

                case IDLE:
                    shooter.setVelocity(shooterVelocity(distanceInches));
                    intake.setPower(INTAKE_IDLE);
                    turret.setPosition(0.9);
                    bootkicker.setPower(BOOTKICKER_IDLE);
                    kicker.setPosition(KICKER_DOWN);
                    break;

                case COLLECT:
                    shooter.setVelocity(shooterVelocity(distanceInches));
                    kicker.setPosition(KICKER_DOWN);
                    bootkicker.setPower(BOOTKICKER_COLLECT);
                    intake.setPower(INTAKE_COLLECT);
                    break;

                case SHOOT:
                    shooter.setVelocity(shooterVelocity(distanceInches));
                    // auto-fire while balls exist
                    if (distanceCM < 7) {
                        sleep(100);
                        kicker.setPosition(KICKER_UP);
                        sleep(600);
                        kicker.setPosition(KICKER_DOWN);
                        sleep(200);
                        telemetry.addData("kicker shoot", true);
                        telemetry.update();
                    }
                    // stop when no ball
                    if (distanceCM >= 7) {
                        kicker.setPosition(KICKER_DOWN);
                    }
                    intake.setPower(INTAKE_COLLECT);
                    bootkicker.setPower(BOOTKICKER_COLLECT);
                    telemetry.addData("shoot mode", true);
                    telemetry.update();
                    break;
                case ABORT:
                    intake.setPower(INTAKE_ABORT);
                    bootkicker.setPower(BOOTKICKER_ABORT);
                    armservo.setPosition(ARM_ABORT);
                    kicker.setPosition(KICKER_DOWN);
            }

            //telemetry
            telemetry.addData("MODE", currentState);
            telemetry.addData("Distance (cm)", distanceCM);
            telemetry.addData("Current State is:", currentState);
            telemetry.addData("Motor Power:", shooter.getVelocity());
            telemetry.addData("Intake Power:", intake.getPower());
            telemetry.addData("x:", pinpointLocalizer.getPoseEstimate().getX());
            telemetry.addData("y:", pinpointLocalizer.getPoseEstimate().getY());
            telemetry.addData("heading:", pinpointLocalizer.getHeading());
            telemetry.addData("Distance From Goal:", distanceInches);
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("ty (deg)", ty);
            telemetry.addData("servo current pos", turret.getPosition());
            telemetry.addData("Distance (in)", "%.1f", distanceInches);
            telemetry.update();
        }
    }
    public double shooterVelocity(double distanceInInches) {
        if (distanceInInches>=range1.l && distanceInInches<range1.r){
            return range1.speed;
        }
        else if (distanceInInches>=range2.l && distanceInInches<range2.r){
            return range2.speed;
        }
        else{
            return range3.speed;
        }
    }

}
class Range {
    double l;
    double r;
    double speed;
    Range(double l, double r, double speed) {
        this.l = l;
        this.r = r;
        this.speed = speed;
    }
}