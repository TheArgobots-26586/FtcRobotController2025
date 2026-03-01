package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;


@TeleOp(name="DECODE_FINAL_TELEOP3SWITCH", group="Robot")
public class DECODE_FINAL_TELEOP3SWITCH extends LinearOpMode {



    private static final double CAMERA_HEIGHT_INCHES = 12.9;
    private static final double TAG_HEIGHT_INCHES    = 29.5;
    private static final double CAMERA_PITCH_DEGREES = 5.0;
    private static final double SERVO_CENTER = 0.9;
    private static final double SERVO_MIN = 0.8;
    private static final double SERVO_MAX = 1;
    private static final double SERVO_GAIN = 0.005;
    private static final double TX_DEADBAND = 4.0;

    private AnalogInput turretinput;
    double MAXV = 0.47, MINV = 2.79;
    int kickerStage = 0;
    double distanceInches = 0;

    //================CONSTANTS================//
    public static final double KICKER_DOWN = 0.9;
    public static final double KICKER_UP = 0.53;
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

    private Limelight3A limelight;
    private DcMotorEx intake;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    private Servo kicker;
    private DcMotorEx bootkicker;

    private ElapsedTime runtime = new ElapsedTime();
    private RevColorSensorV3 distanceSensor;
    private PinpointLocalizer pinpointLocalizer;

    Range range1 = new Range(0, 55, 1100);
    Range range2 = new Range(55, 80, 1200);
    Range range4 = new Range(112, Integer.MAX_VALUE, 1430);
    Range range3 = new Range(80, 112, 1410);

    enum RobotState { IDLE, COLLECT, SHOOT, ABORT }
    RobotState currentState = RobotState.IDLE;

    boolean lastA = false;
    boolean lastB = false;
    boolean lastOptions = false;
    private Servo turret = null;
    public Servo armservo = null;
    double tx1=0;
    double ty=0;
    boolean gamepad1LastA = false;
    boolean aPressed = false;

    @Override
    public void runOpMode() {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        leftFront  = hardwareMap.get(DcMotorEx.class, "LeftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        rightBack  = hardwareMap.get(DcMotorEx.class, "RightBack");

        intake     = hardwareMap.get(DcMotorEx.class, "intake");
        kicker     = hardwareMap.get(Servo.class, "kicker");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");
        bootkicker = hardwareMap.get(DcMotorEx.class, "bootkicker");
        armservo   = hardwareMap.get(Servo.class, "armservo");
        turret     = hardwareMap.get(Servo.class, "rotator");
        limelight  = hardwareMap.get(Limelight3A.class, "limelight");
        turretinput = hardwareMap.get(AnalogInput.class,"turretinput");

        limelight.pipelineSwitch(0);
        limelight.start();

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(KICKER_DOWN);
        armservo.setPosition(ARM_SERVO_POSITION);
        turret.setPosition(0.778);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinpointLocalizer = new PinpointLocalizer(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            armservo.setPosition(ARM_SERVO_POSITION);
            pinpointLocalizer.update();

            if (gamepad1.options && !lastOptions) {
                pinpointLocalizer.resetHeading();
            }
            lastOptions = gamepad1.options;

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

            // VISION
            double curV = turretinput.getVoltage();
            double turrpos = (curV-MINV) / (MAXV-MINV);
            LLResult result = limelight.getLatestResult();
            aPressed = gamepad1.a;

            if (result != null && result.isValid()) {
                if(aPressed && !gamepad1LastA) {
                    gamepad1.rumble(100);
                }
                ty = result.getTy();
                double totalVerticalAngle = CAMERA_PITCH_DEGREES + ty;
                distanceInches = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(Math.toRadians(totalVerticalAngle));
            }
            gamepad1LastA = aPressed;

            if (result != null && result.isValid() && result.getTx() != tx1) {
                int detectedID = result.getFiducialResults().get(0).getFiducialId();
                tx1 = result.getTx();
                if (detectedID == 20) {
                    double val = Math.min(Math.max(0.68, turrpos + (tx1 / 280)-0.005), 0.88);
                    turret.setPosition(val);
                } else if (detectedID == 24) {
                    double val = Math.min(Math.max(0.68, turrpos + (tx1 / 280)+0.005), 0.88);
                    turret.setPosition(val);
                }
            }

            // STATE TOGGLES
            if (gamepad2.a && !lastA) {
                currentState = (currentState == RobotState.COLLECT) ? RobotState.IDLE : RobotState.COLLECT;
            }
            if (gamepad2.b && !lastB) {
                if (currentState == RobotState.SHOOT) {
                    currentState = RobotState.IDLE;
                } else {
                    currentState = RobotState.SHOOT;
                    kickerStage = 0;
                }
            }
            if(gamepad2.x) currentState = RobotState.ABORT;

            lastA = gamepad2.a;
            lastB = gamepad2.b;

            // STATE MACHINE
            switch (currentState) {
                case IDLE:
                    shooter.setVelocity(shooterVelocity(distanceInches));
                    intake.setPower(INTAKE_IDLE);
                    bootkicker.setPower(BOOTKICKER_IDLE);
                    kicker.setPosition(KICKER_DOWN);
                    armservo.setPosition(0.24);
                    break;

                case COLLECT:
                    shooter.setVelocity(shooterVelocity(distanceInches));
                    kicker.setPosition(KICKER_DOWN);
                    bootkicker.setPower(BOOTKICKER_COLLECT);
                    intake.setPower(INTAKE_COLLECT);
                    armservo.setPosition(0.24);
                    break;

                case SHOOT:
                    shooter.setVelocity(shooterVelocity(distanceInches));


                    switch (kickerStage) {
                        case 0:
                            if (distanceCM < 7) {
                                runtime.reset();
                                kickerStage = 1;
                            }
                            break;
                        case 1:
                            if (runtime.milliseconds() >= 100) {
                                kicker.setPosition(KICKER_UP);
                                runtime.reset();
                                kickerStage = 2;
                            }
                            break;
                        case 2:
                            if (runtime.milliseconds() >= 500) {
                                kicker.setPosition(KICKER_DOWN);
                                runtime.reset();
                                kickerStage = 3;
                            }
                            break;
                        case 3:
                            if (runtime.milliseconds() >= 200) {
                                telemetry.addData("kicker shoot", true);
                                kickerStage = 0;
                            }
                            break;
                    }

                    // stop when no ball
                    if (distanceCM >= 7 && kickerStage == 0) {
                        kicker.setPosition(KICKER_DOWN);
                        armservo.setPosition(0.25);
                    }

                    intake.setPower(INTAKE_COLLECT);
                    bootkicker.setPower(BOOTKICKER_COLLECT);
                    telemetry.addData("shoot mode", true);
                    break;

                case ABORT:
                    intake.setPower(INTAKE_ABORT);
                    bootkicker.setPower(BOOTKICKER_ABORT);
                    armservo.setPosition(ARM_ABORT);
                    kicker.setPosition(KICKER_DOWN);
                    break;
            }

            // Telemetry
            telemetry.addData("MODE", currentState);
            telemetry.addData("Kicker Stage", kickerStage);
            telemetry.addData("Distance (cm)", distanceCM);
            telemetry.addData("Motor Velocity:", shooter.getVelocity());
            telemetry.addData("Distance From Goal:", distanceInches);
            dashboardTelemetry.addData("Intake Amps", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }

    public double shooterVelocity(double distanceInInches) {
        if (distanceInInches >= range1.l && distanceInInches < range1.r) return range1.speed;
        else if (distanceInInches >= range2.l && distanceInInches < range2.r) return range2.speed;
        else if (distanceInInches >= range3.l && distanceInInches < range3.r) return range3.speed;
        else return range4.speed;
    }

    static class Range {
        double l, r, speed;
        Range(double l, double r, double speed) {
            this.l = l;
            this.r = r;
            this.speed = speed;
        }
    }
}