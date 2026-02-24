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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;


@TeleOp(name="DECODE_FINAL_TELEOP3A", group="Robot")
public class DECODE_FINAL_TELEOP3A extends LinearOpMode {

/*================CONTROLS================

    A  - Toggle COLLECT mode. Press again to return to IDLE
    B - Toggle SHOOT mode Press again to return to IDLE
    X - Hold to align TURRET with APRIL TAG
    Joystick - BASIC driver controls

==========================================*/

    private static final double CAMERA_HEIGHT_INCHES = 12.9;   // camera lens height
    private static final double TAG_HEIGHT_INCHES    = 29.5;   // AprilTag center height
    private static final double CAMERA_PITCH_DEGREES = 5.0;    // camera upward tilt


    private static final double TX_DEADBAND = 4.0;
    private AnalogInput turretinput;
    double MAXV = 0.47,MINV = 2.79;
    int kickerStage = 0;
    double distanceInches = 0;

    //================CONSTANTS================//
    public static final double TURRET_RIGHT = 0.88;
    public static final double TURRET_LEFT = 0.68;
    public static final double TURRET_CENTER = 0.778;
    public static final double KICKER_DOWN = 0.73;//0.225
    public static final double KICKER_UP = 0.36;//0.6
    public static final double KICKER_MIDDLE = 0.53;
    public static final double ARM_SERVO_POSITION = 0.175;//0.043 //0.045
    public static final double INTAKE_IDLE = -0.1;
    public static final double BOOTKICKER_IDLE = -0.1;//-0.1
    public static final double INTAKE_COLLECT = -0.9;
    public static final double BOOTKICKER_COLLECT = -0.4;//-0.4
    public static final double INTAKE_ABORT = 0.5;
    public static final double BOOTKICKER_ABORT = 0.5;
    public static final double ARM_ABORT = 0.2;
    public static final double INTAKE_SHOOT = -0.2;
    public static final double BOOTKICKER_SHOOT = -0.8;
    public static final double MAX_COLOR_SENSED_DISTANCE = 7;


//ELECTRONIC SETUP

    private Limelight3A limelight;
    private DcMotorEx intake;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    private Servo kicker;
    private DcMotorEx bootkicker;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime intakeRuntime = new ElapsedTime();
    private RevColorSensorV3 distanceSensor;
    private PinpointLocalizer pinpointLocalizer;

    Range range1 = new Range(0, 55, 1610);//1100
    Range range2 = new Range(55, 73, 1860);//1200
    //Range range3 = new Range(73,85,)

    Range range3 = new Range(112, Integer.MAX_VALUE, 2140);//1430
    Range range4 = new Range(73, 112, 2115);//1410


    //STATE MACHINE SETUP
    enum RobotState {
        IDLE,
        COLLECT,
        SHOOT,
        ABORT
    }
    enum kickerState{
        Stage1,
        Stage2,
        Stage3,
        Stage4
    }
    RobotState currentState = RobotState.IDLE;

    boolean lastA = false;
    boolean lastB = false;

    // Latch for auto-fire
    boolean lastOptions = false;
    private Servo turret = null;
    public Servo armservo = null;
    double tx=0;
    double ty=0;
    int current_kicker_stage = 0;
    boolean kickerPartial = false;
    double newPos;
    boolean gamepad1LastA = false;
    boolean aPressed = false;
    boolean bumperPressed = false;
    int ballsShot = 0;

    @Override
    public void runOpMode() {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        //HARDWARE MAP
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        leftFront  = hardwareMap.get(DcMotorEx.class, "LeftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        rightBack  = hardwareMap.get(DcMotorEx.class, "RightBack");

        intake     = hardwareMap.get(DcMotorEx.class, "intake");
        kicker     = hardwareMap.get(Servo.class, "kicker");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");
        bootkicker = hardwareMap.get(DcMotorEx.class, "bootkicker");
        armservo = hardwareMap.get(Servo.class, "armservo");
        turret = hardwareMap.get(Servo.class, "rotator");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretinput = hardwareMap.get(AnalogInput.class,"turretinput");

        //PID Coefficients
        PIDFCoefficients shooterCoefficients = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        double p = shooterCoefficients.p;
        double i = shooterCoefficients.i;
        double d = shooterCoefficients.d;
        double f = shooterCoefficients.f;
        PIDFCoefficients pidfNew = new PIDFCoefficients(14, i, d, f);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        telemetry.addData("p: ",p);//10
        telemetry.addData("i: ",i);//3
        telemetry.addData("d: ",d);//0
        telemetry.addData("f: ",f);//0
        telemetry.update();

        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(KICKER_DOWN);
        armservo.setPosition(ARM_SERVO_POSITION);
        turret.setPosition(TURRET_CENTER);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinpointLocalizer = new PinpointLocalizer(hardwareMap);
        armservo.setPosition(ARM_SERVO_POSITION);
        double offset;

        waitForStart();

        while (opModeIsActive()) {
           // armservo.setPosition(ARM_SERVO_POSITION);
            pinpointLocalizer.update();
            boolean options = gamepad1.options;
            if (options && !lastOptions) {
                pinpointLocalizer.resetHeading();
            }
            lastOptions = options;

            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);

            // DRIVETRAIN
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;
            double heading = pinpointLocalizer.getHeading();
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);



            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFront.setPower((rotY + rotX + rx) / denom);
            leftBack.setPower((rotY - rotX + rx) / denom);
            rightFront.setPower((rotY - rotX - rx) / denom);
            rightBack.setPower((rotY + rotX - rx) / denom);

            // APRIL TAG DETECTION
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
                // Update the existing distanceInches variable (no 'double' prefix)
                distanceInches = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
                        Math.tan(Math.toRadians(totalVerticalAngle));
                //telemetry.addData("Tag Seen", true);
            }

            gamepad1LastA = aPressed;
            // A(Toggle for IDLE and COLLECT) B(Toggle for SHOOT and IDLE)
            boolean a = gamepad2.a;
            boolean b = gamepad2.b;
            boolean xPressed = gamepad2.x;

            if (result != null && result.isValid() && result.getTx() != tx) {
                int detectedID = result.getFiducialResults().get(0).getFiducialId();
                telemetry.addData("sees april tag, this is detectedid: ", detectedID);
                tx = result.getTx();
                double val = 0;
                if (detectedID == 20) {
                    val = Math.min(Math.max(TURRET_LEFT, turrpos + (tx / 360)-0.005), TURRET_RIGHT);
                    turret.setPosition(val);
                    telemetry.addData("servo target pos blue", val );
                } else if (detectedID == 24) {
                    val = Math.min(Math.max(TURRET_LEFT, turrpos + (tx / 360)+0.005), TURRET_RIGHT);
                    turret.setPosition(val);
                    telemetry.addData("servo target pos red", val );
                }
//                double angle = (val - TURRET_CENTER) * 360;
//                offset = Math.asin((12 * Math.sqrt(2) * Math.sin(Math.toRadians(angle-45)))/distanceInches);
//                offset = Math.toDegrees(offset)/360;
//                turret.setPosition(val + offset);
//                telemetry.addData("offset: ", offset);
            }


            if (a && !lastA) {
                if (currentState == RobotState.COLLECT) {
                    currentState = RobotState.IDLE;
                    ballsShot = 0;
                } else {
                    currentState = RobotState.COLLECT;
                    ballsShot = 0;
                }
            }
            if (b && !lastB) {
                if (currentState == RobotState.SHOOT) {
                    ballsShot = 0;
                    currentState = RobotState.IDLE;
                } else {
                    currentState = RobotState.SHOOT;
                }
            }
            if(xPressed) {
                currentState = RobotState.ABORT;
            }
            if (gamepad1.right_bumper && !bumperPressed){
                turret.setPosition(TURRET_CENTER);
                bumperPressed = true;
            }
            else {
                bumperPressed = false;
            }

            lastA = a;
            lastB = b;
            // STATE MACHINE
            switch (currentState) {
                case IDLE:
                    shooter.setVelocity(shooterVelocity(distanceInches));
                    intake.setPower(INTAKE_IDLE);
                    bootkicker.setPower(BOOTKICKER_IDLE);
                    armservo.setPosition(ARM_SERVO_POSITION);
                    break;
                case COLLECT:
                    intakeRuntime.reset();
                    if (distanceCM<7){
                        kicker.setPosition(KICKER_MIDDLE);
                        kickerPartial = true;
                    }
                    //logic 1
//                    if (runtime.milliseconds()>500){
//                        armservo.setPosition(0.085 - armservo.getPosition());
//                        runtime.reset();
//                    }
                    //logic 2
//                    if (intakeRuntime.milliseconds()%1000< 500){
//                        armservo.setPosition(0.043);
//                    }
//                    else {
//                        armservo.setPosition(0.042);
//                    }

                    shooter.setVelocity(shooterVelocity(distanceInches));
                    bootkicker.setPower(BOOTKICKER_COLLECT);
                    intake.setPower(INTAKE_COLLECT);
                    armservo.setPosition(ARM_SERVO_POSITION);
                    break;
                case SHOOT:
                    if(ballsShot == 0) {
                        intake.setPower(0);
                        bootkicker.setPower(0);
                    }
                    else {
                        intake.setPower(INTAKE_COLLECT);
                        bootkicker.setPower(BOOTKICKER_COLLECT);
                    }
                    shooter.setVelocity(shooterVelocity(distanceInches));
                    if ((distanceCM < 7 && kickerStage == 0) || kickerPartial) {
                        runtime.reset();
                        kickerStage = 1;
                        kickerPartial = false;
                    }
// delay 100ms
                    if (kickerStage == 1 && runtime.milliseconds() >= 100) {
                        kicker.setPosition(KICKER_UP);
                        runtime.reset();
                        kickerStage = 2;
                    }
// delay 400ms
                    if (kickerStage == 2 && runtime.milliseconds() >= 400) {//500
                        kicker.setPosition(KICKER_DOWN);
                        runtime.reset();
                        kickerStage = 3;
                    }
// delay 200ms
                    if (kickerStage == 3 && runtime.milliseconds() >= 200) {
                        telemetry.addData("kicker shoot", true);
                        ballsShot++;
                        kickerStage = 0;
                    }
                    // stop when no ball
                    if (distanceCM >= 7) {
                        armservo.setPosition(ARM_SERVO_POSITION+0.005);
                    }
                    telemetry.addData("shoot mode", true);
                    //telemetry.update();
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
            //  telemetry.addData("Current State is:", currentState);
            telemetry.addData("Motor Power:", shooter.getVelocity());
            telemetry.addData("Intake Power:", intake.getPower());
            telemetry.addData("x:", pinpointLocalizer.getPoseEstimate().getX());
            telemetry.addData("y:", pinpointLocalizer.getPoseEstimate().getY());
            telemetry.addData("heading:", pinpointLocalizer.getHeading());
            telemetry.addData("Distance From Goal:", distanceInches);
            telemetry.addData("tx (deg)", result.getTx());
            telemetry.addData("ty (deg)", ty);
            telemetry.addData("servo current pos", turret.getPosition());
            telemetry.addData("turret exact pose", turrpos);
            telemetry.addData("kicker pos",kicker.getPosition());
            dashboardTelemetry.addData("Boot Kicker Power",bootkicker.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("Intake Power",intake.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("LeftFront Power",leftFront.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("LeftBack Power",leftBack.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("RightFront Power",rightFront.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("RightBack Power",rightBack.getCurrent(CurrentUnit.AMPS));

            dashboardTelemetry.addData("Shooter Current",shooter.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("Shooter Velocity",shooter.getVelocity());
            telemetry.update();
            dashboardTelemetry.update();
        }
    }

    public double shooterVelocity(double distanceInInches) {
        if (distanceInInches>=range1.l && distanceInInches<range1.r){
            telemetry.addData("range1", range1.speed);
            return range1.speed;
        }
        else if (distanceInInches>=range2.l && distanceInInches<range2.r){
            telemetry.addData("range2", range2.speed);
            return range2.speed;
        }
        else if(distanceInInches>=range3.l && distanceInInches<range3.r) {
            telemetry.addData("range3", range3.speed);
            return range3.speed;
        }
        else{
            telemetry.addData("range4", range4.speed);
            return range4.speed;
        }
    }
}