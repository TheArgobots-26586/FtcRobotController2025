package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

// replace with (not step) function
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

enum State {
    COLLECT,
    SHOOT,
}
enum KickerStages {
    STAGE_0,
    STAGE_1,
}

@Autonomous(name = "BlueBackAutonAsync", group = "Robot")
public class BlueBackAutonAsync extends LinearOpMode {
    private DcMotor intake = null;
    private Servo turret = null;
    private DcMotorEx shooter = null;
    private Servo kicker = null;
    private DcMotor bootKicker = null;
    private AnalogInput turretInput = null;
    private State robotState= State.SHOOT;
    double tx = 0;
    double ty = 0;
    KickerStages kickerStage = KickerStages.STAGE_0;
    double distanceInches;
    double ballsShot = 0;
    int curSequence = 0;
    boolean aborting = false;
    boolean kickerPartial = true;
    private Servo armservo = null;
    private Limelight3A limelight = null;
    private ElapsedTime kickerTimer = new ElapsedTime(), exitShootTimer = new ElapsedTime(), flickertimer = new ElapsedTime();
    private ElapsedTime abortTimer = new ElapsedTime(), ballSettleTimer = new ElapsedTime();
    private RevColorSensorV3 distanceSensor = null;

    Range range1 = new Range(0, 55, 1610);
    Range range2 = new Range(55, 73, 1860);
    Range range4 = new Range(112, Integer.MAX_VALUE, 2200);
    Range range3 = new Range(73, 112, 2115);

    public static final double SHOOTER_COLLECT_VELOCITY = 1000;
    public static final double SHOOTER_IDLE_VELOCITY = 2200;
    public static final double TURRET_LEFT_POS= 0.68;
    public static final double TURRET_RIGHT_POS = 0.88;
    public static final double TURRET_CENTER = 0.778;
    public static final double KICKER_DOWN = 0.71;//0.225
    public static final double KICKER_UP = 0.373; //0.36
    public static final double KICKER_MIDDLE = 0.53;
    public static final double ARM_SERVO_POSITION = 0.175;
    public static final double INTAKE_IDLE = -0.1;
    public static final double BOOTKICKER_IDLE = -0.1;
    public static final double INTAKE_COLLECT = -0.85;
    public static final double BOOTKICKER_COLLECT = -0.2;
    public static final double INTAKE_SHOOT = -0.6;
    public static final double BOOTKICKER_SHOOT = -0.4;
    public static final double MAX_COLOR_SENSED_DISTANCE = 7;
    public static final double MAXV = 0.47,MINV = 2.79;
    public static final double CAMERA_PITCH_DEGREES = 5.0;
    public static final double TAG_HEIGHT_INCHES = 29.5;
    public static final double CAMERA_HEIGHT_INCHES = 12.9;
    public static final double INTAKE_ABORT = 0.5;
    public static final double BOOTKICKER_ABORT = 0.5;
    public static final double ARM_ABORT = 0.2;
    public static final double DISTANCESENSORVALUE = 7;
    public static final double TURRETOFFSET = 0;

    String position = "BlueBack";

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

    void Limelight_update() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getTx() != tx) {
            int detectedID = result.getFiducialResults().get(0).getFiducialId();
            if (detectedID == 20) {
                telemetry.addData("sees april tag, this is detectedid: ", detectedID);
                tx = result.getTx();
                ty = result.getTy();
                double totalVerticalAngle = CAMERA_PITCH_DEGREES + ty;
                distanceInches = (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
                        Math.tan(Math.toRadians(totalVerticalAngle));
            }
        }
    }

    boolean Turret_align() {
        double curV = turretInput.getVoltage();
        double turret_pos = (curV-MINV) / (MAXV-MINV);
        double target = Math.min(Math.max(turret_pos + (tx/450)+TURRETOFFSET, TURRET_LEFT_POS), TURRET_RIGHT_POS);
        turret.setPosition(target);
        return (Math.abs(turret_pos-target) < 0.01);
    }

    boolean Shoot() {
        double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
        if(ballsShot == 0) {
            intake.setPower(0);
            bootKicker.setPower(0);
            armservo.setPosition(ARM_SERVO_POSITION);
        }
        else {
            intake.setPower(INTAKE_SHOOT);
            bootKicker.setPower(BOOTKICKER_SHOOT);
            armservo.setPosition(ARM_SERVO_POSITION);
        }
        shooter.setVelocity(shooterVelocity(distanceInches));
        // checks if it sees a ball and if the flywheel is up to the right speed
        boolean sees_ball = (distanceCM < DISTANCESENSORVALUE) || kickerPartial;
        if (!sees_ball) {
            ballSettleTimer.reset();
        }
        telemetry.addData("Distance Sensor value: ", kickerPartial ? distanceCM : -1);
        telemetry.addData("KickerPartial : ", kickerPartial);
        // boolean uptospeed = Math.abs(shooter.getVelocity()-shooterVelocity(distanceInches)) <= 0.03*shooterVelocity(distanceInches);
        // if (!uptospeed) {
        //    flickertimer.reset();
        //}
        switch (kickerStage) {
            case STAGE_0:
                boolean uptospeed = Math.abs(shooter.getVelocity()-shooterVelocity(distanceInches)) <= 0.03*shooterVelocity(distanceInches);
                if (!uptospeed || ballSettleTimer.milliseconds() < 100) break;
                kicker.setPosition(KICKER_UP);
                kickerPartial = false;
                kickerTimer.reset();
                kickerStage = KickerStages.STAGE_1;
            case STAGE_1:
                if (kickerTimer.milliseconds() < 200) break;
                kicker.setPosition(KICKER_DOWN);
                kickerTimer.reset();
                ballsShot++;
                kickerTimer.reset();
                kickerStage = KickerStages.STAGE_0;
        }
        telemetry.addData("Kicker Stage", kickerStage);

        // stop when no ball
        if (distanceCM > DISTANCESENSORVALUE && (kicker.getPosition() == KICKER_DOWN)) {
            armservo.setPosition(ARM_SERVO_POSITION+0.0075);
        }
        if (exitShootTimer.milliseconds() > 1000) {
            armservo.setPosition(ARM_SERVO_POSITION);
        }
        if (distanceCM <= DISTANCESENSORVALUE || (kicker.getPosition() < KICKER_DOWN)) {
            exitShootTimer.reset();
        }
        telemetry.addData("shoot mode", true);

        return (exitShootTimer.milliseconds() >= 3000);
    }


    void Collect_Init() {
        bootKicker.setPower(BOOTKICKER_COLLECT);
        intake.setPower(INTAKE_COLLECT);
        armservo.setPosition(ARM_SERVO_POSITION);
        shooter.setVelocity(SHOOTER_COLLECT_VELOCITY); //reducing speed to save power.
    }

    void Idle() {
        intake.setPower(INTAKE_IDLE);
        bootKicker.setPower(BOOTKICKER_IDLE);
        armservo.setPosition(ARM_SERVO_POSITION);
        shooter.setVelocity(SHOOTER_IDLE_VELOCITY);
    }

    void Abort() {
        intake.setPower(INTAKE_ABORT);
        bootKicker.setPower(BOOTKICKER_ABORT);
        armservo.setPosition(ARM_ABORT);
        kicker.setPosition(KICKER_DOWN);
    }

    public void runOpMode() {
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        bootKicker = hardwareMap.get(DcMotor.class, "bootkicker");
        armservo = hardwareMap.get(Servo.class, "armservo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(Servo.class, "rotator");
        turretInput = hardwareMap.get(AnalogInput.class, "turretinput");

        //LIMELIGHT SETUP
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectoryVelocityConstraint slowVel = SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint slowAccel = SampleMecanumDrive.getAccelerationConstraint(20);

        Pose2d startPose;
        if (position.equals("RedBack")) {
            startPose = new Pose2d(-50, -12, Math.toRadians(0));
        } else if (position.equals("BlueBack")) {
            startPose = new Pose2d(60, -12, Math.toRadians(180));
        } else if (position.equals("BlueFront")) {
            startPose = new Pose2d(49, 49, Math.toRadians(45));
        } else {
            startPose = new Pose2d(49, -49, Math.toRadians(-45));
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        TrajectorySequence start = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .build();
        TrajectorySequence pickBalls_1 = drive.trajectorySequenceBuilder(new Pose2d(50, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(30, -30, Math.toRadians(-90)))
                .forward(30)
                .addTemporalMarker(() -> Idle())
                .lineToLinearHeading(new Pose2d(50, -12, Math.toRadians(190)))
                .build();
        TrajectorySequence exitZone = drive.trajectorySequenceBuilder(new Pose2d(54, -12, Math.toRadians(190)))
                .lineToLinearHeading(new Pose2d(40, -15, Math.toRadians(-80)))
                .build();
        TrajectorySequence pickBalls_loadingZone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(50, -14, Math.toRadians(-90)))
                .forward(52)
                .addTemporalMarker(() -> Idle())
                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(210)))
                .build();
        telemetry.addData("Trajectory Creation Time:", timer.milliseconds());

        ArrayList<TrajectorySequence> Sequences = new ArrayList<>();
        Sequences.add(pickBalls_1);
        Sequences.add(exitZone);

        waitForStart();
        robotState = State.SHOOT;
        shooter.setVelocity(2150);
        armservo.setPosition(ARM_SERVO_POSITION);
        kicker.setPosition(KICKER_MIDDLE);
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequence(start);
        while (!isStopRequested() && opModeIsActive()) {
            Limelight_update();
            //turret.setPosition(TURRET_CENTER-(double)17/(double)450);
            if (distanceSensor.getDistance(DistanceUnit.CM) <= DISTANCESENSORVALUE && (kicker.getPosition() > KICKER_UP)) {
                kicker.setPosition(KICKER_MIDDLE);
                kickerPartial = true;
            }
            boolean aligned = true;
            aligned = Turret_align() || (ballsShot > 0);
            telemetry.addData("Is Aligned: ", aligned);
            telemetry.addData("Kicker Position", kicker.getPosition());
            telemetry.addData("Shooter Velocity: ", shooter.getVelocity());
            telemetry.addData("Balls Shot:", ballsShot);

            switch (robotState) {
                case SHOOT:
                    if (!aligned)
                        break;
                    if (ballsShot < 3 && !aborting) {
                        if (Shoot()) {
                            Abort();
                            sleep(1000);
                            aborting = true;
                        }
                        break;
                    }
                    telemetry.addData("Balls Shot: ", ballsShot);
                    telemetry.addData("Tx: ", tx);
                    ballsShot = 0;
                    robotState = State.COLLECT;
                    aborting = false;
                    Collect_Init();
                    if (curSequence < Sequences.size()) {
                        telemetry.addData("Following TrajectorySequence", curSequence);
                        drive.followTrajectorySequenceAsync(Sequences.get(curSequence));
                    }
                    curSequence++;
                    break;
                case COLLECT:
                    if(!drive.isBusy()) {
                        robotState = State.SHOOT;
                    } else {
                        drive.update();
                    }
                    break;

            }
            telemetry.update();

        }

    }
}
