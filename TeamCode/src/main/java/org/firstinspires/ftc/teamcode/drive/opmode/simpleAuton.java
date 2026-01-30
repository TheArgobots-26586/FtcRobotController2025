package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "simpleAuton", group = "Robot")
public class simpleAuton extends LinearOpMode {

    private DcMotor intake = null;
   // private Servo rotator = null;
    private DcMotorEx shooter = null;
    private Servo kicker = null;
    private DcMotor bootkicker = null;
    double tx = 0;
    double ty = 0;
    private Servo armservo = null;
    private Limelight3A limelight;

    private ElapsedTime runtime = new ElapsedTime();
    private RevColorSensorV3 distanceSensor;
    public static final double KICKER_DOWN = 0.225;
    public static final double KICKER_UP = 0.6;
    public static final double ARM_SERVO_POSITION = 0.24;
    public static final double INTAKE_IDLE = -0.1;
    public static final double BOOTKICKER_IDLE = -0.1;
    public static final double INTAKE_COLLECT = -0.9;
    public static final double BOOTKICKER_COLLECT = -0.4;
    public static final double INTAKE_SHOOT = -0.2;
    public static final double BOOTKICKER_SHOOT = -0.2;
    public static final double MAX_COLOR_SENSED_DISTANCE = 7;
    String position = "BlueBack";
    double heading;
    private Servo turret;


    @Override
    public void runOpMode() {
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");
        armservo = hardwareMap.get(Servo.class, "armservo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(Servo.class, "rotator");


        //LIMELIGHT SETUP
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();


        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectoryVelocityConstraint slowVel = SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint slowAccel = SampleMecanumDrive.getAccelerationConstraint(10);

        Pose2d startPose;
        if (position.equals("RedBack")) {
            startPose = new Pose2d(-60, -12, Math.toRadians(0));
        } else if (position.equals("BlueBack")) {
            startPose = new Pose2d(72, -12, Math.toRadians(180));
        } else if (position.equals("BlueFront")) {
            startPose = new Pose2d(49, 49, Math.toRadians(45));
        } else {
            startPose = new Pose2d(49, -49, Math.toRadians(-45));
        }
        kicker.setPosition(0.225);
        TrajectorySequence traj21BlueFront = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(15,12))
                .build();


//        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(startPose);

        kicker.setPosition(0.225);
        //  bootkicker.setPower(-0.6);

        runtime.reset();
        drive.followTrajectorySequence(traj21BlueFront);

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Pose Resets", drive.getNumSetPosCalls());
            telemetry.addData("numinstances", PinpointLocalizer.num_instances);
            telemetry.update();
        }
    }
}