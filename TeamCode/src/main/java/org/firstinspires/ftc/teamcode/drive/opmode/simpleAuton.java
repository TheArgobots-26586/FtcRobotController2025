package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "simpleAuton", group = "Robot")
public class simpleAuton extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor intake = null;
    private Servo rotator = null;
    private DcMotorEx shooter = null;
    private Servo kicker = null;
    private DcMotor bootkicker = null;
    private Servo armservo = null;

    private int detectedTag = -1;
    private ElapsedTime runtime = new ElapsedTime();

    // Set your starting position here
    String position = "BlueBack";

    @Override
    public void runOpMode() {
        // --- Hardware Mapping ---
        intake = hardwareMap.get(DcMotor.class, "intake");
        rotator = hardwareMap.get(Servo.class, "rotator");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");
        armservo = hardwareMap.get(Servo.class, "armservo");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // --- Start Pose Logic (Fixed duplicated IF) ---
        Pose2d startPose;
        if (position.equals("RedBack")) {
            startPose = new Pose2d(12, -60, Math.toRadians(180));
        } else if (position.equals("BlueBack")) {
            startPose = new Pose2d(-12, -60, Math.toRadians(90));
        } else {
            startPose = new Pose2d(-12, -60, 0); // Default fallback
        }
        drive.setPoseEstimate(startPose);

        // --- Limelight Init ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // --- Detection Loop (Before Start) ---
        while (!isStarted() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    int id = fid.getFiducialId();
                    if (id == 21 || id == 22 || id == 23) {
                        detectedTag = id;
                        break;
                    }
                }
            }
            telemetry.addData("Status", "INITIALIZED - PLACE ROBOT AT " + startPose.toString());
            telemetry.addData("Detected AprilTag", detectedTag);
            telemetry.addData("Selected Position", position);
            telemetry.update();
        }

        // --- Build Trajectories ---
        TrajectorySequence traj21BlueBack = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> shooter.setVelocity(1600))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0.25)
                .strafeTo(new Vector2d(-55, -36))
                .lineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(0)))
                .addTemporalMarker(() -> kicker.setPosition(0.6))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.25))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.6))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.25))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.6))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.25))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();

        // --- Initial Actuator Setup ---
        kicker.setPosition(0.33);
        bootkicker.setPower(-0.4);
        intake.setPower(-0.9);
        armservo.setPosition(0.1375);

        // --- Execution with LIVE position printing ---
        if (position.equals("BlueBack")) {
            // Use Async so we can update telemetry in the loop below
            drive.followTrajectorySequenceAsync(traj21BlueBack);
        }

        // This loop runs WHILE the robot is moving
        while (opModeIsActive() && drive.isBusy()) {
            drive.update(); // Tells RoadRunner to move the motors
            printLiveTelemetry(drive);
        }

        // --- Finalize & Pose Storage ---
        Pose2d finalPose = drive.getPoseEstimate();
        PoseStorage.currentPose = finalPose;

        // Keep printing final position until the OpMode is stopped
        while (opModeIsActive()) {
            telemetry.addData("STATUS", "AUTON COMPLETE");
            printLiveTelemetry(drive);
        }
    }

    private void printLiveTelemetry(SampleMecanumDrive drive) {
        Pose2d pose = drive.getPoseEstimate();
        telemetry.addData("Current X", String.format("%.2f", pose.getX()));
        telemetry.addData("Current Y", String.format("%.2f", pose.getY()));
        telemetry.addData("Heading", String.format("%.2f deg", Math.toDegrees(pose.getHeading())));
        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }
}