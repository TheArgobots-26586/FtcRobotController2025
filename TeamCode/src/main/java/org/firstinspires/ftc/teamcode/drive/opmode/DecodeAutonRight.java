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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "DecodeAutonRight", group = "Robot")
public class DecodeAutonRight extends LinearOpMode {

    // Limelight
    private Limelight3A limelight;
    private DcMotor intake = null;
    private Servo rotator = null;
    private DcMotorEx shooter = null;
    private Servo kicker = null;


    // Detected AprilTag ID
    private int detectedTag = -1; // -1 = none seen

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        rotator = hardwareMap.get(Servo.class, "rotator");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        // -------------------- Drive Init --------------------
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-12, -60, 0);
        drive.setPoseEstimate(startPose);

        //  Limelight Init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        // AprilTag Detection (INIT LOOP)
        while (!isStarted() && !isStopRequested()) {

            LLResult result = limelight.getLatestResult();
            kicker.setPosition(0.33);
            shooter.setVelocity(1600);

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    int id = fid.getFiducialId();

                    if (id == 21 || id == 22 || id == 23) {
                        detectedTag = id;
                        break;
                    }
                }
            }

            telemetry.addData("Detected AprilTag", detectedTag);
            telemetry.update();
        }

        // -------------------- Trajectories --------------------
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    shooter.setVelocity(1600);
                })
                .splineToLinearHeading(
                        new Pose2d(36, -36, Math.toRadians(90)),
                        Math.toRadians(90)
                )
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intake.setPower(-0.9);
                })
                .strafeTo(new Vector2d(48, -36))
                .addTemporalMarker(() -> {
                    intake.setPower(0);
                })
                .splineToLinearHeading(new Pose2d(12, -60, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    rotator.setPosition(0.67);//change as needed
                })
                .addTemporalMarker(() -> kicker.setPosition(0.8))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.33))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.8))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.33))
                .build();

        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    shooter.setVelocity(1600);
                })
                .splineToLinearHeading(
                        new Pose2d(36, -12, Math.toRadians(90)),
                        Math.toRadians(90)
                )
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intake.setPower(-0.9);
                })
                .strafeTo(new Vector2d(60, -12))
                .addTemporalMarker(() -> {
                    intake.setPower(0);
                })
                .splineToLinearHeading(new Pose2d(12, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(12, -60, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    rotator.setPosition(0.67);//change as needed
                })
                .addTemporalMarker(() -> kicker.setPosition(0.8))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.33))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.8))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.33))
                .build();


        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    shooter.setVelocity(1600);
                })
                .splineToLinearHeading(
                        new Pose2d(36, 12, Math.toRadians(90)),
                        Math.toRadians(90)
                )
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intake.setPower(-0.9);
                })
                .strafeTo(new Vector2d(48, 12))
                .addTemporalMarker(() -> {
                    intake.setPower(0);
                })
                .splineToLinearHeading(new Pose2d(12, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(12, -60, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    rotator.setPosition(0.67);//change as needed
                })
                .addTemporalMarker(() -> kicker.setPosition(0.8))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.33))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.8))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> kicker.setPosition(0.33))
                .build();

        // -------------------- Start Autonomous --------------------
        runtime.reset();
        waitForStart();

        if (isStopRequested()) return;

        // -------------------- Select Trajectory --------------------
        switch (detectedTag) {
            case 21:
                drive.followTrajectorySequence(traj21);
                break;

            case 22:
                drive.followTrajectorySequence(traj22);
                break;

            case 23:
                drive.followTrajectorySequence(traj23);
                break;

            default:
                // Fallback if no tag seen
                drive.followTrajectorySequence(traj22);
                break;
        }

        telemetry.addData("Auton Finished", runtime.toString());
        telemetry.update();
    }
}