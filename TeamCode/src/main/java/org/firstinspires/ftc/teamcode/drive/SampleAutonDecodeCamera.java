package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "SampleAutonDecodeCamera", group = "Robot")
public class SampleAutonDecodeCamera extends LinearOpMode {

    // Limelight
    private Limelight3A limelight;

    // Detected AprilTag ID
    private int detectedTag = -1; // -1 = none seen

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // -------------------- Drive Init --------------------
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(72, -24, 0);
        drive.setPoseEstimate(startPose);

        // -------------------- Limelight Init --------------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        // -------------------- AprilTag Detection (INIT LOOP) --------------------
        while (!isStarted() && !isStopRequested()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    int id = fid.getFiducialId();

                    if (id == 22 || id == 23 || id == 24) {
                        detectedTag = id;
                        break;
                    }
                }
            }

            telemetry.addData("Detected AprilTag", detectedTag);
            telemetry.update();
        }

        // -------------------- Trajectories --------------------
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(0, -24))
                .waitSeconds(1)
                .strafeTo(new Vector2d(72, -24))
                .build();

        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(0, -36))
                .waitSeconds(1)
                .strafeTo(new Vector2d(72, -36))
                .build();

        TrajectorySequence traj24 = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(0, -12))
                .waitSeconds(1)
                .strafeTo(new Vector2d(72, -12))
                .build();

        // -------------------- Start Autonomous --------------------
        runtime.reset();
        waitForStart();

        if (isStopRequested()) return;

        // -------------------- Select Trajectory --------------------
        switch (detectedTag) {
            case 22:
                drive.followTrajectorySequence(traj22);
                break;

            case 23:
                drive.followTrajectorySequence(traj23);
                break;

            case 24:
                drive.followTrajectorySequence(traj24);
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
