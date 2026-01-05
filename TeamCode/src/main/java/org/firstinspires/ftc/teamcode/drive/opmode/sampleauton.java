package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "sampleauton", group = "Robot")
public class sampleauton extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)
                .forward(50)

                .waitSeconds(1)
                .strafeLeft(30)
                .build();

        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("Status", "Initialized");
//            telemetry.addData("Start Pose", startPose.toString());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();


        drive.followTrajectorySequence(test);

        Pose2d finalPose = drive.getPoseEstimate();
        PoseStorage.currentPose = finalPose;

        while (opModeIsActive() && !isStopRequested()) {
           // displayTelemetry(drive);
//            telemetry.addData("Status", "Path Finished");
//            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

//    private void displayTelemetry(SampleMecanumDrive drive) {
//        Pose2d currentPose = drive.getPoseEstimate();
//        telemetry.addData("Current X", String.format("%.2f", currentPose.getX()));
//        telemetry.addData("Current Y", String.format("%.2f", currentPose.getY()));
//        telemetry.addData("Heading (Deg)", String.format("%.2f", Math.toDegrees(currentPose.getHeading())));
//    }
}