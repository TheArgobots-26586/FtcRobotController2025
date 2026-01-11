package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueFront_VisionReset", group = "Robot")
public class BlueFront2 extends LinearOpMode {

    private DcMotor intake = null;
    private DcMotorEx shooter = null;
    private Servo kicker = null;
    private Limelight3A limelight;
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        // --- INITIALIZATION ---
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        drive = new SampleMecanumDrive(hardwareMap);

        limelight.pipelineSwitch(0);
        limelight.start();

        Pose2d startPose = new Pose2d(45, 45, Math.toRadians(45));
        drive.setPoseEstimate(startPose);

        // --- THE TRAJECTORY ---
        TrajectorySequence fullTraj = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> shooter.setVelocity(-1090))
                .back(53)

                // --- VISION RESET POINT ---
                .waitSeconds(1.0) // Pause to let the robot settle
                .addTemporalMarker(() -> {
                    relocalizeWithLimelight();
                })

                .addTemporalMarker(() -> kicker.setPosition(0.8))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> kicker.setPosition(0.25))

                // Continue with the rest of your moves
                .back(8)
                .turn(Math.toRadians(46.5))
                .forward(58,
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10)
                )
                .back(58)
                .build();

        waitForStart();

        // Start the background telemetry thread we made earlier
        startTelemetryThread();

        drive.followTrajectorySequence(fullTraj);
    }

    /**
     * This method asks Limelight for the Botpose and updates RoadRunner
     */
    private void relocalizeWithLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            // Convert Meters to Inches
            double x = botpose.getPosition().x * 39.37;
            double y = botpose.getPosition().y * 39.37;
            // Get Heading (Limelight returns degrees, RR needs Radians)
            double heading = Math.toRadians(botpose.getOrientation().getYaw());

            // Snap RoadRunner to this new
            drive.setPoseEstimate(new Pose2d(x, y, heading));
        }
    }

    private void startTelemetryThread() {
        Thread telemetryThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                Pose2d rrPose = drive.getPoseEstimate();
                telemetry.addData("CURRENT X", rrPose.getX());
                telemetry.addData("CURRENT Y", rrPose.getY());
                telemetry.addData("CURRENT H", Math.toDegrees(rrPose.getHeading()));
                telemetry.update();
                try { Thread.sleep(50); } catch (InterruptedException e) {}
            }
        });
        telemetryThread.start();
    }
}