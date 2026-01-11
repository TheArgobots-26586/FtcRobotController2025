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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueFront", group = "Robot")
public class BlueFront extends LinearOpMode {

    private DcMotor intake = null;
    private Servo rotator = null;
    private DcMotorEx shooter = null;
    private Servo kicker = null;
    private DcMotor bootkicker = null;
    private Servo armservo = null;
    private Limelight3A limelight;

    private ElapsedTime runtime = new ElapsedTime();
    private RevColorSensorV3 distanceSensor;
    String position = "BlueFront";
    double heading;


    @Override
    public void runOpMode() {
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        intake = hardwareMap.get(DcMotor.class, "intake");
        rotator = hardwareMap.get(Servo.class, "rotator");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");
        armservo = hardwareMap.get(Servo.class, "armservo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectoryVelocityConstraint slowVel = SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint slowAccel = SampleMecanumDrive.getAccelerationConstraint(10);

        Pose2d startPose;
        if (position.equals("RedBack")) {
            startPose = new Pose2d(12, -60, Math.toRadians(180));
        } else if (position.equals("BlueBack")) {
            startPose = new Pose2d(-12, -60, Math.toRadians(90));
        } else if (position.equals("BlueFront")) {
            startPose = new Pose2d(45, 45, 45);
        } else {
            startPose = new Pose2d(-12, -60, 0);
        }
        kicker.setPosition(0.25);

        TrajectorySequence traj21BlueFront = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> shooter.setVelocity(-1090))
                .back(53)
                .waitSeconds(1)
                .addTemporalMarker(() -> kicker.setPosition(0.8))
                .waitSeconds(1)
                .addTemporalMarker(() -> kicker.setPosition(0.25))
                .waitSeconds(1)
                .addTemporalMarker(() -> bootkicker.setPower(-0.7))
                .addTemporalMarker(() -> armservo.setPosition(0.124))
                .addTemporalMarker(() -> intake.setPower(-0.9))
                .waitSeconds(1)

                .addTemporalMarker(() -> {

                    if (distanceSensor.getDistance(DistanceUnit.CM) < 7.5) {

                        kicker.setPosition(0.8);

                    }
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> kicker.setPosition(0.25))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    if (distanceSensor.getDistance(DistanceUnit.CM) < 7.5) {
                        kicker.setPosition(0.8);

                    }
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> kicker.setPosition(0.25))

//                .addTemporalMarker(() -> {
//                    heading = drive.getPoseEstimate().getHeading();
//                   // heading = Math.toRadians(heading);
//                })


                //---Next Balls---
                .addTemporalMarker(() -> {
                    LLResult result = limelight.getLatestResult();

                    if (result != null && result.isValid()) {
                        // getBotpose() returns position relative to the center of the field (0,0)
                        Pose3D botpose = result.getBotpose();

                        // Convert meters to inches (if your code uses inches)
                        double fieldX = botpose.getPosition().x * 39.37;
                        double fieldY = botpose.getPosition().y * 39.37;
                        double Aprilheading = botpose.getOrientation().getYaw();

                        telemetry.addData("Field X", fieldX); // Distance from center toward audience
                        telemetry.addData("Field Y", fieldY);
                        telemetry.addData("Pose",Math.toDegrees(Aprilheading));
                        telemetry.update();
                    }
                })
                .back(8)
                .turn(Math.toRadians(46.5))
                .addTemporalMarker(() -> armservo.setPosition(0.1385))
                .forward(58,
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10)
                )
                .addTemporalMarker(() -> shooter.setVelocity(-1100))
                .addTemporalMarker(() -> intake.setPower(0.6))
                .back(58)
                .addTemporalMarker(() -> {
                    heading = drive.getPoseEstimate().getHeading();
                })
                .turn(heading-Math.toRadians(45))

                //.back(54)




              //  .turn(Math.toRadians(-51))




                .addTemporalMarker(() -> kicker.setPosition(0.8))
                .waitSeconds(1)
                .addTemporalMarker(() -> kicker.setPosition(0.25))
                .waitSeconds(1)
                .addTemporalMarker(() -> bootkicker.setPower(-0.7))
                .addTemporalMarker(() -> intake.setPower(-0.9))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    if (distanceSensor.getDistance(DistanceUnit.CM) < 7.5) {

                        kicker.setPosition(0.8);

                    }
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> kicker.setPosition(0.25))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    if (distanceSensor.getDistance(DistanceUnit.CM) < 7.5) {
                        kicker.setPosition(0.8);

                    }
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> kicker.setPosition(0.25))





//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> kicker.setPosition(0.25))
//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> kicker.setPosition(0.8))
//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> kicker.setPosition(0.25))




                .build();


        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(startPose);

        kicker.setPosition(0.33);
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