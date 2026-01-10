package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    private ElapsedTime runtime = new ElapsedTime();
    private RevColorSensorV3 distanceSensor;
    String position = "BlueFront";

    @Override
    public void runOpMode() {
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        intake = hardwareMap.get(DcMotor.class, "intake");
        rotator = hardwareMap.get(Servo.class, "rotator");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");
        armservo = hardwareMap.get(Servo.class, "armservo");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        TrajectorySequence traj21BlueFront = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> shooter.setVelocity(-1100))
                .back(53)
                .waitSeconds(1)
                .addTemporalMarker(() -> kicker.setPosition(0.8))
                .waitSeconds(1)
                .addTemporalMarker(() -> kicker.setPosition(0.25))
                .waitSeconds(1)
                .addTemporalMarker(() -> bootkicker.setPower(-0.7))
                .addTemporalMarker(() -> intake.setPower(-0.7))
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