package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueFront2", group = "Robot")
public class BlueFront2 extends LinearOpMode {

    private DcMotor intake, bootkicker;
    private Servo rotator, kicker, armservo;
    private DcMotorEx shooter;
    private ElapsedTime runtime = new ElapsedTime();

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

        Pose2d startPose = new Pose2d(54, 54, Math.toRadians(45));

        drive.resetPinPoint(startPose);//correction added

        TrajectorySequence mainTraj = drive.trajectorySequenceBuilder(startPose)
                .back(20)
                .addTemporalMarker(() -> shooter.setVelocity(-1200))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    kicker.setPosition(0.8);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    kicker.setPosition(0.33);
                })
                .build();

        // --- 4. INITIAL SETUP ---
        kicker.setPosition(0.33);
        bootkicker.setPower(-0.4);

        telemetry.addData("Status", "Pinpoint Synced at 45 Degrees");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();


        drive.followTrajectorySequence(mainTraj);

        // Final Position Telemetry
        Pose2d finalPose = drive.getPoseEstimate();
        telemetry.addData("Final X", finalPose.getX());
        telemetry.addData("Final Y", finalPose.getY());
        telemetry.addData("Final Heading", Math.toDegrees(finalPose.getHeading()));
        telemetry.update();

        // Keep OpMode alive to read telemetry
        while (opModeIsActive() && !isStopRequested());
    }
}