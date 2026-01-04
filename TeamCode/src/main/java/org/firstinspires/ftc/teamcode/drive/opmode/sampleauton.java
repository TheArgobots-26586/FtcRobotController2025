package org.firstinspires.ftc.teamcode.drive.opmode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
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

@Autonomous(name = "sampleauton", group = "Robot")
public class sampleauton extends LinearOpMode {

    // Limelight


    private ElapsedTime runtime = new ElapsedTime();
     // BlueBack, BlueFront, RedBack, RedFront

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-12, -60, 0);
        drive.setPoseEstimate(startPose);

        //  Limelight Init




        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-36, -36))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-55, -36))
                .build();



        runtime.reset();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(test);



        telemetry.addData("Auton Finished", runtime.toString());
        telemetry.update();
    }
}