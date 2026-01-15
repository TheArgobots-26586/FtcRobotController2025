package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(name="LimelightLocation", group="Robot")
public class LimelightLocation extends LinearOpMode {

    Limelight3A limelight;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Pipeline configured in Limelight UI
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Limelight initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() && result.getBotpose() != null) {

                Pose3D botPose = result.getBotpose();

                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;
                x = x*39.37 ;
                y = y*39.37 ;

                double headingDeg =
                        botPose.getOrientation().getYaw(AngleUnit.DEGREES);

                telemetry.addData("X (in)", x);
                telemetry.addData("Y (in)", y);
                telemetry.addData("Heading (deg)", headingDeg);
                telemetry.addData("Tags Seen", result.getFiducialResults().size());

            } else {
                telemetry.addLine("No valid AprilTag pose");
            }

            telemetry.update();
        }
    }
}