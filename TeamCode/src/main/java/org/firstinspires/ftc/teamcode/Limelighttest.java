package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Limelighttest", group="Robot")
public class Limelighttest extends LinearOpMode {

    private Limelight3A limelight;


    final double CAMERA_HEIGHT = 13.5;    // h1: Inches from floor to lens center
    final double TAG_HEIGHT = 30;       // h2: Inches from floor to Tag 20 center
    final double MOUNT_ANGLE = 0;      // Degrees your camera is tilted UP
    // ------------------------------------------

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // ty is the vertical offset from the crosshair to the target
                double ty = result.getTy();

                // Calculate total angle from horizon to the tag
                double totalAngleRad = Math.toRadians(MOUNT_ANGLE + ty);

                // Math: distance = (HeightDifference) / tan(totalAngle)
                // We use Math.abs to ensure distance is positive
                double heightDifference = Math.abs(TAG_HEIGHT - CAMERA_HEIGHT);
                double distanceInches = heightDifference / Math.tan(totalAngleRad);

                telemetry.addData("Vertical Offset (ty)", "%.2f°", ty);
                telemetry.addData("Horizontal Distance", "%.2f in", distanceInches);
            } else {
                telemetry.addLine("No Tag Detected");
            }
            telemetry.update();
        }
    }
}