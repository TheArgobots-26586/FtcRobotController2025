package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DistanceCheck", group = "TeleOp")
public class DistanceCheck extends LinearOpMode {

    // ================== CAMERA / TAG SETUP ==================
    private static final double CAMERA_HEIGHT_INCHES = 12.9;   // camera lens height
    private static final double TAG_HEIGHT_INCHES    = 29.5;   // AprilTag center height
    private static final double CAMERA_PITCH_DEGREES = 5.0;    // camera upward tilt

    // ================== HARDWARE ==================
    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // ----------- Hardware Map -----------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ----------- Limelight Setup -----------
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addLine("No AprilTag detected");
                telemetry.update();
                continue;
            }

            double tx = result.getTx(); // horizontal offset (deg)
            double ty = result.getTy(); // vertical offset (deg)

            double totalVerticalAngle =
                    CAMERA_PITCH_DEGREES + ty;
            double distanceInches =
                    (TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
                            Math.tan(Math.toRadians(totalVerticalAngle));
            telemetry.addData("Tag Seen", true);
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("ty (deg)", ty);
            telemetry.addData("Distance (in)", "%.1f", distanceInches);
            telemetry.update();
        }
    }
}

// actual distance: 119, limelight: 111, shooter power: 1415 - 100% accurate
// actual distance: 125, limelight: 116, shooter power: 1415 - 60% accurate : 1430