package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Limelighttest", group="Robot")
public class Limelighttest extends LinearOpMode {

    private Limelight3A limelight;


    final double CAMERA_HEIGHT = 13.5;    // h1: Inches from floor to lens center
    final double TAG_HEIGHT = 30;       // h2: Inches from floor to Tag 20 center
    final double MOUNT_ANGLE = 0;      // Degrees your camera is tilted UP
    private Servo turret;
    private boolean b = false;
    private double tx;
    // ------------------------------------------

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        turret = hardwareMap.get(Servo.class,"rotator");

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            tx = result.getTx();
            if (result != null && result.isValid()) {
                // ty is the vertical offset from the crosshair to the target
                double ty = result.getTy();

                // Calculate total angle from horizon to the tag
                double totalAngleRad = Math.toRadians(MOUNT_ANGLE + ty);

                // Math: distance = (HeightDifference) / tan(totalAngle)
                // We use Math.abs to ensure distance is positive
                double heightDifference = Math.abs(TAG_HEIGHT - CAMERA_HEIGHT);
                double distanceInches = heightDifference / Math.tan(totalAngleRad);
                // Inside your while(opModeIsActive) loop...
                b = gamepad1.b;
                if (b ) {


                    gamepad1.rumble(100);

                    if (result != null && result.isValid()) {
                        int detectedID = result.getFiducialResults().get(0).getFiducialId();

                            // Use a gain to convert degrees (tx) to servo power/position
                            // Adjust 0.003 higher for faster movement, lower for smoother movement
                        double kP = 0.003;
                        double movement = tx * kP;

                        if (detectedID == 20) {
                                // Apply correction and clip to your safe range [0.75, 1]
                            double val = Math.min(Math.max(0.75, turret.getPosition() + movement), 1);
                            turret.setPosition(val);


                        } else if (detectedID == 24) {
                                // Apply correction and clip to your safe range [0.75, 1]
                            double val = Math.min(Math.max(0.75, turret.getPosition() + movement), 1);
                            turret.setPosition(val);

                        }
                        telemetry.update();
                    }

                }
            }
                    // Set state to SHOOT only once when pressed


            telemetry.update();
        }
    }
}