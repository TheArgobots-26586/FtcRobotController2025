package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint Official Sanity Test", group = "Test")
public class pinpoint extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    // Measured from center of robot to the wheels (in mm)
    private static final double X_OFFSET_MM = -111.76;
    private static final double Y_OFFSET_MM = -127;
    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);


        pinpoint.setOffsets(X_OFFSET_MM, Y_OFFSET_MM, DistanceUnit.MM);


        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);


        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized. Sticker-Side UP.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            // Press 'A' to reset position to 0,0 for easier distance testing
            if (gamepad1.a) {
                pinpoint.resetPosAndIMU();
            }

            Pose2D pos = pinpoint.getPosition();


            telemetry.addData("Status", pinpoint.getDeviceStatus());

            telemetry.addLine("\n--- POSITION (Verify with Tape Measure) ---");
            telemetry.addData("X (Inches)", "%.2f", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y (Inches)", "%.2f", pos.getY(DistanceUnit.INCH));

            // Check Step 11: Rotate 360 and see if this says 360
            telemetry.addData("Heading (Deg)", "%.2f", pos.getHeading(AngleUnit.DEGREES));

            telemetry.addLine("\n--- HARDWARE DEBUG ---");
            telemetry.addData("X-Pod Raw Counts", pinpoint.getEncoderX());
            telemetry.addData("Y-Pod Raw Counts", pinpoint.getEncoderY());
            telemetry.addData("Update Rate (Hz)", "%.1f", pinpoint.getFrequency());

            telemetry.update();
        }
    }
}