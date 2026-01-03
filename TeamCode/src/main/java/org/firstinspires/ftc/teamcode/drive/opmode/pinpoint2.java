package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
@TeleOp(name = "Pinpoint Field Position (Inches)", group = "Odometry")
public class pinpoint2 extends LinearOpMode {

    private I2cDeviceSynch pinpoint;

    // ===== OFFSETS (mm) — PROVIDED BY YOU =====
    private static final double X_OFFSET_MM = 111.76;  // left +, right -
    private static final double Y_OFFSET_MM = -127 ;   // forward +, backward -

    private static final double MM_TO_INCH = 1.0 / 25.4;

    // Field position stored in mm
    private double fieldX_mm = 0.0;
    private double fieldY_mm = 0.0;

    private double lastHeadingRad = 0.0;

    @Override
    public void runOpMode() {

        pinpoint = hardwareMap.get(I2cDeviceSynch.class, "odo");
        pinpoint.setI2cAddress(I2cAddr.create8bit(0x28));
        pinpoint.engage();

        telemetry.addLine("Pinpoint Initialized");
        telemetry.addLine("Start position set to (0, 0)");
        telemetry.update();

        waitForStart();

        // Force reset on start
        fieldX_mm = 0.0;
        fieldY_mm = 0.0;
        lastHeadingRad = 0.0;

        while (opModeIsActive()) {

            // Read Pinpoint data
            // [0–3]  = ΔX (mm, robot frame)
            // [4–7]  = ΔY (mm, robot frame)
            // [8–11] = Heading (degrees)
            byte[] data = pinpoint.read(0x04, 12);

            double dX_robot = bytesToFloat(data, 0);
            double dY_robot = bytesToFloat(data, 4);
            double headingRad = Math.toRadians(bytesToFloat(data, 8));

            double dTheta = headingRad - lastHeadingRad;
            lastHeadingRad = headingRad;

            // ===== OFFSET CORRECTION =====
            double correctedDX = dX_robot - dTheta * Y_OFFSET_MM;
            double correctedDY = dY_robot + dTheta * X_OFFSET_MM;

            // ===== ROBOT → FIELD TRANSFORM =====
            double cos = Math.cos(headingRad);
            double sin = Math.sin(headingRad);

            fieldX_mm += correctedDX * cos - correctedDY * sin;
            fieldY_mm += correctedDX * sin + correctedDY * cos;

            // Convert to inches for display
            double fieldX_in = fieldX_mm * MM_TO_INCH;
            double fieldY_in = fieldY_mm * MM_TO_INCH;

            telemetry.addData("X (in)", "%.2f", fieldX_in);
            telemetry.addData("Y (in)", "%.2f", fieldY_in);
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(headingRad));
            telemetry.update();
        }
    }

    private float bytesToFloat(byte[] buffer, int offset) {
        int bits =
                (buffer[offset] & 0xFF) |
                        ((buffer[offset + 1] & 0xFF) << 8) |
                        ((buffer[offset + 2] & 0xFF) << 16) |
                        ((buffer[offset + 3] & 0xFF) << 24);
        return Float.intBitsToFloat(bits);
    }
}