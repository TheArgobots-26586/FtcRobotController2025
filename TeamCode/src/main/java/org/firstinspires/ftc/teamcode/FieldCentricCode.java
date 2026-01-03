package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="PinpointFieldCentricPinpoint", group="Robot")
public class FieldCentricCode extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // Pinpoint Driver
    private GoBildaPinpointDriver odo;

    boolean lastOptions = false;

    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "RightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Pinpoint
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Calibration Settings
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //old values before pinpoint
        // odo.setOffsets(-127.0, -76.2, DistanceUnit.MM);
        //these are the new values
        odo.setOffsets(-111.0, -127.2, DistanceUnit.MM);
        // odo.setOffsets(-127.0, -76.2, DistanceUnit.MM); // Using your previous offsets

        // Since you are "sticker-side up", no orientation change needed
        // but we reset to ensure we start at 0
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized - Pinpoint IMU Active");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // IMPORTANT: Must call update() to get new Gyro/Encoder data
            odo.update();

            // Reset heading on "Options" button
            boolean options = gamepad1.options;
            if (options && !lastOptions) {
                odo.resetPosAndIMU();
            }
            lastOptions = options;

            // DRIVETRAIN MATH
            double y  = -gamepad1.left_stick_y / 1.5;
            double x  =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;

            // Use Pinpoint instead of Rev IMU
            double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFront.setPower((rotY + rotX + rx) / denom);
            leftBack.setPower((rotY - rotX + rx) / denom);
            rightFront.setPower((rotY - rotX - rx) / denom);
            rightBack.setPower((rotY + rotX - rx) / denom);

            // TELEMETRY
            telemetry.addData("Heading (Deg)", Math.toDegrees(heading));
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("X-Pod Raw (Frozen?)", odo.getEncoderX());
            telemetry.update();
        }
    }
}