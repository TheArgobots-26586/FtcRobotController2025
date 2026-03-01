package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;


@TeleOp(name="PinpointFieldCentricPinpoint", group="Robot")
public class SampleFieldCentricPinpoint extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // Pinpoint Driver
    private GoBildaPinpointDriver odo;

    boolean lastOptions = false;
    private PinpointLocalizer pinpointLocalizer;

    @Override
    public void runOpMode() {

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
       // distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        leftFront  = hardwareMap.get(DcMotorEx.class, "LeftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        rightBack  = hardwareMap.get(DcMotorEx.class, "RightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        pinpointLocalizer = new PinpointLocalizer(hardwareMap);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Calibration Settings
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //old values before pinpoint
       // odo.setOffsets(-127.0, -76.2, DistanceUnit.MM);
        //these are the new values
        odo.setOffsets(-111.0, -127.2, DistanceUnit.MM);

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized - Pinpoint IMU Active");
        telemetry.update();
        pinpointLocalizer = new PinpointLocalizer(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            pinpointLocalizer.update();
            boolean options = gamepad1.options;
            if (options && !lastOptions) {
                pinpointLocalizer.resetHeading();
            }
            lastOptions = options;

            double y  = -gamepad1.left_stick_y / 1.5;
            double x  =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;
            double heading = pinpointLocalizer.getHeading();
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFront.setPower((rotY + rotX + rx) / denom);
            leftBack.setPower((rotY - rotX + rx) / denom);
            rightFront.setPower((rotY - rotX - rx) / denom);
            rightBack.setPower((rotY + rotX - rx) / denom);


            // TELEMETRY
            dashboardTelemetry.addData("RightBack Power",rightBack.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("RightFront Power",rightFront.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("LeftFront Power",leftFront.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("LeftBack Power",leftBack.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Heading (Deg)", Math.toDegrees(heading));
            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.update();
        }
    }
}