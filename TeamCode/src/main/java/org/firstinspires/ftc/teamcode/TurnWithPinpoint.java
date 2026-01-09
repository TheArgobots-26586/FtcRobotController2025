package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TurnWithPinpoint", group="Robot")
public class TurnWithPinpoint extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private GoBildaPinpointDriver odo;


    double Kp = 0.8;

    boolean lastOptions = false;
    double xgoal = 132;//60
    double ygoal = 60;//36

    @Override
    public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "RightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(-111.0, -127.2, DistanceUnit.MM);
        odo.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            double xPos = odo.getPosX(DistanceUnit.INCH);
            double yPos = odo.getPosY(DistanceUnit.INCH);
            double heading = odo.getHeading(AngleUnit.RADIANS); // Radians

            // Calculate manual inputs
            double y  = -gamepad1.left_stick_y / 1.5;
            double x  =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;


            if (gamepad1.a) {

                double targetAngle = Math.atan2(ygoal - yPos, xgoal - xPos) + Math.PI;

                double angleError = targetAngle - heading;

                while (angleError > Math.PI)  angleError -= 2 * Math.PI;
                while (angleError < -Math.PI) angleError += 2 * Math.PI;
                rx = angleError * Kp;

                rx = Math.max(-0.5, Math.min(0.5, rx));
            }

            if (gamepad1.options && !lastOptions) {
                odo.resetPosAndIMU();
            }
            lastOptions = gamepad1.options;

            // Field Centric Drive Math
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denom = Math.max(abs(rotY) + abs(rotX) + abs(rx), 1);
            leftFront.setPower((rotY + rotX + rx) / denom);
            leftBack.setPower((rotY - rotX + rx) / denom);
            rightFront.setPower((rotY - rotX - rx) / denom);
            rightBack.setPower((rotY + rotX - rx) / denom);

            telemetry.addData("Heading (Deg)", Math.toDegrees(heading));
            telemetry.addData("Target (Deg)", Math.toDegrees(Math.atan2(ygoal - yPos, xgoal - xPos)));
            telemetry.update();
        }
    }
}