package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="ConstantDecodeOuttake", group="Robot")
public class ConstantDecodeOuttake extends LinearOpMode {

    private DcMotorEx outputMotor;

    // Set the velocity target in ticks per second
    private static final double TARGET_VELO0 = 0;
    private static final double TARGET_VELO1 = -1000;
    private static final double TARGET_VELO2 = -650;
    private static final double TARGET_VELO3 = -1550;

    @Override
    public void runOpMode() {
        outputMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Using velocity tracking with encoders starting (needed)
        outputMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outputMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Run the motor at constant speed
            if (gamepad1.a) {
                outputMotor.setVelocity(TARGET_VELO1);
                telemetry.addData("Target Velo", TARGET_VELO1);
                telemetry.update();

            }
            else if (gamepad1.y) {
                outputMotor.setVelocity(TARGET_VELO0);
                telemetry.addData("Target Velo", TARGET_VELO2);
                telemetry.update();
            }
            else if (gamepad1.x) {
                outputMotor.setVelocity(TARGET_VELO2);
                telemetry.addData("Target Velo", TARGET_VELO2);
                telemetry.update();

            }
            else if (gamepad1.b) {
                outputMotor.setVelocity(TARGET_VELO3);
                telemetry.addData("Target Velo", TARGET_VELO3);
                telemetry.update();

            }
            telemetry.addData("Actual Velo", outputMotor.getVelocity());
            telemetry.update();

        }
    }
}
