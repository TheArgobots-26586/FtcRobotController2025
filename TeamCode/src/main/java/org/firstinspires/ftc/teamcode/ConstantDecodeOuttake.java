package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="ConstantDecodeOuttake", group="Robot")
public class ConstantDecodeOuttake extends LinearOpMode {

    private DcMotorEx outputMotor;

    // Set the velocity target in ticks per second
    private static final double TARGET_VELO1 = 1620;
    private static final double TARGET_VELO0 = 2150;
    private static final double TARGET_VELO2 = 2140;
    private static final double TARGET_VELO3 = 2140;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        outputMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        // Using velocity tracking with encoders starting (needed)
        outputMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outputMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean accelerating = false;
        boolean deccel = false;
        PIDFCoefficients shooterCoefficients = outputMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        double p = shooterCoefficients.p;
        double i = shooterCoefficients.i;
        double d = shooterCoefficients.d;
        double f = shooterCoefficients.f;
        double timePassed = 0;
        PIDFCoefficients pidfNew = new PIDFCoefficients(10, i, d, f);
        outputMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);
        double newP = shooterCoefficients.p;

        telemetry.addData("p: ",p);//10
        telemetry.addData("i: ",i);//3
        telemetry.addData("d: ",d);//0
        telemetry.addData("f: ",f);//0
        telemetry.addData("new p: ",newP);//0
        outputMotor.setVelocity(1460);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Run the motor at constant speed
            if (gamepad1.a) {
                runtime.reset();
                outputMotor.setVelocity(TARGET_VELO1);
               // telemetry.addData("Target Velo", TARGET_VELO1);
                //telemetry.update();
                accelerating = true;

            }
            if(gamepad1.dpad_up) {
                runtime.reset();
                outputMotor.setVelocity(0);
                telemetry.addData("Target Velo", 0);

                deccel = true;
            }
            if (outputMotor.getVelocity() >= TARGET_VELO1-50  && accelerating){
                timePassed = runtime.milliseconds();
                telemetry.addData("Time:",runtime.milliseconds());

                accelerating = false;
                runtime.reset();
            }
            if (outputMotor.getVelocity() <= 20 && deccel){
                telemetry.addData("Time:",runtime.milliseconds());
                deccel = false;
                runtime.reset();
            }

            else if (gamepad1.y) {
                outputMotor.setVelocity(TARGET_VELO0);
                telemetry.addData("Target Velo", TARGET_VELO2);
            }
            else if (gamepad1.x) {
                outputMotor.setVelocity(TARGET_VELO2);
                telemetry.addData("Target Velo", TARGET_VELO2);

            }
            else if (gamepad1.b) {
                outputMotor.setVelocity(TARGET_VELO3);
                telemetry.addData("Target Velo", TARGET_VELO3);
                //telemetry.update();

            }
            dashboardTelemetry.addData("Shooter Current",outputMotor.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("Shooter Velocity",outputMotor.getVelocity());
            telemetry.addData("Time: ", timePassed);
            telemetry.addData("Actual Velo", outputMotor.getVelocity());
            telemetry.update();

        }
    }
}
