package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sqrt;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;

@TeleOp(name="DecodeTestFinal4a", group="Robot")
public class DecodeTestFinal4a extends LinearOpMode {

    private double currentPos = 0.5;

    private static final double CAMERA_CENTER_POS = 0.35;
    private static final double SERVO_GAIN = 0.0067;
    private static final double ROTATOR_MIN = 0.45;
    private static final double ROTATOR_MAX = 1.0;

    // ---------------- HARDWARE ----------------
    private DcMotor intake;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    private Servo kicker;
    private DcMotor bootkicker;
    private RevColorSensorV3 distanceSensor;
    private PinpointLocalizer pinpointLocalizer;
    private Servo turret;
    private Servo armservo;


    private Limelight3A limelight;

    double ygoal = 60;
    double xgoal = 60;

    enum RobotState { IDLE, COLLECT, SHOOT }
    RobotState currentState = RobotState.IDLE;

    boolean lastA = false;
    boolean lastB = false;
    boolean lastOptions = false;
    boolean lastX = false;

    @Override
    public void runOpMode() {

        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "RightBack");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        kicker     = hardwareMap.get(Servo.class, "kicker");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");
        bootkicker = hardwareMap.get(DcMotor.class, "bootkicker");
        armservo   = hardwareMap.get(Servo.class, "armservo");
        turret     = hardwareMap.get(Servo.class, "rotator");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(0.25);
        turret.setPosition(currentPos);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pinpointLocalizer = new PinpointLocalizer(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            pinpointLocalizer.update();

            if (gamepad1.options && !lastOptions) {
                pinpointLocalizer.resetHeading();
            }
            lastOptions = gamepad1.options;

            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);

            if (gamepad2.a && !lastA) {
                currentState = (currentState == RobotState.COLLECT)
                        ? RobotState.IDLE : RobotState.COLLECT;
            }
            if (gamepad2.b && !lastB) {
                currentState = (currentState == RobotState.SHOOT)
                        ? RobotState.IDLE : RobotState.SHOOT;
            }
            lastA = gamepad2.a;
            lastB = gamepad2.b;

            double ly = -gamepad1.left_stick_y / 1.5;
            double lx =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;

            double heading = pinpointLocalizer.getHeading();
            double rotX = lx * Math.cos(-heading) - ly * Math.sin(-heading);
            double rotY = lx * Math.sin(-heading) + ly * Math.cos(-heading);

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFront.setPower((rotY + rotX + rx) / denom);
            leftBack.setPower((rotY - rotX + rx) / denom);
            rightFront.setPower((rotY - rotX - rx) / denom);
            rightBack.setPower((rotY + rotX - rx) / denom);

            armservo.setPosition(0.1375);


            Pose2d currentPose = pinpointLocalizer.getPoseEstimate();
            double xPos = currentPose.getX();
            double yPos = currentPose.getY();

            double dx = xgoal - xPos;
            double dy = ygoal - yPos;
            double distance_from_goal = sqrt(dx * dx + dy * dy);


            boolean x = gamepad2.x;//click x to autoallign turret using apriltag

            if (x && !lastX) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                        if (fid.getFiducialId() == 20) {

                            double tx = result.getTx();
                            double targetPos = CAMERA_CENTER_POS + (tx * SERVO_GAIN);
                            targetPos = Math.max(ROTATOR_MIN, Math.min(ROTATOR_MAX, targetPos));

                            currentPos = targetPos;
                            break;
                        }
                    }
                }
            }
            lastX = x;


            switch (currentState) {

                case IDLE:
                    intake.setPower(-0.1);
                    bootkicker.setPower(-0.1);
                    kicker.setPosition(0.25);
                    break;

                case COLLECT:
                    kicker.setPosition(0.25);
                    bootkicker.setPower(-0.4);
                    intake.setPower(-1);
                    break;

                case SHOOT:
                    if (distance_from_goal > 28 && distance_from_goal < 55) {
                        shooter.setVelocity(-1100);
                    } else if (distance_from_goal >= 55 && distance_from_goal < 65) {
                        shooter.setVelocity(-1200);
                    } else {
                        shooter.setVelocity(-1600);
                    }

                    intake.setPower(-0.2);
                    bootkicker.setPower(-0.2);

                    if (distanceCM < 7) {
                        kicker.setPosition(0.6);
                        sleep(600);
                        kicker.setPosition(0.25);
                    } else {
                        kicker.setPosition(0.25);
                    }
                    break;
            }

            turret.setPosition(currentPos);

            telemetry.addData("MODE", currentState);
            telemetry.addData("Dist to Goal", "%.2f", distance_from_goal);
            telemetry.addData("X", "%.2f", xPos);
            telemetry.addData("Y", "%.2f", yPos);
            telemetry.addData("Turret Pos", "%.3f", currentPos);
            telemetry.update();
        }
    }
}
