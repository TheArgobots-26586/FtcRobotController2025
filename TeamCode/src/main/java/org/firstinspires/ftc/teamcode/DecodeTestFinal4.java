package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sqrt;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;


@TeleOp(name="DecodeTestFinal4", group="Robot")
public class DecodeTestFinal4 extends LinearOpMode {
    private final double SERVO_CENTER = 0.5;
    private double currentPos = 0.5;
    private final double SERVO_MIN = 0.0;
    private final double SERVO_MAX = 1.0;
    private final double SERVO_RAD_RANGE = Math.toRadians(180);

    private DcMotor intake;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    private Servo kicker;
    private DcMotor bootkicker;
    private RevColorSensorV3 distanceSensor;
    private PinpointLocalizer pinpointLocalizer;
    private Servo turret;
    private Servo armservo;

    double ygoal = 60;
    double xgoal = 60;

    enum RobotState { IDLE, COLLECT, SHOOT }
    RobotState currentState = RobotState.IDLE;

    boolean lastA = false;
    boolean lastB = false;
    boolean lastOptions = false;

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

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(0.25);
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
                currentState = (currentState == RobotState.COLLECT) ? RobotState.IDLE : RobotState.COLLECT;
            }
            if (gamepad2.b && !lastB) {
                currentState = (currentState == RobotState.SHOOT) ? RobotState.IDLE : RobotState.SHOOT;
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

            //armservo position
            armservo.setPosition(0.1375);

            Pose2d currentPose = pinpointLocalizer.getPoseEstimate();
            double xPos = currentPose.getX();
            double yPos = currentPose.getY();
//distance calc
            double dx = xgoal - xPos;
            double dy = ygoal - yPos;
            double distance_from_goal = sqrt(dx*dx + dy*dy);

            double angleToGoal = Math.atan2(dy, dx);
            double relativeAngle = angleToGoal - heading;
            //servo allignment

            while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
            while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

            double servoPosition = SERVO_CENTER + (relativeAngle / SERVO_RAD_RANGE);
            servoPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPosition));
                //allign servo to goal with x
            if (gamepad2.x) {
                currentPos = servoPosition;
            }
            //Auto allignment of turret using odometry<---!
//Chasis Rotation from Odo
//            if (gamepad1.a) {
//
//                double targetAngle = Math.atan2(ygoal - yPos, xgoal - xPos) + Math.PI;
//
//                double angleError = targetAngle - heading;
//
//                while (angleError > Math.PI)  angleError -= 2 * Math.PI;
//                while (angleError < -Math.PI) angleError += 2 * Math.PI;
//                rx = angleError * Kp;//set kp
//
//                rx = Math.max(-0.5, Math.min(0.5, rx));
//            }

            switch (currentState) {
                case IDLE:
                 //   shooter.setVelocity(0);
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
            telemetry.addData("Turret Pos", "%.2f", servoPosition);
            telemetry.update();
        }
    }
}