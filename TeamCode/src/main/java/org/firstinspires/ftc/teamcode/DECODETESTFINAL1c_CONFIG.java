package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sqrt;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;

@TeleOp(name="DecodeTestFinal1c - Config", group="Robot")
public class DECODETESTFINAL1c_CONFIG extends LinearOpMode {

    private DcMotor intake;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    double Kp = 0.8;
    private Servo kicker;
    private DcMotor bootkicker;
    private RevColorSensorV3 distanceSensor;
    private PinpointLocalizer pinpointLocalizer;
    double distance_from_goal = 0;
    double xPos = 0;
    double yPos = 0;
    double robot_heading = 0;
    double ygoal = 121;
    double xgoal = 60;

    // 1200 from 53-65 inches  - camera to april tag
    private static final double targetPower= -1100; // 1100 from less than 33in from april tag will fail. Works from 63-33in

    enum RobotState {
        IDLE,
        COLLECT,
        SHOOT
    }

    RobotState currentState = RobotState.IDLE;

    boolean lastA = false;
    boolean lastB = false;
    boolean lastDpadRight = false;

    // Latch for auto-fire
    boolean firingEnabled = false;
    boolean lastOptions = false;
    private Servo turret = null;
    public Servo armservo = null;

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
        armservo = hardwareMap.get(Servo.class, "armservo");
        // turret = hardwareMap.get(Servo.class, "rotator");
        PinpointLocalizer pos = new PinpointLocalizer(hardwareMap);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(0.225);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinpointLocalizer = new PinpointLocalizer(hardwareMap);

        // You can change this to using the pinpoint computer. That will eliminate the need for
        // PoseStorage. At the start of teleop we can directly get the pose from the pinpoint computer.
        // Pinpoint computer is also more accurate.


        waitForStart();
        while (opModeIsActive()) {
            pinpointLocalizer.update();
            boolean options = gamepad1.options;
            if (options && !lastOptions) {
                pinpointLocalizer.resetHeading();
            }
            lastOptions = options;

            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);

            // MODE TOGGLE (IF/ ELSE
            boolean a = gamepad2.a;
            boolean b = gamepad2.b;

            if (a && !lastA) {
                if (currentState == RobotState.COLLECT) {
                    currentState = RobotState.IDLE;
                } else {
                    currentState = RobotState.COLLECT;
                }
            }
            if (b && !lastB) {
                if (currentState == RobotState.SHOOT) {
                    currentState = RobotState.IDLE;
                } else {
                    currentState = RobotState.SHOOT;
                }
            }
            lastA = a;
            lastB = b;

            // DRIVETRAIN
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


            //  FIRE BUTTON (edge) Not NEEDED ANYMORE
            boolean pressed = gamepad2.dpad_right && !lastDpadRight;
            lastDpadRight = gamepad2.dpad_right;

            armservo.setPosition(Config.ARM_SERVO_POSITION);

            double xPos = pos.odo.getPosX(DistanceUnit.INCH);
            double yPos = pos.odo.getPosY(DistanceUnit.INCH);
            double robot_heading = pos.odo.getHeading(AngleUnit.RADIANS);
            double distancey = 70-yPos;
            double distancex = 140-xPos;
            double distance_from_goal = sqrt(distancex*distancex+distancey*distancey);
            double angleToGoal = Math.atan2(ygoal - yPos, xgoal - xPos);


            double relativeAngle = angleToGoal - robot_heading;
            while (relativeAngle > Math.PI)  relativeAngle -= 2 * Math.PI;
            while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;


            double servoPosition = Config.SERVO_CENTER + (relativeAngle / Config.SERVO_RAD_RANGE);
            servoPosition = Math.max(Config.SERVO_MIN, Math.min(Config.SERVO_MAX, servoPosition));

//            if (gamepad1.a) {
//                turret.setPosition(servoPosition);
//            } else {
//                turret.setPosition(SERVO_CENTER);
//            }
            if (gamepad1.x) {

                double targetAngle = Math.atan2(ygoal - yPos, xgoal - xPos) + Math.PI;

                double angleError = targetAngle - heading;

                while (angleError > Math.PI)  angleError -= 2 * Math.PI;
                while (angleError < -Math.PI) angleError += 2 * Math.PI;
                rx = angleError * Kp;

                rx = Math.max(-0.5, Math.min(0.5, rx));
            }
            // STATE MACHINE
            switch (currentState) {

                case IDLE:
                    if (distance_from_goal>5 && distance_from_goal<55){
                        shooter.setVelocity(Config.closeVelocity);
                    }
                    else if (distance_from_goal>55 && distance_from_goal<65){
                        shooter.setVelocity(Config.midVelocity);
                    }
                    else{
                        shooter.setVelocity(Config.farVelocity);
                    }
                    firingEnabled = false;
                    shooter.setVelocity(Config.VELO_CLOSE);
                    intake.setPower(Config.INTAKE_IDLE);
                    bootkicker.setPower(Config.BOOTKICKER_IDLE);
                    kicker.setPosition(Config.KICKER_DOWN);
                    break;

                case COLLECT:
                    if (distance_from_goal>5 && distance_from_goal<55){
                        shooter.setVelocity(Config.closeVelocity);
                    }
                    else if (distance_from_goal>55 && distance_from_goal<65){
                        shooter.setVelocity(Config.midVelocity);
                    }
                    else{
                        shooter.setVelocity(Config.farVelocity);
                    }
                    firingEnabled = false;
                    //shooter.setVelocity(VELO_CLOSE);
                    kicker.setPosition(Config.KICKER_DOWN);
                    bootkicker.setPower(Config.BOOTKICKER_COLLECT);
                    intake.setPower(Config.INTAKE_COLLECT);
                    //armservo.setPosition(0.1375);
                    // armservo.setPosition(0.67);

                    break;

                case SHOOT:
                    if (distance_from_goal>5 && distance_from_goal<57){
                        shooter.setVelocity(Config.closeVelocity);
                    }
                    else if (distance_from_goal>57 && distance_from_goal<65){
                        shooter.setVelocity(Config.midVelocity);
                    }
                    else{
                        shooter.setVelocity(Config.farVelocity);
                    }
                    intake.setPower(Config.INTAKE_COLLECT);
                    bootkicker.setPower(Config.BOOTKICKER_COLLECT);
                    //  shooter.setVelocity(VELO_CLOSE);
                    // enable auto-fire on button press

                    // auto-fire while balls exist
                    if (distanceCM < 7) {
                        kicker.setPosition(0.6);
                        sleep(600);
                        kicker.setPosition(0.25);
                    }

                    // stop when no ball
                    if (distanceCM >= 7) {
                        // firingEnabled = false;
                        kicker.setPosition(0.25);
                    }
                    break;
            }


//→ A  - Toggle COLLECT mode
//→ Press again to return to IDLE
// B
//→ Toggle SHOOT mode
//→ Press again to return to IDLE
//            D-pad Right
//→ No effect (present in code but not used)

            telemetry.addData("MODE", currentState);
            telemetry.addData("Distance (cm)", distanceCM);
            telemetry.addData("Firing Enabled", firingEnabled);
            telemetry.addData("Current State is:", currentState);
            telemetry.addData("Motor Power:", shooter.getVelocity());
            telemetry.addData("Intake Power:", intake.getPower());
            telemetry.addData("x:", pinpointLocalizer.getPoseEstimate().getX());
            telemetry.addData("y:", pinpointLocalizer.getPoseEstimate().getY());
            telemetry.addData("heading:", pinpointLocalizer.getHeading());
            telemetry.addData("Distance From Goal:", distance_from_goal);
            telemetry.update();
        }
    }
}


//Gamepad 1 (Driver)
//Left stick → Drive (field-centric)
//Right stick X → Turn
//Gamepad 2 (Operator)
//A → Toggle COLLECT mode
//B → Toggle SHOOT mode
//D-pad Right → Start auto-fire (shoot all balls that are in)
//COLLECT mode active → Intake + bootkicker run
//SHOOT mode active → Shooter spins up
//No balls detected → Shooting stops automatically