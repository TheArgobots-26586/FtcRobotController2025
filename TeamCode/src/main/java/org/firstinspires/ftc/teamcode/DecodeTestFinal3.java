package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="DecodeTestFinal3", group="Robot")
public class DecodeTestFinal3 extends LinearOpMode {

    private DcMotor intake;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    private Servo kicker;
    private DcMotor bootkicker;
    private RevColorSensorV3 distanceSensor;
    private IMU imu;

    private static final double VELO_CLOSE = 1100;
    private static final double VELO_FAR = 1600; //shooter

    enum RobotState {
        IDLE,
        COLLECT,
        SHOOT,
        SHOOTFAR
    }

    RobotState currentState = RobotState.IDLE;

    boolean lastA = false;
    boolean lastB = false;
    boolean lastX = false;
    boolean lastDpadRight = false;

    // Latch for auto-fire
    boolean firingEnabled = false;
    boolean lastOptions = false;
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

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(0.33);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        boolean options = gamepad1.options;




        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {



            if (options && !lastOptions) {
                imu.resetYaw();
            }
            lastOptions = options;
            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);

            // MODE TOGGLE (IF / ELSE ONLY)
            boolean a = gamepad2.a;
            boolean b = gamepad2.b;
            boolean x_button = gamepad2.x;

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

            if (x_button && !lastX) {
                if (currentState == RobotState.SHOOTFAR) {
                    currentState = RobotState.IDLE;
                } else {
                    currentState = RobotState.SHOOTFAR;
                }
            }


            lastA = a;
            lastB = b;
            lastX = x_button;

            // DRIVETRAIN
            double y  = -gamepad1.left_stick_y / 1.5;
            double x  =  gamepad1.left_stick_x / 1.5;
            double rx =  gamepad1.right_stick_x / 1.5;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFront.setPower((rotY + rotX + rx) / denom);
            leftBack.setPower((rotY - rotX + rx) / denom);
            rightFront.setPower((rotY - rotX - rx) / denom);
            rightBack.setPower((rotY + rotX - rx) / denom);

            //  FIRE BUTTON (edge) Not NEEDED ANYMORE
            boolean pressed = gamepad2.dpad_right && !lastDpadRight;
            lastDpadRight = gamepad2.dpad_right;//not needed

            // STATE MACHINE
            switch (currentState) {

                case IDLE:
                    firingEnabled = false;
                    shooter.setVelocity(VELO_CLOSE);
                    intake.setPower(-0.9);
                    bootkicker.setPower(-0.4);
                    kicker.setPosition(0.33);
                    break;

                case COLLECT:
                    firingEnabled = false;
                    //shooter.setVelocity(VELO_CLOSE);
                    kicker.setPosition(0.33);
                    bootkicker.setPower(-0.4);
                    intake.setPower(-1);
                    armservo.setPosition(0.17);
                    break;
                case SHOOTFAR:
                    intake.setPower(-1);
                    bootkicker.setPower(-0.4);
                    shooter.setVelocity(VELO_FAR);
                    // enable auto-fire on button press
//                    if (pressed) {//pressed is dpad_right
//                        firingEnabled = true;
//                    }
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


                case SHOOT:
                    intake.setPower(-1);
                    bootkicker.setPower(-0.4);


                    shooter.setVelocity(VELO_CLOSE);


                    // enable auto-fire on button press
//                    if (pressed) {//pressed is dpad_right
//                        firingEnabled = true;
//                    }

                    // auto-fire while balls exist
                    if (distanceCM < 7) {
                        kicker.setPosition(0.6);
                        sleep(600);
                        kicker.setPosition(0.25);
                    }

                    // stop when no ball
                    if (distanceCM >= 7) {
                        // firingEnabled = false;
                        kicker.setPosition(0.33);
                    }
                    break;
            }


//A  - Toggle COLLECT mode
// Press again to return to IDLE
// Toggle SHOOT mode
// Press again to return to IDLE
//  D-pad Right
// No effect (present in code but not used)

            telemetry.addData("MODE", currentState);
            telemetry.addData("Distance (cm)", distanceCM);
            telemetry.addData("Firing Enabled", firingEnabled);
            telemetry.addData("Current State is:", currentState);
            telemetry.addData("Motor Power:", shooter.getVelocity());
            telemetry.update();
        }
    }
}


//Gamepad 1 (Driver)
//
//Left stick → Drive (field-centric)
//
//Right stick X → Turn
//
//Gamepad 2 (Operator)
//
//A → Toggle COLLECT mode
//
//B → Toggle SHOOT mode
//
//D-pad Right → Start auto-fire (shoot all balls that are in)

//
//COLLECT mode active → Intake + bootkicker run
//
//SHOOT mode active → Shooter spins up
//
//No balls detected → Shooting stops automatically