package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;

@TeleOp(name="DecodeTestFinal - 1.3", group="Robot")
public class DecodeTestFinal_Version1_3 extends LinearOpMode {

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
    PinpointLocalizer pinpointLocalizer = new PinpointLocalizer(hardwareMap);
    @Override
    public void runOpMode() {
        Config config = new Config(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            pinpointLocalizer.update();
            boolean options = gamepad1.options;
            if (options && !lastOptions) {
                pinpointLocalizer.resetHeading();
            }
            lastOptions = options;

            double distanceCM = config.distanceSensor.getDistance(DistanceUnit.CM);

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
            config.leftFront.setPower((rotY + rotX + rx) / denom);
            config.leftBack.setPower((rotY - rotX + rx) / denom);
            config.rightFront.setPower((rotY - rotX - rx) / denom);
            config.rightBack.setPower((rotY + rotX - rx) / denom);


            //  FIRE BUTTON (edge) Not NEEDED ANYMORE
            lastDpadRight = gamepad2.dpad_right;//dpadright to

            config.armservo.setPosition(Config.ARM_SERVO_POSITION);
            // STATE MACHINE
            switch (currentState) {

                case IDLE:
                    firingEnabled = false;
                    config.shooter.setVelocity(Config.VELO_CLOSE);
                    config.intake.setPower(Config.INTAKE_IDLE);
                    config.bootkicker.setPower(Config.BOOTKICKER_IDLE);
                    config.kicker.setPosition(Config.KICKER_DOWN);
                    break;

                case COLLECT:
                    firingEnabled = false;
                    //shooter.setVelocity(VELO_CLOSE);
                    config.kicker.setPosition(Config.KICKER_DOWN);
                    config.bootkicker.setPower(Config.BOOTKICKER_COLLECT);
                    config.intake.setPower(Config.INTAKE_COLLECT);
                    //armservo.setPosition(0.1375);
                    // armservo.setPosition(0.67);
                    break;

                case SHOOT:
                    config.intake.setPower(Config.INTAKE_SHOOT);
                    config.bootkicker.setPower(Config.BOOTKICKER_SHOOT);
                    config.shooter.setVelocity(Config.VELO_CLOSE);


                    // enable auto-fire on button press
//                    if (pressed) {//pressed is dpad_right
//                        firingEnabled = true;
//                    }

                    // auto-fire while balls exist
                    if (distanceCM < Config.MAX_COLOR_SENSED_DISTANCE) {
                        config.kicker.setPosition(Config.KICKER_UP);
                        sleep(600);
                        config.kicker.setPosition(Config.KICKER_DOWN);
                    }

                    // stop when no ball
                    if (distanceCM >= Config.MAX_COLOR_SENSED_DISTANCE) {
                        // firingEnabled = false;
                        config.kicker.setPosition(Config.KICKER_DOWN);
                    }
                    break;
            }


//→ A  - Toggle COLLECT mode
//→ Press again to return to IDLE
//
//                    B
//→ Toggle SHOOT mode
//→ Press again to return to IDLE
//
//            D-pad Right
//→ No effect (present in code but not used)

            telemetry.addData("MODE", currentState);
            telemetry.addData("Distance (cm)", distanceCM);
            telemetry.addData("Firing Enabled", firingEnabled);
            telemetry.addData("Current State is:", currentState);
            telemetry.addData("Motor Power:", config.shooter.getVelocity());
            telemetry.addData("Intake Power:", config.intake.getPower());
            telemetry.addData("x:", pinpointLocalizer.getPoseEstimate().getX());
            telemetry.addData("y:", pinpointLocalizer.getPoseEstimate().getY());
            telemetry.addData("heading:", pinpointLocalizer.getHeading());
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