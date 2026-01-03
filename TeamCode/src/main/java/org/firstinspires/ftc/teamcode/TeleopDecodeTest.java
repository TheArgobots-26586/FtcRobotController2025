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

@TeleOp(name="TeleopDecodeTest", group="Robot")
public class TeleopDecodeTest extends LinearOpMode {

    private DcMotor intake;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    private Servo kicker;
    private CRServo bootkicker;
    private RevColorSensorV3 distanceSensor;
    private IMU imu;

    private static final double VELO_CLOSE = 700;

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

    @Override
    public void runOpMode() {

        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_distance");

        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");

        intake = hardwareMap.get(DcMotor.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        bootkicker = hardwareMap.get(CRServo.class, "bootkicker");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(0.33);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));
    }
}