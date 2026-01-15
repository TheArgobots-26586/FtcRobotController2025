package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;

class Config {

    public DcMotor intake, leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx shooter;
    public Servo kicker;
    public DcMotor bootkicker;
    public RevColorSensorV3 distanceSensor;
    public Servo armservo;
    public PinpointLocalizer pinpointLocalizer;
    public static final double KICKER_DOWN = 0.25;
    public static final double KICKER_UP = 0.6;
    // 1200 from 53-65 inches  - camera to april tag
    public static final double VELO_CLOSE = -1100; // 1100 from less than 33in from april tag will fail. Works from 63-33in
    public static final double ARM_SERVO_POSITION = 0.1375;
    public static final double INTAKE_IDLE = -0.1;
    public static final double BOOTKICKER_IDLE = -0.1;
    public static final double INTAKE_COLLECT = -1;
    public static final double BOOTKICKER_COLLECT = -0.4;
    public static final double INTAKE_SHOOT = -0.2;
    public static final double BOOTKICKER_SHOOT = -0.2;
    public static final double MAX_COLOR_SENSED_DISTANCE = 7;

    public Config(HardwareMap hardwareMap) {
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
        kicker.setPosition(Config.KICKER_DOWN);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinpointLocalizer = new PinpointLocalizer(hardwareMap);

    }
}
