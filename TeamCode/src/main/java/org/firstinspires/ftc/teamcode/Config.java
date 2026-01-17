package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


class Config {

    public DcMotor intake, leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx shooter;
    public Servo kicker;
    public DcMotor bootkicker;
    public RevColorSensorV3 distanceSensor;
    public Limelight3A limelight;
    public Servo armservo;
    public static final double KICKER_DOWN = 0.225;
    public static final double KICKER_UP = 0.6;
    // 1200 from 53-65 inches  - camera to april tag
    public static final double VELO_CLOSE = -1100; // 1100 from less than 33in from april tag will fail. Works from 63-33in
    public static final double ARM_SERVO_POSITION = 0.24;
    public static final double INTAKE_IDLE = -0.1;
    public static final double BOOTKICKER_IDLE = -0.1;
    public static final double INTAKE_COLLECT = -0.9;
    public static final double BOOTKICKER_COLLECT = -0.4;
    public static final double INTAKE_SHOOT = -0.2;
    public static final double BOOTKICKER_SHOOT = -0.2;
    public static final double MAX_COLOR_SENSED_DISTANCE = 7;
    public static final double CAMERA_CENTER_POS = 0.35;
    public static final double SERVO_GAIN = 0.0067;
    public static final double ROTATOR_MIN = 0.45;
    public static final double ROTATOR_MAX = 1.0;


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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        kicker.setPosition(Config.KICKER_DOWN);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
