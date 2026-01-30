package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


class Config {
    public static final double KICKER_DOWN = 0.225;
    public static final double KICKER_UP = 0.6;
    // 1200 from 53-65 inches  - camera to april tag
    public static final double VELO_CLOSE = 1200; // 1100 from less than 33in from april tag will fail. Works from 63-33in
    public static final double ARM_SERVO_POSITION = 0.24;
    public static final double INTAKE_IDLE = -0.1;
    public static final double BOOTKICKER_IDLE = -0.1;
    public static final double INTAKE_COLLECT = -0.9;
    public static final double BOOTKICKER_COLLECT = -0.4;
    public static final double INTAKE_SHOOT = -0.2;
    public static final double BOOTKICKER_SHOOT = -0.2;
    public static final double MAX_COLOR_SENSED_DISTANCE = 7;
    public static final double SERVO_CENTER = 0.5; // Your servo's "straight ahead" position
    public static final double SERVO_MIN = 0.0;    // Limit to prevent hitting the robot frame
    public static final double SERVO_MAX = 1.0;    // Limit to prevent hitting the robot frame
    public static final double SERVO_RAD_RANGE = Math.toRadians(180);

    public static final int closeVelocity = 1100;

    public static final int midVelocity = 1200;

    public static final int farVelocity = 1475;

}