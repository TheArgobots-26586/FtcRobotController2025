package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class poseStorage {
    public static Pose2d currentPose = new Pose2d();
    public static StandardTrackingWheelLocalizer localizer = null;
}