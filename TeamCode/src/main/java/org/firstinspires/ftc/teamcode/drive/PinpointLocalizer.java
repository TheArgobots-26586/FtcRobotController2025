package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointLocalizer implements Localizer {

    int num_setpos_calls =0;
    private final GoBildaPinpointDriver odo;
    public static int num_instances = 0;
    private Pose2d poseEstimate = new Pose2d(0, 0, 0);//chance x and y back to 0,0

    public PinpointLocalizer(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set your pod type
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        num_instances ++;

        // TODO: Replace with YOUR measured offsets (mm)
        //odo.setOffsets(-111, -127, DistanceUnit.MM);
        odo.setOffsets(-86.36, -121.9, DistanceUnit.MM);

        // Set directions based on your mounting
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        odo.resetPosAndIMU();
    }

    @Override
    public void update() {
        odo.update();

        // Standard position pull
        Pose2D p = odo.getPosition();

        poseEstimate = new Pose2d(
                p.getX(DistanceUnit.INCH),
                p.getY(DistanceUnit.INCH),
                p.getHeading(AngleUnit.RADIANS)
        );
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        poseEstimate = pose;
        num_setpos_calls++;

        // Pinpoint requires its own Pose2D format to reset internal tracking
        odo.setPosition(new Pose2D(
                DistanceUnit.INCH,
                pose.getX(),
                pose.getY(),
                AngleUnit.RADIANS,
                pose.getHeading()
        ));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        // Individual axis velocities in INCHES
        double vx = odo.getVelX(DistanceUnit.INCH);
        double vy = odo.getVelY(DistanceUnit.INCH);


        // This matches the screenshot requirement for UnnormalizedAngleUnit
        double vh = odo.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized());

        return new Pose2d(vx, vy, vh);
    }


    public void setPoseVelocity(@Nullable Pose2d poseVelocity) {

    }

    public double getHeading() {
        return odo.getHeading(AngleUnit.RADIANS);
    }
    public void resetHeading() {
        odo.setHeading(0, AngleUnit.RADIANS);
    }

    public void reset() { odo.resetPosAndIMU();}

    public int getNumSetPosCalls() { return num_setpos_calls;}
}