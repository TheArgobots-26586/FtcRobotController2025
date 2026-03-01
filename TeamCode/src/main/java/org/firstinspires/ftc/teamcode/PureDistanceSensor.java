package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor: REV 2M TeleOp", group = "Sensor")
public class PureDistanceSensor extends LinearOpMode {

    private Rev2mDistanceSensor sensorDistance;

    @Override
    public void runOpMode() {

        sensorDistance = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_distance");

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            double distanceCM = sensorDistance.getDistance(DistanceUnit.CM);

            telemetry.addData("Distance (cm)", "%.2f", distanceCM);


            if (distanceCM < 20.0) {
                telemetry.addData("Warning", "Object Close!");
            }

            telemetry.update();
        }
    }
}
