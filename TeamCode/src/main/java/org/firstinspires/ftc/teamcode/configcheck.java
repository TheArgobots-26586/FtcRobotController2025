//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
//
//@TeleOp(name="config check", group="Robot")
//public class configcheck extends LinearOpMode {
//    PinpointLocalizer pinpointLocalizer;
//    Config config;
//    @Override
//    public void runOpMode() {
//        telemetry.addLine("Config initialization");
//        telemetry.update();
//        config = new Config(hardwareMap);
//        pinpointLocalizer = new PinpointLocalizer(hardwareMap);
//        waitForStart();
//        while (opModeIsActive()) {
//            telemetry.addData("in while loop", 0);
//            telemetry.update();
//        }
//    }
//}