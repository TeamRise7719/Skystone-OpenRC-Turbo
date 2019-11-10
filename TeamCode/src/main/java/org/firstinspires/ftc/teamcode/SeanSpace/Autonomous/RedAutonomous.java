//package org.firstinspires.ftc.teamcode.SeansPlayground.Autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.SeansPlayground.Subsystems.AutonomousPathing;
//
///**
// * Created by Sean Cardosi on 10/22/19
// * This does not work yet.
// */
//@Disabled
//@Autonomous(name = "Red Auto", group = "FraserAuto")
//public class RedAutonomous extends OpMode {
//
//    AutonomousPathing select;
//
//    @Override
//    public void init() {
//        select = new AutonomousPathing(telemetry);
//    }
//
//    @Override
//    public void init_loop() {
//        super.init_loop();
//        select.makeSelections(gamepad1);
//    }
//
//    @Override
//    public void loop() {
//        select.runAutoRed();
//    }
//}
