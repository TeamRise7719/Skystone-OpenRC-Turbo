package org.firstinspires.ftc.teamcode.SeansPlayground.PurePursuit.DriveWheelBasedPurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.SeansPlayground.PurePursuit.OdometerBasedPurePursuit.CurvePoint;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 11/6/2019
 * Example of how to use Pure Pursuit to create an autonomous.
 */
@Autonomous(name = "Drive Wheel Pure Pursuit Test", group = "Pure Pursuit")
public class DriveWheelBasedPurePursuitTest extends OpMode {

    DriveWheelPurePursuitMovement movement;

    @Override
    public void init() {
        movement = new DriveWheelPurePursuitMovement(telemetry, hardwareMap);
        movement.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(20,0,0.5,0.5,50,Math.toRadians(50),0.5));
//        allPoints.add(new CurvePoint(20,20,0.5,0.5,50,Math.toRadians(50),0.5));

        movement.followCurve(allPoints, Math.toRadians(90));//Robot will get stuck spinning while looking for another point to go to at the endPoint.
    }
}
