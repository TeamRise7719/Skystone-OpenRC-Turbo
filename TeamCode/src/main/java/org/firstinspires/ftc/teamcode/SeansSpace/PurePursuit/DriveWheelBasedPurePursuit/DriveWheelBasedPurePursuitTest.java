package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitGeometry.CurvePoint;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 11/6/2019
 * Example of how to use Pure Pursuit.
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

        ArrayList<CurvePoint> waypoints = new ArrayList<>();
        waypoints.add(new CurvePoint(0, 200, 1, 1, 20));
        waypoints.add(new CurvePoint(200, 200, 1, 1, 20));
        //TODO: MAKE THIS BETTER
        //WARNING: If a point in the middle of the path is closer to the robot than the next point, the robot will skip to that point.
        //WARNING: Robot will get stuck spinning while looking for another point to go to at the endPoint.
        DriveWheelPurePursuitMovement.followCurve(waypoints, Math.toRadians(0));
    }
}
