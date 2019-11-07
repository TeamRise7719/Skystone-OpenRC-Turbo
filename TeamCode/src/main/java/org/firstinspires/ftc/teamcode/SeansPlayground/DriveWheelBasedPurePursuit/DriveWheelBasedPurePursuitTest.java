package org.firstinspires.ftc.teamcode.SeansPlayground.DriveWheelBasedPurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SeansPlayground.PurePursuit.CurvePoint;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.SeansPlayground.PurePursuit.PurePursuitMovement.followCurve;

/**
 * Created by Sean Cardosi on 11/6/2019
 * Example of how to use Pure Pursuit to create an autonomous.
 */
@Autonomous(name = "Drive Wheel Pure Pursuit Test", group = "Pure Pursuit")
public class DriveWheelBasedPurePursuitTest extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));

        followCurve(allPoints, Math.toRadians(90));//Robot will get stuck spinning while looking for another point to go to at the endPoint.
    }
}
