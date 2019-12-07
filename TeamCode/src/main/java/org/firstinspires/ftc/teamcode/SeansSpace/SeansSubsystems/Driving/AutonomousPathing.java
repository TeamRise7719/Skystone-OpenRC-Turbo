package org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Driving;

import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitGeometry.CurvePoint;

import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit.PurePursuitMovement.followCurve;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 10/22/2019
 * Class used during autonomous initialization that reads the Skystones location and uses a predefined path
 * based off of the Skystone locations. TODO: Finish this class
 */
public class AutonomousPathing {

    private boolean path14 = false;//Path to follow if the die roll was a 1 or a 4
    private boolean path25 = false;//Path to follow if the die roll was a 2 or a 5
    private boolean path36 = false;//Path to follow if the die roll was a 3 or a 6



    public AutonomousPathing() {}

    public void findPath() {

        /*
         * Code here to find the Skystones (x,y) location and to make the correct boolean true.
         * Find Skystones locations using TensorFlowObjectDetection.
         */


    }
    public void runPurePursuitPath() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        if (path14) {

            //Code to follow to get Skystones in this position
            allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50));

        } else if (path25) {

            //Code to follow to get Skystones in this position
            allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50));

        } else if (path36) {

            //Code to follow to get Skystones in this position
            allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50));

        }
        followCurve(allPoints, Math.toRadians(90));//Robot will get stuck spinning while looking for another point to go to at the endPoint.
    }


    public void runPIDPath() {

        if (path14) {

            //Code to follow to get Skystones in this position

        } else if (path25) {

            //Code to follow to get Skystones in this position

        } else if (path36) {

            //Code to follow to get Skystones in this position

        }
    }
}
