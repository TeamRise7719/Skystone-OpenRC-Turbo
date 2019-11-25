package org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Driving;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitMathElements.CurvePoint;

import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit.PurePursuitMovement.followCurve;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 10/22/2019
 * Class used during autonomous initialization that reads the Skystones location and uses a predefined path
 * based off of the Skystone locations. TODO: Finish this class
 */
public class AutonomousPathing {

    Telemetry telemetry;
    boolean path14 = false;//Path to follow if the die roll was a 1 or a 4
    boolean path25 = false;//Path to follow if the die roll was a 2 or a 5
    boolean path36 = false;//Path to follow if the die roll was a 3 or a 6



    public AutonomousPathing(Telemetry tel) {
        telemetry = tel;
    }

    public void findPath() {

        /**
         * Code here to find the Skystones (x,y) location and to make the correct boolean true.
         * Find Skystones locations using TensorFlowObjectDetection.
         */


    }
    public void runPurePursuitPath() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        if (path14 == true) {

            //Code to follow to get Skystones in this position
            allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50));

        } else if (path25 == true) {

            //Code to follow to get Skystones in this position
            allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50));

        } else if (path36 == true) {

            //Code to follow to get Skystones in this position
            allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50));

        }
        followCurve(allPoints, Math.toRadians(90));//Robot will get stuck spinning while looking for another point to go to at the endPoint.
    }


    public void runPIDPath() {

        if (path14 == true) {

            //Code to follow to get Skystones in this position

        } else if (path25 == true) {

            //Code to follow to get Skystones in this position

        } else if (path36 == true) {

            //Code to follow to get Skystones in this position

        }
    }
}