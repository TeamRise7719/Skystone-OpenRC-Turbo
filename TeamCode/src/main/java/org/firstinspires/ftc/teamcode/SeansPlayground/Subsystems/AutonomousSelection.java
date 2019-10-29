package org.firstinspires.ftc.teamcode.SeansPlayground.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SeansPlayground.PurePursuit.CurvePoint;

import static org.firstinspires.ftc.teamcode.SeansPlayground.PurePursuit.PurePursuitMovement.followCurve;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 10/22/2019
 * Class used during autonomous initialization that
 * uses gamepad inputs to select what we want to do.
 */
public class AutonomousSelection {

    Telemetry telemetry;
    private boolean parkOverAllianceBridge = false;
    private boolean firstSkystone = false;
    private boolean secondSkystone = false;
    private boolean moveBuildFoundation = false;
    private boolean parkInBuildSite = false;
    private boolean thirdStone = false;

    public AutonomousSelection(Telemetry tel) {
        telemetry = tel;
    }

    /**
     * Uses gamepad inputs to choose what to do during autonomous.
     * This should be called during initialization (use init_loop()).
     * @param gamepad
     */
    public void makeSelections(Gamepad gamepad) {

            telemetry.addData("Park Over the Alliance Bridge: Press the a button.", parkOverAllianceBridge);
            telemetry.addData("Score One Skystone: Press the b button.", firstSkystone);
            telemetry.addData("Score a Second Skystone: Press the x button.", secondSkystone);
            telemetry.addData("Move the Build Foundation: Press the y button.", moveBuildFoundation);
            telemetry.addData("Park in the Build Site: Press the dpad up.", parkInBuildSite);
            telemetry.addData("Try for a Third Stone: Press the dpad down.", thirdStone);

            if (gamepad.a) {
                parkOverAllianceBridge = !parkOverAllianceBridge;
            }
            if (gamepad.b) {
                firstSkystone = !firstSkystone;
            }
            if (gamepad.x) {
                secondSkystone = !secondSkystone;
            }
            if (gamepad.y) {
                moveBuildFoundation = !moveBuildFoundation;
            }
            if (gamepad.dpad_up) {
                parkInBuildSite = !parkInBuildSite;
            }
            if (gamepad.dpad_down) {
                thirdStone = !thirdStone;
            }
    }

    /**
     * Instructions for the red autonomous using Pure Pursuit based off of the controller inputs.
     * Called in a normal OpMode as Pure Pursuit runs in a normal OpMode. I have made an example
     * of how to program paths for this method.
     */
    public void runAutoRed() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        if (parkOverAllianceBridge == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }
        if (firstSkystone == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }
        if (secondSkystone == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }
        if (moveBuildFoundation == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }
        if (parkInBuildSite == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }
        if (thirdStone == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }

        followCurve(allPoints, Math.toRadians(90));//Robot will get stuck spinning while looking for another point to go to at the endPoint.
    }

    /**
     * Instructions for the blue autonomous using Pure Pursuit based off of the controller inputs.
     * Called in a normal OpMode as Pure Pursuit runs in a normal OpMode. I have made an example
     * of how to program paths for this method.
     */
    public void runAutoBlue() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        if (parkOverAllianceBridge == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }
        if (firstSkystone == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }
        if (secondSkystone == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }
        if (moveBuildFoundation == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }
        if (parkInBuildSite == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }
        if (thirdStone == true) {
            allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));
        }

        followCurve(allPoints, Math.toRadians(90));//Robot will get stuck spinning while looking for another point to go to at the endPoint.
    }
}
