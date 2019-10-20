package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitMovement.followCurve;
import static org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitMovement.goToPosition;

/**
 * Created by Sean Cardosi.
 * Example of how to use Pure Pursuit to create an autonomous.
 */
@Autonomous(name = "Pure Pursuit Test", group = "Pure Pursuit")
public class PurePursuitTest extends OpMode {


    public void init() {

    }

    public void loop() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));

        followCurve(allPoints, Math.toRadians(90));//Robot will get stuck spinning while looking for another point to go to at the endPoint.
    }
}
