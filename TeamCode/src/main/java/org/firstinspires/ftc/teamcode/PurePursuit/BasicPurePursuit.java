package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitMovement.followCurve;
import static org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitMovement.goToPosition;

public class BasicPurePursuit extends OpMode {


    public void init() {

        PurePursuitMovement.init();

    }

    public void loop() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0,0,1.0,1.0,50,Math.toRadians(50),1.0));

        followCurve(allPoints, Math.toRadians(90));//Robot will get stuck spinning while looking for another poiint to go to at then endPoint.
    }
}
