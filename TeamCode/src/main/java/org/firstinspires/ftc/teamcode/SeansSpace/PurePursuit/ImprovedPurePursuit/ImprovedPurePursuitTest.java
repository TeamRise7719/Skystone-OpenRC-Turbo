package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.ImprovedPurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 2019-12-05.
 */
public class ImprovedPurePursuitTest extends OpMode {

    PathGenerator path;

    @Override
    public void init() {

        path = new PathGenerator(telemetry,hardwareMap);
        path.init();
    }

    @Override
    public void loop() {

        path.startThread();

        ArrayList<Point> pathPoints = new ArrayList<>();
        pathPoints.add(new Point(0,0));
        pathPoints.add(new Point(10,10));
        pathPoints.add(new Point(20,10));
        pathPoints.add(new Point(30,20));
        path.followPath(pathPoints,0.5,25);

    }

    public void stop() {
        path.stopThread();
    }
}
