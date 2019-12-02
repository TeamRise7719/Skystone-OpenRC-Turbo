package org.firstinspires.ftc.teamcode.SeansSpace.SeanThreading;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Sean Cardosi on 2019-11-30.
 */
public class SeanThread implements Runnable {


    boolean isRunning = true;
    int i = 0;

    @Override
    public void run() {
        while (true) {

            try {

                i++;

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
//    public void stop() {
//        isRunning = false;
//    }
}
