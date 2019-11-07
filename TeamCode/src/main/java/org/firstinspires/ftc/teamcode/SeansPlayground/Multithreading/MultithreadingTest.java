package org.firstinspires.ftc.teamcode.SeansPlayground.Multithreading;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MultithreadingTest extends OpMode {

    aThread a;
    @Override
    public void init() {
        a = new aThread(telemetry);


    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            a.run();
        } else if (gamepad1.b) {
            a.interrupt();
        }
    }
}
