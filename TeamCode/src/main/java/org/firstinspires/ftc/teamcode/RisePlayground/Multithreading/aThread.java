package org.firstinspires.ftc.teamcode.RisePlayground.Multithreading;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class aThread extends Thread {

    Telemetry telemetry;
    public aThread(Telemetry tel) {
        telemetry = tel;

        this.setName("aThread");
        telemetry.addData("%s",this.getName());

    }
    @Override
    public void run() {
        telemetry.addData("Starting Thread %s", this.getName());

        while (isInterrupted() == false) {
            telemetry.addData("yeet"," this is a thread");

            if (isInterrupted() == true) {
                break;
            }
        }
        telemetry.addData("Ending Thread %s", this.getName());
    }
}
