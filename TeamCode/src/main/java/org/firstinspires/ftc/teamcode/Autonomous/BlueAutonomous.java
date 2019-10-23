package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Driving.AutonomousSelection;

/**
 * Created by Sean Cardosi on 10/22/19
 * This does not work yet.
 */
@Autonomous(name = "Blue Auto", group = "FraserAuto")
public class BlueAutonomous extends OpMode {

    AutonomousSelection select;

    @Override
    public void init() {
        select = new AutonomousSelection(telemetry);
    }

    @Override
    public void init_loop() {
        select.makeSelections(gamepad1);
    }

    @Override
    public void loop() {
        select.runAutoBlue();
    }
}
