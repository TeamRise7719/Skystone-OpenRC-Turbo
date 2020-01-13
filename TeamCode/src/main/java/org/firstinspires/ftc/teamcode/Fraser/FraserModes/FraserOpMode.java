package org.firstinspires.ftc.teamcode.Fraser.FraserModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserDrivetrain;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Media.RobotMedia;

/**
 * Created by Sean Cardosi on 2020-01-11.
 */
public abstract class FraserOpMode extends OpMode {

    public FraserComponents component;
    public RobotMedia media;
    public FraserDrivetrain robot;
    boolean isReady = false;

    public FraserOpMode() {}

    @Override
    public void init() {
        component = new FraserComponents(hardwareMap);
        component.init();
        robot = new FraserDrivetrain(hardwareMap);
        robot.runUsingEncoders();

        isReady = true;
    }

    @Override
    public void init_loop() {
        if(isReady) {
            telemetry.addData(">", "Robot Ready!");
            telemetry.update();
        }
    }

    @Override
    public void loop() {

    }

    @Override
    public void start() {
        super.start();
        media = new RobotMedia(hardwareMap);
        media.startTimer();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
