package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.teamcode.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.Drivetrain;

/*
 * Created by Sean Cardosi on 9/22/2019.
 */
@TeleOp(name = "BasicTeleOp", group = "Drive")
public class BasicTeleOp extends OpMode {

    private Drivetrain robot;

    private RobotComponents component;

    private boolean isReady = false;

    double turn = 0;

    @Override
    public void init() {

        //Initialize robot
        robot = new Drivetrain(hardwareMap, telemetry);
        robot.runUsingEncoders();

        component = new RobotComponents(hardwareMap, telemetry);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        dashboard.sendTelemetryPacket(packet);
        isReady = true;

    }

    @Override
    public void init_loop() {
        if(isReady==true) {
            telemetry.addData(">", "Robot Ready!");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {

        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\
        robot.drive(gamepad1, telemetry);

        if (gamepad1.x) {
            robot.resetHeading();
        }
        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\


        //----------------------------------------------=+(Intake)+=----------------------------------------------\\
        if ((gamepad1.left_trigger > 0.1)&&(Math.abs(gamepad1.right_trigger ) < 0.1)) {//This is a precaution
            component.intakeStone();//Intake stone
        } else if ((gamepad1.right_trigger > 0.1)&&(Math.abs(gamepad1.left_trigger ) < 0.1)) {//This is a precaution
            component.ejectStone();//Eject stone
        } else {
            component.stopStone();//Stop power to the stone intake
        }
        //----------------------------------------------=+(Intake)+=----------------------------------------------\\


        //TODO: Check the correct encoder values for max and min. Check the correct motor directions
        //----------------------------------------------=+(Lift)+=----------------------------------------------\\
        component.liftControlUp(gamepad2);

        //----------------------------------------------=+(Lift)+=----------------------------------------------\\


//        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\
//        if (gamepad1.dpad_left) {
//            turn -= 0.05;
//            component.rotate(yeet);
//        } else if (gamepad1.dpad_right) {
//            turn += 0.05;
//            component.rotate(yeet);
//        }
//        if (gamepad1.y) {
//            component.release();
//        } else if (gamepad1.a) {
//            component.grab();
//        }
//        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\

    }
}