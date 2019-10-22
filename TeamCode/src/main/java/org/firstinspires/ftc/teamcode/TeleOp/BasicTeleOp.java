package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.Drivetrain;

/*
 * Created by Sean Cardosi on 9/22/2019.
 */
@TeleOp(name = "BasicTeleOp", group = "Drive")
public class BasicTeleOp extends OpMode {//TODO: We need a robot naming convention... we just need a robot name.

    private Drivetrain robot;

    private RobotComponents component;

    private boolean isReady = false;

    @Override
    public void init() {

        //Initialize robot
        robot = new Drivetrain(hardwareMap, telemetry);
        robot.runUsingEncoders();

        component = new RobotComponents(hardwareMap, telemetry);

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


        //TODO: This will need to be double checked. Unsure of the correct trigger values. Check motor direction.
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
        component.liftControlUp(gamepad1);
        component.liftControlDown(gamepad1);
        //----------------------------------------------=+(Lift)+=----------------------------------------------\\
    }
}