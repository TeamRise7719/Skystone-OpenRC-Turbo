package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SeansSpace.Subsystems.RobotMedia;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.Drivetrain;
import com.qualcomm.robotcore.hardware.Servo;
/*
 * Created by Sean Cardosi on 9/22/2019.
 */
@TeleOp(name = "BasicTeleOp", group = "Drive")
public class BasicTeleOp extends OpMode {

    Servo lgrab , rgrab;
    private Drivetrain robot;
    private RobotComponents component;
    private RobotMedia media;//:D

    private boolean isReady = false;

    double turn = 0;

    @Override
    public void init() {

        lgrab = hardwareMap.servo.get("lgrab");
        rgrab = hardwareMap.servo.get("rgrab");
        rgrab.setDirection(Servo.Direction.REVERSE);

        //Initialize robot
        robot = new Drivetrain(hardwareMap, telemetry);
        robot.runUsingEncoders();

        component = new RobotComponents(hardwareMap, telemetry);

        media = new RobotMedia(hardwareMap);//:D

        isReady = true;


            lgrab.setPosition(0.25);
            rgrab.setPosition(0.25);


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

        if (gamepad1.y) {
            lgrab.setPosition(1);
            rgrab.setPosition(1);
        }
        if (gamepad1.a) {
            lgrab.setPosition(0.25);
            rgrab.setPosition(0.25);
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


        //----------------------------------------------=+(Media)+=----------------------------------------------\\
        media.playSounds(gamepad1, hardwareMap);//:D
        //----------------------------------------------=+(Media)+=----------------------------------------------\\
    }
}