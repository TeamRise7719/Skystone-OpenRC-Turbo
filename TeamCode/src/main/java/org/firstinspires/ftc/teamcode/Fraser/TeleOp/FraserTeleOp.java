package org.firstinspires.ftc.teamcode.Fraser.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserDrivetrain;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Media.RobotMedia;

/*
 * Created by Sean Cardosi on 9/22/2019.
 */
@TeleOp(name = "Fraser TeleOp", group = "Fraser")
public class FraserTeleOp extends OpMode {


    private FraserDrivetrain robot;
    private FraserComponents component;
    //D: Don't do it...
    private RobotMedia media;//:D

    private boolean isReady = false;

    @Override
    public void init() {

        //Initialize robot
        robot = new FraserDrivetrain(hardwareMap);
        robot.runUsingEncoders();

        component = new FraserComponents(hardwareMap);
        component.foundationInit();
        //D: Don't do it...

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
    public void start() {
        super.start();
        media = new RobotMedia(hardwareMap);//:D
        media.startTimer();
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
            component.ejectStone();//Intake stone
        } else if ((gamepad1.right_trigger > 0.1)&&(Math.abs(gamepad1.left_trigger ) < 0.1)) {//This is a precaution
            component.intakeStone();//Eject stone
        } else {
            component.stopStone();//Stop power to the stone intake
        }
        //----------------------------------------------=+(Intake)+=----------------------------------------------\\


        //----------------------------------------------=+(Lift)+=----------------------------------------------\\
        component.liftManual(gamepad2);
        //----------------------------------------------=+(Lift)+=----------------------------------------------\\


        //----------------------------------------------=+(Block Build)+=----------------------------------------------\\
        if (gamepad2.right_bumper){
            component.clawGrab();
        }

        if (gamepad2.left_bumper){
            component.autoClaw();
        }

        if(gamepad2.y)
        {
            component.shoulderDown();
        }

        if(gamepad2.a)
        {
            component.shoulderUp();
        }
        //----------------------------------------------=+(Block Build)+=----------------------------------------------\\


        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\
        if (gamepad1.a) {
            component.foundationRelease();
        }
        if (gamepad1.y) {
            component.foundationGrab();
        }
        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\

        //D: Don't do it...
        //----------------------------------------------=+(Media)+=----------------------------------------------\\
        media.playSounds(gamepad1, gamepad2, hardwareMap);//:D
        //----------------------------------------------=+(Media)+=----------------------------------------------\\
    }
}