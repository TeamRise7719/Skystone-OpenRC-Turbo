package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Driving.Drivetrain;

/*
 * Created by Sean Cardosi on 9/22/2019.
 */
@TeleOp(name = "BasicTeleOp", group = "Drive")
public class BasicTeleOp extends OpMode {

    private Drivetrain robot;

    private DcMotor intake;

    private boolean isReady = false;

    @Override
    public void init() {

        //Initialize robot
        robot = new Drivetrain(hardwareMap, telemetry);
        robot.runUsingEncoders();

        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

        if (gamepad1.a){
            intake.setPower(-1.0);
        }
        if (gamepad1.b){
            intake.setPower(1.0);
        }
        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\
    }
}