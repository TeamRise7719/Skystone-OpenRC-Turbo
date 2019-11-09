//package org.firstinspires.ftc.teamcode.JordansAutonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//
//import org.firstinspires.ftc.teamcode.RisePlayground.Subsystems.AutonomousPathing;
//import org.firstinspires.ftc.teamcode.Subsystems.Driving.RobotComponents;
//import org.firstinspires.ftc.teamcode.Subsystems.Driving.SeansEncLibrary;
//import org.firstinspires.ftc.teamcode.Subsystems.Driving.Drivetrain;
//import org.firstinspires.ftc.teamcode.Subsystems.Transitioning.AutoTransitioner;
//import org.firstinspires.ftc.teamcode.Subsystems.VisionTargeting.SkystoneDetectionPhone;
//import org.firstinspires.ftc.teamcode.Subsystems.VisionTargeting.SkystoneDetectionWebcam;
//
//@Autonomous(name = "Blue_Auto_V1", group = "Autonomous")
//public class Blue_Auto_V1 extends LinearOpMode {
//
//    private Drivetrain robot;
//    boolean isSkystone;
//    private RobotComponents component;
//    private AutonomousPathing path;
//    int position;
//    SeansEncLibrary enc;
//    SkystoneDetectionPhone phone;
//    SkystoneDetectionWebcam webcam;
//    DcMotor intake;
//    DcMotor left_back_drive;
//    DcMotor left_front_drive;
//    DcMotor right_back_drive;
//    DcMotor right_front_drive;
//
//
//    double inches = COUNTS_PER_INCH;
//    private static final double     COUNTS_PER_MOTOR_REV    = 537.6;
//    private static final double     EXTERNAL_GEAR_RATIO    = (25/32);     // This is < 1.0 if geared UP
//    private static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
//    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * EXTERNAL_GEAR_RATIO) /
//            (WHEEL_DIAMETER_INCHES * Math.PI);
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
//        component = new RobotComponents(hardwareMap,telemetry);
//
//        waitForStart();
//
//
//        while (right_front_drive.getCurrentPosition()< 10*COUNTS_PER_INCH) {
//            right_front_drive.setPower(0.5);
//            left_front_drive.setPower(0.5);
//            right_back_drive.setPower(0.5);
//            left_back_drive.setPower(0.5);
//        }
//        right_front_drive.setPower(0);
//        left_front_drive.setPower(0);
//        right_back_drive.setPower(0);
//        left_back_drive.setPower(0);
//        while (left_front_drive.getCurrentPosition()< 10*COUNTS_PER_INCH){
//            right_front_drive.setPower(0.5);
//            left_front_drive.setPower(0.5);
//            right_back_drive.setPower(0.5);
//            left_back_drive.setPower(0.5);
//        }
//        right_front_drive.setPower(0);
//        left_front_drive.setPower(0);
//        right_back_drive.setPower(0);
//        left_back_drive.setPower(0);
//        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//
//
////        //moves toward skyblock
//        right_front_drive.setTargetPosition(32*inches);
//        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right_front_drive.setPower(0.5);
//        left_front_drive.setTargetPosition(32*inches);
//        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        left_front_drive.setPower(0.5);
//        right_back_drive.setTargetPosition(32*inches);
//        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right_back_drive.setPower(0.5);
//        left_back_drive.setTargetPosition(32*inches);
//        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        left_back_drive.setPower(0.5);
//
//
//
////        //vision code goes here
//        while (phone.TFdetect(isSkystone) == false) {
//            path.runPurePursuitPath();
//
//
//        }
//
////        //intakes block
//          component.intakeStone();
//          sleep(1900);
////        //move to toward bridge
//        right_front_drive.setTargetPosition(-18*inches);
//        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right_front_drive.setPower(0.5);
//        left_front_drive.setTargetPosition(-18*inches);
//        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        left_front_drive.setPower(0.5);
//        right_back_drive.setTargetPosition(-18*inches);
//        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right_back_drive.setPower(0.5);
//        left_back_drive.setTargetPosition(-18*inches);
//        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        left_back_drive.setPower(0.5);
//
//        enc.gyroHold(0.5,45,2.5);
//
//        right_front_drive.setTargetPosition(27*inches);
//        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right_front_drive.setPower(0.5);
//        left_front_drive.setTargetPosition(27*inches);
//        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        left_front_drive.setPower(0.5);
//        right_back_drive.setTargetPosition(27*inches);
//        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right_back_drive.setPower(0.5);
//        left_back_drive.setTargetPosition(27*inches);
//        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        left_back_drive.setPower(0.5);
//
//
//// other side
//
//        //24
//        //turn 120
//        // drive 40
//        //turn 45
//        //drive 32
//        //vision
//        //intake
//        //back 32
//        //turn 120
//        //drive 45
//        AutoTransitioner.transitionOnStop(this, "BasicTeleOp");//Transition to TeleOp
//    }
//
//
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
