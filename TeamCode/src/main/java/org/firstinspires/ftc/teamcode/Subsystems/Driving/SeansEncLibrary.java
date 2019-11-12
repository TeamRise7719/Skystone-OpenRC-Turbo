package org.firstinspires.ftc.teamcode.Subsystems.Driving;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import android.util.Log;
import org.firstinspires.ftc.teamcode.Subsystems.Util.Threading;
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class SeansEncLibrary {//TODO:Change this class to work using the new odometers.

    DcMotor left_back_drive;
    DcMotor left_front_drive;
    DcMotor right_back_drive;
    DcMotor right_front_drive;
    SynchronousPID turnPID;
    SynchronousPID leftDrivingPID;
    SynchronousPID rightDrivingPID;

    public BNO055IMU gyro;
    public Orientation gyro_angle;

    Telemetry telemetry;
    LinearOpMode linearOpMode;

    Robot robot;

//    I2CXL ultrasonicFront;
//    I2CXL ultrasonicBack;

    public double     COUNTS_PER_MOTOR_REV    = 537.6;
    public double     EXTERNAL_GEAR_RATIO     = 0.78125;     // This is < 1.0 if geared UP
    public double     WHEEL_DIAMETER_INCHES   = 3.937;     // For figuring circumference
    public double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * EXTERNAL_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415));

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public  final double     DRIVE_SPEED             = 0.8;     // Nominal speed
    public  final double     DRIVE_SPEED_SLOW             = 0.4;     // Slower speed for better accuracy.

    public  final double     TURN_SPEED              = 0.8;     // Nominal half speed for better accuracy.

    public static final double     HEADING_THRESHOLD       = 0.05;      // As tight as we can make it with an integer gyro
    public static final int     ENCODER_THRESHOLD       = 20;      // As tight as we can make it with an integer gyro


    private static  double     P_TURN_COEFF            = 0.03;//.007     // Larger is more responsive, but also less stable
    private static  double     I_TURN_COEFF            = 0.0015;//.00001   // Larger is more responsive, but also less stable
    private static  double     D_TURN_COEFF            = 0.000075;//.000003     // Larger is more responsive, but also less stable


    private static final double     P_DRIVE_COEFF           = 0.0005;     // Larger is more responsive, but also less stable
    private static final double     I_DRIVE_COEFF           = 0.000005;     // Larger is more responsive, but also less stable
    private static final double     D_DRIVE_COEFF           = 0.0000006;     // Larger is more responsive, but also less stable
    public int LEFT = -1;
    public int RIGHT = 1;

    public SeansEncLibrary(HardwareMap hardwareMap, Telemetry tel, LinearOpMode opMode) {
        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");

        left_back_drive = hardwareMap.dcMotor.get("leftB");
        left_front_drive = hardwareMap.dcMotor.get("leftF");
        right_back_drive = hardwareMap.dcMotor.get("rightB");
        right_front_drive = hardwareMap.dcMotor.get("rightF");

        telemetry = tel;
        linearOpMode = opMode;
    }


     public void init(){

        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         Threading.startThread(() -> {
             try {
                 Log.i("IMUt", "starting imu stuff");

                 BNO055IMU.Parameters param = new BNO055IMU.Parameters();
                 param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                 param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                 param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                 gyro.initialize(param);
                 gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


             } catch (Exception ex) {
                 telemetry.addData("Error:", ex.toString());
             }
         });

//         BNO055IMU.Parameters param = new BNO055IMU.Parameters();
//         param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//         param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//         param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//         gyro.initialize(param);
//         gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        turnPID = new SynchronousPID(P_TURN_COEFF, I_TURN_COEFF, D_TURN_COEFF);
        leftDrivingPID = new SynchronousPID(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
        rightDrivingPID = new SynchronousPID(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);

        turnPID.setOutputRange(-TURN_SPEED, TURN_SPEED);
        turnPID.setInputRange(-180, 180);
    }


    //Stop All Motors
    public void stop_all_motors(){
        left_back_drive.setPower(0);
        right_back_drive.setPower(0);
        left_front_drive.setPower(0);
        right_front_drive.setPower(0);
    }

    public void steeringDrive(double distance,
                              boolean steeringToggle){

        distance = -distance;

        double steeringSpeed;
        double leftDriveSpeed;
        double rightDriveSpeed;

        leftDrivingPID.reset();
        rightDrivingPID.reset();
        turnPID.reset();

        int moveCounts = ((int)(distance * COUNTS_PER_INCH));
        int newLeftTarget = left_back_drive.getCurrentPosition() + moveCounts;
        int newRightTarget = right_back_drive.getCurrentPosition() + moveCounts;

        int encLeft;
        int encRight;
        ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        etime.reset();


        turnPID.setContinuous(true);
        gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        turnPID.setSetpoint(gyro_angle.firstAngle);
        turnPID.setOutputRange(-0.6,0.6);


        leftDrivingPID.setContinuous(false);
        leftDrivingPID.setSetpoint(newLeftTarget);
        leftDrivingPID.setOutputRange(-0.8,0.8);


        rightDrivingPID.setContinuous(false);
        rightDrivingPID.setSetpoint(newRightTarget);
        rightDrivingPID.setOutputRange(-0.8,0.8);

        int sum = 0;

        while (linearOpMode.opModeIsActive()) {

            sum++;
            if((((Math.abs(newLeftTarget-left_back_drive.getCurrentPosition()))<ENCODER_THRESHOLD)
                    && ((((Math.abs(newRightTarget-right_back_drive.getCurrentPosition()))<ENCODER_THRESHOLD))))){
                break;
            }

            if (steeringToggle){
                gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                steeringSpeed = turnPID.calculate(gyro_angle.firstAngle);
            } else {
                steeringSpeed = 0;
            }

            encLeft = left_back_drive.getCurrentPosition();
            leftDriveSpeed = leftDrivingPID.calculate(encLeft);

            encRight = right_back_drive.getCurrentPosition();
            rightDriveSpeed = rightDrivingPID.calculate(encRight);


            left_back_drive.setPower(leftDriveSpeed + steeringSpeed);
            left_front_drive.setPower(leftDriveSpeed + steeringSpeed);
            right_back_drive.setPower(rightDriveSpeed - steeringSpeed);
            right_front_drive.setPower(rightDriveSpeed - steeringSpeed);

            telemetry.addData("LErr/RErr", "%s:%s",newLeftTarget - encLeft, newRightTarget - encRight);
            telemetry.addData("HeadingErr/CurrentHeading", "%f:%f", turnPID.getError(),gyro_angle.firstAngle);
            telemetry.addData("LSpd/RSpd/Steer", "%f:%f:%f", leftDriveSpeed, rightDriveSpeed, steeringSpeed);
            telemetry.update();
        }
//        telemetry.addData("Loop total","%s",sum);
//        telemetry.update();
        stop_all_motors();

    }

    /**
     * Use PID to strafe... I'm Tired. This won't work. Please help me.
     * @param distance Distance in inches
     * @param steeringToggle true or false to stay in a straight line
     * @param direction Left should be -1. Right should be +1. You can also call LEFT or RIGHT.
     */
    public void steeringStrafe(double distance,
                               int direction,
                               boolean steeringToggle){

        distance = -distance;

        double steeringSpeed;
        double leftDriveSpeed;
        double rightDriveSpeed;

        leftDrivingPID.reset();
        rightDrivingPID.reset();
        turnPID.reset();

        int moveCounts = ((int)(distance * COUNTS_PER_INCH));
        int newLeftTarget = left_back_drive.getCurrentPosition() + (-direction * moveCounts);
        int newRightTarget = right_back_drive.getCurrentPosition() + (direction * moveCounts);

        int encLeft;
        int encRight;
        ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        etime.reset();


        turnPID.setContinuous(true);
        gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        turnPID.setSetpoint(gyro_angle.firstAngle);
        turnPID.setOutputRange(-0.6,0.6);


        leftDrivingPID.setContinuous(false);
        leftDrivingPID.setSetpoint(newLeftTarget);
        leftDrivingPID.setOutputRange(-0.8,0.8);


        rightDrivingPID.setContinuous(false);
        rightDrivingPID.setSetpoint(newRightTarget);
        rightDrivingPID.setOutputRange(-0.8,0.8);

        int sum = 0;

        while (linearOpMode.opModeIsActive()) {

            sum++;
            if((((Math.abs(newLeftTarget-left_back_drive.getCurrentPosition()))<ENCODER_THRESHOLD)
                    && ((((Math.abs(newRightTarget-right_back_drive.getCurrentPosition()))<ENCODER_THRESHOLD))))){
                break;
            }

            if (steeringToggle){
                gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                steeringSpeed = turnPID.calculate(gyro_angle.firstAngle);
            } else {
                steeringSpeed = 0;
            }

            encLeft = left_back_drive.getCurrentPosition();
            leftDriveSpeed = leftDrivingPID.calculate(encLeft);

            encRight = right_back_drive.getCurrentPosition();
            rightDriveSpeed = rightDrivingPID.calculate(encRight);


            //In my head this works although steering toggle will be an issue.
            left_back_drive.setPower(-direction * (leftDriveSpeed + steeringSpeed));
            left_front_drive.setPower(direction * (leftDriveSpeed + steeringSpeed));
            right_back_drive.setPower(direction * (rightDriveSpeed - steeringSpeed));
            right_front_drive.setPower(-direction * (rightDriveSpeed - steeringSpeed));

            telemetry.addData("LErr/RErr", "%s:%s",newLeftTarget - encLeft, newRightTarget - encRight);
            telemetry.addData("HeadingErr/CurrentHeading", "%f:%f", turnPID.getError(),gyro_angle.firstAngle);
            telemetry.addData("LSpd/RSpd/Steer", "%f:%f:%f", leftDriveSpeed, rightDriveSpeed, steeringSpeed);
            telemetry.update();
        }
//        telemetry.addData("Loop total","%s",sum);
//        telemetry.update();
        stop_all_motors();

    }



    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {
        turnPID.reset();
        turnPID.setContinuous(true);
        turnPID.setSetpoint(-angle);
        turnPID.setOutputRange(-speed,speed);



        boolean doneTurning = false;

        ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        etime.reset();

        // keep looping while we are still active, and not on heading.
        while (linearOpMode.opModeIsActive() && etime.time()<3.5) {
             doneTurning = onHeading(angle);
             if(doneTurning){
                 break;
             }
        }

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        turnPID.reset();
        turnPID.setSetpoint(angle);
        turnPID.setOutputRange(-speed,speed);
        turnPID.setDeadband(HEADING_THRESHOLD);

        ElapsedTime holdTimer = new ElapsedTime();


        // keep looping while we have time remaining.
        holdTimer.reset();
        while (linearOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(angle);
        }

        // Stop all motion;
        left_back_drive.setPower(0);
        left_front_drive.setPower(0);
        right_back_drive.setPower(0);
        right_front_drive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @return
     */
    boolean onHeading(double angle) {

        double motorSpeed;
        gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        motorSpeed = turnPID.calculate(gyro_angle.firstAngle);

        // Send desired speeds to motors.
        left_front_drive.setPower(motorSpeed);
        left_back_drive.setPower(motorSpeed);
        right_front_drive.setPower(-motorSpeed);
        right_back_drive.setPower(-motorSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/Angle", "%5.2f:%5.2f", turnPID.getError(),gyro_angle.firstAngle);
        telemetry.addData("Coef ", turnPID.getState());
        telemetry.addData("Speed.", "%5.2f:%5.2f", -motorSpeed, motorSpeed);
        telemetry.update();

        return turnPID.onTarget(HEADING_THRESHOLD);
    }



//    /**
//     * getError determines the error between the target angle and the robot's current heading
//     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
//     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
//     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
//     */
//
//    public double getErrorUltra(double targetDistance, boolean isBack) {
//
//        double ultrasonicValue;
//        double robotError;
//
//        // calculate error
//        if (isBack) {
//            //use the back sensor
//            ultrasonicValue = (ultrasonicBack.getDistance() / 2.54);
//
//        } else {
//
//            //use the front sensor
//            ultrasonicValue = (ultrasonicFront.getDistance() / 2.54);
//
//        }
//        robotError = targetDistance - ultrasonicValue;
//
//        return robotError;
//    }
//    public void UltrasonicGyroDrive(
//                                    double distance,
//                                    double angle,
//                                    boolean steeringToggle,
//                                    double UltraTolerance,
//                                    boolean isBack,
//                                    double Timeout) {
//        double speed;
//        double max;
//        double error;
//        double steer;
//        double leftSpeed;
//        double rightSpeed;
//        double ultraError;
//
//
//        // Ensure that the opmode is still active
//        if (linearOpMode.opModeIsActive()) {
//            left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            ultraError = getErrorUltra(distance,isBack);
//
//            ElapsedTime etime = new ElapsedTime();
//            etime.reset();
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while ((etime.time() < Timeout) && (linearOpMode.opModeIsActive()) /* && (java.lang.Math.abs(ultraError) > UltraTolerance)*/ ){
//                // adjust relative speed based on heading error.
//                ultraError = getErrorUltra(distance,isBack);
//                speed = ultraError*ULTRA_COEFF;
//
//                error = getError(angle);
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance > 0) {
//                    steer *= -1.0;
//                }
//
//                if (steeringToggle) {
//                    leftSpeed = speed - steer;
//                    rightSpeed = speed + steer;
//                } else {
//                    leftSpeed = speed;
//                    rightSpeed = speed;
//                }
//
//                // Normalize speeds if either one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1) {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                if (leftSpeed > 0.8){
//                    leftSpeed = 0.8;
//                } else if (leftSpeed < -0.8){
//                    leftSpeed = -0.8;
//                }
//
//                if (rightSpeed > 0.8){
//                    rightSpeed = 0.8;
//                } else if (rightSpeed < -0.8){
//                    rightSpeed = -0.8;
//                }
//
//                if(!isBack){
//                    leftSpeed = leftSpeed*-1;
//                    rightSpeed = rightSpeed*-1;
//                }
//
//                if((isBack)&&(ultrasonicBack.getError())){
//                    leftSpeed = 0;
//                    rightSpeed = 0;
//                }
//                else if((!isBack)&&(ultrasonicFront.getError())){
//                    leftSpeed = 0;
//                    rightSpeed = 0;
//                }
//
//                left_back_drive.setPower(leftSpeed);
//                left_front_drive.setPower(leftSpeed);
//                right_back_drive.setPower(rightSpeed);
//                right_front_drive.setPower(rightSpeed);
//
//
//                // Display drive status for the driver.
//                telemetry.addData("Target", distance);
//                telemetry.addData("Error", "%5.1f", ultraError);
//                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            left_back_drive.setPower(0);
//            left_front_drive.setPower(0);
//            right_back_drive.setPower(0);
//            right_front_drive.setPower(0);
//        }
//    }
//
//        public void gyroDrive ( double speed,
//                            double distance,
//                            double angle,
//                            boolean steeringToggle) {
//
//        int     newLeftTarget;
//        int     newRightTarget;
//        int     moveCounts;
//        double  max;
//        double  error;
//        double  steer;
//        double  leftSpeed;
//        double  rightSpeed;
//
//        // Ensure that the opmode is still active
//        if (linearOpMode.opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            moveCounts = (int)(distance * COUNTS_PER_INCH);
//            newLeftTarget = left_back_drive.getCurrentPosition() + moveCounts;
//            newRightTarget = right_back_drive.getCurrentPosition() + moveCounts;
//
//            // start motion.
//            speed = Range.clip(speed, -1.0, 1.0);
//            left_back_drive.setPower(speed);
//            left_front_drive.setPower(speed);
//            right_back_drive.setPower(speed);
//            right_front_drive.setPower(speed);
//
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (linearOpMode.opModeIsActive() && (Math.abs(newLeftTarget-left_back_drive.getCurrentPosition())>ENCODER_THRESHOLD
//                   || Math.abs(newRightTarget-right_back_drive.getCurrentPosition())>ENCODER_THRESHOLD) ) {
//
//                // adjust relative speed based on heading error.
////                error = getError(angle);
//                error = 0;
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0) {
//                    steer *= -1.0;
//                }
//
//                if(steeringToggle) {
//                    leftSpeed = speed - steer;
//                    rightSpeed = speed + steer;
//                }
//                else {
//                    leftSpeed = speed;
//                    rightSpeed = speed;
//                }
//
//                // Normalize speeds if either one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                left_back_drive.setPower(-leftSpeed);
//                left_front_drive.setPower(-leftSpeed);
//                right_back_drive.setPower(-rightSpeed);
////                right_front_drive.setPower(-rightSpeed);
////
////                // Display drive status for the driver.
////                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
////                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
////                telemetry.addData("Actual",  "%7d:%7d",      left_back_drive.getCurrentPosition(),
////                        right_back_drive.getCurrentPosition());
////                telemetry.addData("Speed",   "%5.2f:%5.2f",  -leftSpeed, -rightSpeed);
////                telemetry.update();
////            }
////
////            // Stop all motion;
////            left_back_drive.setPower(0);
////            left_front_drive.setPower(0);
////            right_back_drive.setPower(0);
////            right_front_drive.setPower(0);
////
////            telemetry.addData("LErr/RErr", "%5.2f", angle);
////            telemetry.addData("HeadingErr/CurrentHeading", "%5.2f:%5.2f", turnPID.getError(),gyro_angle.firstAngle);
////            telemetry.addData("Coef ", turnPID.getState());
//////            telemetry.addData("Speed.", "%5.2f:%5.2f", -motorSpeed, motorSpeed);
////            telemetry.update();
////
////        }
////    }
//
//
}