package org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Util.Threading;
import org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Memes.RobotMedia;

public class SeansEncLibrary {//TODO:Change this class to work using the new odometers.

    private DcMotor left_back_drive;
    private DcMotor left_front_drive;
    private DcMotor right_back_drive;
    private DcMotor right_front_drive;
    private SynchronousPID turnPID;
    private SynchronousPID leftFrontDrivingPID;
    private SynchronousPID leftBackDrivingPID;
    private SynchronousPID rightFrontDrivingPID;
    private SynchronousPID rightBackDrivingPID;
    private RobotMedia areYouSpinningYet;

    public BNO055IMU gyro;
    private Orientation gyro_angle;

    private Telemetry telemetry;
    private LinearOpMode linearOpMode;

//    I2CXL ultrasonicFront;
//    I2CXL ultrasonicBack;

    private double     COUNTS_PER_MOTOR_REV    = 537.6;
    private double     EXTERNAL_GEAR_RATIO     = 0.78125;     // This is < 1.0 if geared UP
    private double     WHEEL_DIAMETER_INCHES   = 3.937;     // For figuring circumference
    private double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * EXTERNAL_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415));

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public final double     DRIVE_SPEED             = 0.8;     // Nominal speed
    public final double     DRIVE_SPEED_SLOW             = 0.4;     // Slower speed for better accuracy.

    public  final double     TURN_SPEED              = 0.8;     // Nominal half speed for better accuracy.

    private static final double     HEADING_THRESHOLD       = 0.1;      // As tight as we can make it with an integer gyro
    private static final int     ENCODER_THRESHOLD       = 10;      // As tight as we can make it with an integer gyro


    private static final double     P_TURN_COEFF            = 0.008;//.008     // Larger is more responsive, but also less stable
    private static final double     I_TURN_COEFF            = 00.00000000005;//  // Larger is more responsive, but also less stable
    private static final double     D_TURN_COEFF            = 0.000001;//0.000001     // Larger is more responsive, but also less stable


    private static final double     P_DRIVE_COEFF           = 0.0015;     // Larger is more responsive, but also less stable
    private static final double     I_DRIVE_COEFF           = 0.00000000025 ;     // Larger is more responsive, but also less stable
    private static final double     D_DRIVE_COEFF           = 0.0001;     // Larger is more responsive, but also less stable

    public SeansEncLibrary(HardwareMap hardwareMap, Telemetry tel, LinearOpMode opMode) {
        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");

        left_back_drive = hardwareMap.dcMotor.get("leftB");
        left_front_drive = hardwareMap.dcMotor.get("leftF");
        right_back_drive = hardwareMap.dcMotor.get("rightB");
        right_front_drive = hardwareMap.dcMotor.get("rightF");

        telemetry = tel;
        linearOpMode = opMode;
        areYouSpinningYet = new RobotMedia(hardwareMap);
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
        leftFrontDrivingPID = new SynchronousPID(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
        leftBackDrivingPID = new SynchronousPID(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
        rightFrontDrivingPID = new SynchronousPID(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
        rightBackDrivingPID = new SynchronousPID(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);

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


    /**
     * Function to move forwards, backwards, left, or right using PID.
     * @param distance Distance in inches to move. A positive distance makes the robot move
     *                 forwards and a negative distance makes the robot move backwards.
     *                 If strafe is enabled then forwards is right and backwards is left.
     * @param steeringToggle Steering to stay in a straight line. Just put true or false here.
     * @param strafe Option to strafe instead of moving forwards/backwards. Put true or false here.
     */
    public void steeringDrive(double distance,
                              boolean steeringToggle,
                              boolean strafe) {

        //TODO: Figure out what to change so that this isn't necessary


        distance = -distance;
        int strafeDirection = 0;
        if (distance > 0) { strafeDirection = 1;} else if (distance < 0) { strafeDirection = -1;}

        double steeringSpeed;
        double leftFrontDriveSpeed;
        double leftBackDriveSpeed;
        double rightFrontDriveSpeed;
        double rightBackDriveSpeed;

        leftFrontDrivingPID.reset();
        leftBackDrivingPID.reset();
        rightFrontDrivingPID.reset();
        rightBackDrivingPID.reset();
        turnPID.reset();

        if (!strafe) {
            int moveCounts = ((int) (distance * COUNTS_PER_INCH));
            int newFrontLeftTarget = left_front_drive.getCurrentPosition() + moveCounts;
            int newBackLeftTarget = left_back_drive.getCurrentPosition() + moveCounts;
            int newFrontRightTarget = right_front_drive.getCurrentPosition() + moveCounts;
            int newBackRightTarget = right_back_drive.getCurrentPosition() + moveCounts;

            int encFrontLeft;
            int encBackLeft;
            int encFrontRight;
            int encBackRight;
            ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            etime.reset();


            turnPID.setContinuous(true);
            gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            turnPID.setSetpoint(gyro_angle.firstAngle);
            turnPID.setOutputRange(-0.4, 0.4);


            leftFrontDrivingPID.setContinuous(false);
            leftFrontDrivingPID.setSetpoint(newFrontLeftTarget);
            leftFrontDrivingPID.setOutputRange(-0.4, 0.4);

            leftBackDrivingPID.setContinuous(false);
            leftBackDrivingPID.setSetpoint(newBackLeftTarget);
            leftBackDrivingPID.setOutputRange(-0.4, 0.4);


            rightFrontDrivingPID.setContinuous(false);
            rightFrontDrivingPID.setSetpoint(newFrontRightTarget);
            rightFrontDrivingPID.setOutputRange(-0.4, 0.4);

            rightBackDrivingPID.setContinuous(false);
            rightBackDrivingPID.setSetpoint(newBackRightTarget);
            rightBackDrivingPID.setOutputRange(-0.4, 0.4);

            while (linearOpMode.opModeIsActive()) {

                if ((((Math.abs(newBackLeftTarget - left_back_drive.getCurrentPosition())) < ENCODER_THRESHOLD)
                        && (((Math.abs(newBackRightTarget - right_back_drive.getCurrentPosition())) < ENCODER_THRESHOLD)))) {
                    break;
                }

                if (steeringToggle) {
                    gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    turnPID.calcInit();
                    steeringSpeed = turnPID.timedCalculate(gyro_angle.firstAngle);
                } else {
                    steeringSpeed = 0;
                }

                encFrontLeft = left_front_drive.getCurrentPosition();
                leftFrontDrivingPID.calcInit();
                leftFrontDriveSpeed = leftFrontDrivingPID.timedCalculate(encFrontLeft);

                encBackLeft = left_back_drive.getCurrentPosition();
                leftBackDrivingPID.calcInit();
                leftBackDriveSpeed = leftBackDrivingPID.timedCalculate(encBackLeft);

                encFrontRight = right_front_drive.getCurrentPosition();
                rightFrontDrivingPID.calcInit();
                rightFrontDriveSpeed = rightFrontDrivingPID.timedCalculate(encFrontRight);

                encBackRight = right_back_drive.getCurrentPosition();
                rightBackDrivingPID.calcInit();
                rightBackDriveSpeed = rightBackDrivingPID.timedCalculate(encBackRight);


                left_back_drive.setPower(leftBackDriveSpeed + steeringSpeed);
                left_front_drive.setPower(leftBackDriveSpeed + steeringSpeed);
                right_back_drive.setPower(rightBackDriveSpeed - steeringSpeed);
                right_front_drive.setPower(rightFrontDriveSpeed - steeringSpeed);

//                telemetry.addData("LErr/RErr", "%s:%s", newLeftTarget - encLeft, newRightTarget - encRight);
//                telemetry.addData("HeadingErr/CurrentHeading", "%f:%f", turnPID.getError(), gyro_angle.firstAngle);
//                telemetry.addData("LSpd/RSpd/Steer", "%f:%f:%f", leftDriveSpeed, rightDriveSpeed, steeringSpeed);
//                telemetry.update();
            }

            stop_all_motors();
        }
        if (strafe) {

            int moveCounts = ((int) (distance * COUNTS_PER_INCH));
            int newBackLeftTarget = 0; //= left_back_drive.getCurrentPosition() + (-strafeDirection * moveCounts);
            int newFrontLeftTarget = 0; //= left_front_drive.getCurrentPosition() + (strafeDirection * moveCounts);
            int newBackRightTarget = 0; //= right_back_drive.getCurrentPosition() + (strafeDirection * moveCounts);
            int newFrontRightTarget = 0; //= right_front_drive.getCurrentPosition() + (-strafeDirection * moveCounts);

            if (strafeDirection == 1){
                newBackLeftTarget = left_back_drive.getCurrentPosition() + (-strafeDirection * moveCounts);
                newFrontLeftTarget = left_front_drive.getCurrentPosition() + (strafeDirection * moveCounts);
                newBackRightTarget = right_back_drive.getCurrentPosition() + (strafeDirection * moveCounts);
                newFrontRightTarget = right_front_drive.getCurrentPosition() + (-strafeDirection * moveCounts);
            } else if (strafeDirection == -1){
                newBackLeftTarget = left_back_drive.getCurrentPosition() + (strafeDirection * moveCounts);
                newFrontLeftTarget = left_front_drive.getCurrentPosition() + (-strafeDirection * moveCounts);
                newBackRightTarget = right_back_drive.getCurrentPosition() + (-strafeDirection * moveCounts);
                newFrontRightTarget = right_front_drive.getCurrentPosition() + (strafeDirection * moveCounts);
            }

            int encFrontLeft;
            int encBackLeft;
            int encFrontRight;
            int encBackRight;
            ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            etime.reset();


            turnPID.setContinuous(true);
            gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            turnPID.setSetpoint(gyro_angle.firstAngle);
            turnPID.setOutputRange(-0.2, 0.2);


            leftFrontDrivingPID.setContinuous(false);
            leftFrontDrivingPID.setSetpoint(newFrontLeftTarget);
            leftFrontDrivingPID.setOutputRange(-0.2, 0.2);

            leftBackDrivingPID.setContinuous(false);
            leftBackDrivingPID.setSetpoint(newBackLeftTarget);
            leftBackDrivingPID.setOutputRange(-0.2, 0.2);


            rightFrontDrivingPID.setContinuous(false);
            rightFrontDrivingPID.setSetpoint(newFrontRightTarget);
            rightFrontDrivingPID.setOutputRange(-0.2, 0.2);

            rightBackDrivingPID.setContinuous(false);
            rightBackDrivingPID.setSetpoint(newBackRightTarget);
            rightBackDrivingPID.setOutputRange(-0.2, 0.2);

            while (linearOpMode.opModeIsActive()) {

                if ((((Math.abs(newBackLeftTarget - left_back_drive.getCurrentPosition())) < 10)
                        && (((Math.abs(newBackRightTarget - right_back_drive.getCurrentPosition())) < 10)))) {
                    break;
                }

                if (steeringToggle) {
                    gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    steeringSpeed = turnPID.calculate(gyro_angle.firstAngle);
                } else {
                    steeringSpeed = 0;
                }

                encFrontLeft = left_front_drive.getCurrentPosition();
                leftFrontDrivingPID.calcInit();
                leftFrontDriveSpeed = leftFrontDrivingPID.timedCalculate(encFrontLeft);

                encBackLeft = left_back_drive.getCurrentPosition();
                leftBackDrivingPID.calcInit();
                leftBackDriveSpeed = leftBackDrivingPID.timedCalculate(encBackLeft);

                encFrontRight = right_front_drive.getCurrentPosition();
                rightFrontDrivingPID.calcInit();
                rightFrontDriveSpeed = rightFrontDrivingPID.timedCalculate(encFrontRight);

                encBackRight = right_back_drive.getCurrentPosition();
                rightBackDrivingPID.calcInit();
                rightBackDriveSpeed = rightBackDrivingPID.timedCalculate(encBackRight);


                //In my head this works although steering toggle will be an issue.
//                left_back_drive.setPower(-strafeDirection * (-leftBackDriveSpeed + steeringSpeed));
//                left_front_drive.setPower(strafeDirection * (leftFrontDriveSpeed + steeringSpeed));
//                right_back_drive.setPower(strafeDirection * (rightBackDriveSpeed - steeringSpeed));
//                right_front_drive.setPower(-strafeDirection * (-rightFrontDriveSpeed - steeringSpeed));

                if (strafeDirection == 1){
                    left_back_drive.setPower(-strafeDirection * (-leftBackDriveSpeed + steeringSpeed));
                    left_front_drive.setPower(strafeDirection * (leftFrontDriveSpeed + steeringSpeed));
                    right_back_drive.setPower(strafeDirection * (rightBackDriveSpeed - steeringSpeed));
                    right_front_drive.setPower(-strafeDirection * (-rightFrontDriveSpeed - steeringSpeed));
                } else if (strafeDirection == -1){
                    left_back_drive.setPower(strafeDirection * (-leftBackDriveSpeed + steeringSpeed));
                    left_front_drive.setPower(-strafeDirection * (leftFrontDriveSpeed + steeringSpeed));
                    right_back_drive.setPower(-strafeDirection * (rightBackDriveSpeed - steeringSpeed));
                    right_front_drive.setPower(strafeDirection * (-rightFrontDriveSpeed - steeringSpeed));
                }

//                telemetry.addData("LErr/RErr", "%s:%s", newLeftTarget - encLeft, newRightTarget - encRight);
//                telemetry.addData("HeadingErr/CurrentHeading", "%f:%f", turnPID.getError(), gyro_angle.firstAngle);
//                telemetry.addData("LSpd/RSpd/Steer", "%f:%f:%f", leftDriveSpeed, rightDriveSpeed, steeringSpeed);
//                telemetry.update();
            }
            stop_all_motors();
        }
    }

    public void gyroTurn(double speed, double angle) {
        turnPID.reset();
        turnPID.setContinuous(true);
        turnPID.setSetpoint(-angle);
        turnPID.setOutputRange(-speed,speed);



        boolean doneTurning;

        ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        etime.reset();

        // keep looping while we are still active, and not on heading.
        while (linearOpMode.opModeIsActive() /*&& etime.time()<3.5*/) {
             doneTurning = onHeading(angle);
             if (!doneTurning) {
                 areYouSpinningYet.rightRound(true);
             }
             if(doneTurning){
                 areYouSpinningYet.rightRound(false);
                 break;
             }
        }

        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

    }

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

    boolean onHeading(double angle) {

        double motorSpeed;
        gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        turnPID.calcInit();
        motorSpeed = turnPID.timedCalculate(gyro_angle.firstAngle);

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

    //TODO: Can we delete all of this?
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