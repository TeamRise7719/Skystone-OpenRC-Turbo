package org.firstinspires.ftc.teamcode.MkI.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning.AutoTransitioner;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;
import org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Driving.AutonomousPathing;

/**
 * Created by Sean Cardosi on 2019-12-13.
 */
@TeleOp(name = "Blue Two Stone",group = "MkI Blue Auto")
public class BlueOneSkystone extends LinearOpMode {

    AutonomousPathing path;
    String pose;
    ElapsedTime etime = new ElapsedTime();
    double tileWidth = 24;
    double stoneLength = 8;
    Point robotStartPose = new Point(9,9+tileWidth);

    public void waitFor(int time){
        time = time/1000;
        etime.reset();
        while ((etime.time() < time)&&(opModeIsActive())) {
            idle();
        }
    }

    @Override
    public void runOpMode() {
        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        RobotComponents mech = new RobotComponents(hardwareMap);
        enc.init();
        path = new AutonomousPathing(hardwareMap);
        path.init();

        while (!this.isStarted) {
            path.initSearch(telemetry);
        }

        path.stopSearch();

        //--------------------------=+(Start Here)+=--------------------------\\
        waitForStart();

        pose = path.findPath();





        //--------------------------=+(Left)+=--------------------------\\






        if (pose.equals("left")) {

            enc.steeringDrive(4,false,false);//Get Clear of the Wall
            robotStartPose.x += 4;//Add the change to the robot pose

            //Calculate the angle to turn to face the stone... this is for speed
            Point skystonePose = new Point(2*tileWidth,tileWidth + 2*stoneLength + stoneLength/2);
            double angleToStone = -Math.round(Math.atan2(skystonePose.y-robotStartPose.y,skystonePose.x-robotStartPose.x));
            //Calculate the distance to the stone
            double distanceToStone = Math.round(Math.hypot(skystonePose.x-robotStartPose.x,skystonePose.y-robotStartPose.y));

            enc.gyroTurn(enc.TURN_SPEED, angleToStone);//Point to the stone
            mech.intakeStone();
            waitFor(1000);//Fold out intake
            mech.ejectStone();
            waitFor(1000);//Enable intake
            enc.steeringDrive(distanceToStone+4,false,false);//Get the stone
            enc.steeringDrive(-5,false,false);//Back up to stay clear of bridge
            enc.gyroTurn(enc.TURN_SPEED,-90);//Turn towards bridge
            enc.steeringDrive(35,false,false);//Go past bridge
            mech.intakeStone();
            waitFor(1000);//Eject the stone
            enc.steeringDrive(-2,false,false);//Park






        //--------------------------=+(Middle)+=--------------------------\\






        } else if (pose.equals("middle")) {

            enc.steeringDrive(4,false,false);//Get Clear of the Wall
            robotStartPose.x += 4;//Add the change to the robot pose

            //Calculate the angle to turn to face the stone... this is for speed
            Point skystonePose = new Point(2*tileWidth,tileWidth + 1*stoneLength + stoneLength/2);
            double angleToStone = -Math.round(Math.atan2(skystonePose.y-robotStartPose.y,skystonePose.x-robotStartPose.x));
            //Calculate the distance to the stone
            double distanceToStone = Math.round(Math.hypot(skystonePose.x-robotStartPose.x,skystonePose.y-robotStartPose.y));

            enc.gyroTurn(enc.TURN_SPEED, angleToStone);//Point to the stone
            mech.intakeStone();
            waitFor(1000);//Fold out intake
            mech.ejectStone();
            waitFor(1000);//Enable intake
            enc.steeringDrive(distanceToStone+4,false,false);//Get the stone
            enc.steeringDrive(-5,false,false);//Back up to stay clear of bridge
            enc.gyroTurn(enc.TURN_SPEED,-90);//Turn towards bridge
            enc.steeringDrive(35,false,false);//Go past bridge
            mech.intakeStone();
            waitFor(1000);//Eject the stone
            enc.steeringDrive(-2,false,false);//Park






        //--------------------------=+(Right)+=--------------------------\\






        } else if (pose.equals("right")) {

            enc.steeringDrive(4,false,false);//Get Clear of the Wall
            robotStartPose.x += 4;//Add the change to the robot pose

            //Calculate the angle to turn to face the stone... this is for speed
            Point skystonePose = new Point(2*tileWidth,tileWidth + 1*stoneLength + stoneLength/2);
            double angleToStone = -Math.round(Math.atan2(skystonePose.y-robotStartPose.y,skystonePose.x-robotStartPose.x));
            //Calculate the distance to the stone
            double distanceToStone = Math.round(Math.hypot(skystonePose.x-robotStartPose.x,skystonePose.y-robotStartPose.y));

            enc.gyroTurn(enc.TURN_SPEED, angleToStone);//Point to the stone
            mech.intakeStone();
            waitFor(1000);//Fold out intake
            mech.ejectStone();
            waitFor(1000);//Enable intake
            enc.steeringDrive(distanceToStone+4,false,false);//Get the stone
            enc.steeringDrive(-5,false,false);//Back up to stay clear of bridge
            enc.gyroTurn(enc.TURN_SPEED,-90);//Turn towards bridge
            enc.steeringDrive(35,false,false);//Go past bridge
            mech.intakeStone();
            waitFor(1000);//Eject the stone
            enc.steeringDrive(-2,false,false);//Park






        //--------------------------=+(Unknown)+=--------------------------\\







        } else if (pose.equals("unknown")) {//Default to left and hope the skystone is left
            //TODO: Change this to blind grab 2 stones instead of hoping for 1 skystone.

            enc.steeringDrive(4,false,false);//Get Clear of the Wall
            robotStartPose.x += 4;//Add the change to the robot pose

            //Calculate the angle to turn to face the stone... this is for speed
            Point skystonePose = new Point(2*tileWidth,tileWidth + 2*stoneLength + stoneLength/2);
            double angleToStone = -Math.round(Math.atan2(skystonePose.y-robotStartPose.y,skystonePose.x-robotStartPose.x));
            //Calculate the distance to the stone
            double distanceToStone = Math.round(Math.hypot(skystonePose.x-robotStartPose.x,skystonePose.y-robotStartPose.y));

            enc.gyroTurn(enc.TURN_SPEED, angleToStone);//Point to the stone
            mech.intakeStone();
            waitFor(1000);//Fold out intake
            mech.ejectStone();
            waitFor(1000);//Enable intake
            enc.steeringDrive(distanceToStone+4,false,false);//Get the stone
            enc.steeringDrive(-5,false,false);//Back up to stay clear of bridge
            enc.gyroTurn(enc.TURN_SPEED,-90);//Turn towards bridge
            enc.steeringDrive(35,false,false);//Go past bridge
            mech.intakeStone();
            waitFor(1000);//Eject the stone
            enc.steeringDrive(-2,false,false);//Park
        }
        AutoTransitioner.transitionOnStop(this,"MkI TeleOp");//Transition
    }
}
