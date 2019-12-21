package org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Memes;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sean Cardosi on 2019-11-8
 */
public class RobotMedia {

    private ElapsedTime endgameTimer;
    private MediaPlayer horn;
    private MediaPlayer swamp;
    private MediaPlayer endgame;
    private MediaPlayer spinMe;
    private boolean startState = false;
    private boolean backState = false;
    private boolean endgamePlayed = false;
    private boolean lBumperState = false;

    public RobotMedia(HardwareMap hardwareMap) {
        horn = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.horn);
        swamp = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.myaah);
        endgame = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.endgame);
        spinMe = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.rightround);
        endgameTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public void startTimer() {
        endgameTimer.reset();
    }

    public void playSounds(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap) {

        //----------------------------------------------=+(Horn)+=----------------------------------------------\\
        if ((gamepad1.start)&&(!startState)) {
            horn.reset();
            horn = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.horn);
            horn.start();
        }
        startState = gamepad1.start;

        if(horn.isPlaying()&&!startState){
            horn.stop();
        }
        //----------------------------------------------=+(Horn)+=----------------------------------------------\\


        //----------------------------------------------=+(Myaah)+=----------------------------------------------\\
        if ((gamepad2.start)&&(!backState)) {
            swamp.reset();
            swamp = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.myaah);
            swamp.start();
        }

        backState = gamepad2.start;

        if(swamp.isPlaying()&&!backState){
            swamp.stop();
        }
        //----------------------------------------------=+(Myaah)+=----------------------------------------------\\


        //----------------------------------------------=+(Endgame)+=----------------------------------------------\\
        if (endgameTimer.seconds() >= 90 && !endgamePlayed) {
            endgame.reset();
            endgame = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.endgame);
            endgame.start();//We are in the endgame now
            endgamePlayed = true;
        }
        if (endgame.isPlaying() && endgame.isLooping()) {
            endgame.stop();
        }
        //----------------------------------------------=+(Endgame)+=----------------------------------------------\\


        //----------------------------------------------=+(Right Round)+=----------------------------------------------\\
        if ((gamepad1.left_bumper)&&(!lBumperState)) {
            spinMe.reset();
            spinMe = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.rightround);
            spinMe.start();
            lBumperState = true;
        }

        lBumperState = gamepad1.left_bumper;

        if (spinMe.isPlaying() && lBumperState) {
            spinMe.start();
        }
        //----------------------------------------------=+(Right Round)+=----------------------------------------------\\
    }
    //----------------------------------------------=+(Right Round Only)+=----------------------------------------------\\
    public void rightRound(boolean play) {//This might keep getting reset. Needs Testing.

        if (play) {
//            spinMe.reset();
//            spinMe = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.rightround);
            spinMe.start();
        } else {
            spinMe.stop();
        }
    }
    //----------------------------------------------=+(Right Round Only)+=----------------------------------------------\\

}
