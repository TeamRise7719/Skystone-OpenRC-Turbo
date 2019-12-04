package org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Memes;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @Author Sean Cardosi
 * @Date 11/8/19
 */
public class RobotMedia {

    ElapsedTime endgameTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    MediaPlayer horn;
    MediaPlayer myaah;
    MediaPlayer endgame;
    MediaPlayer spinMe;
    private boolean startState = false;
    private boolean backState = false;
    private boolean endgamePlayed = false;
    private boolean lBumperState = false;

    public RobotMedia(HardwareMap hardwareMap) {
        horn = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.horn);
        myaah = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.myaah);
        endgame = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.endgame);
        spinMe = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.rightround);
    }
    public void startTimer() {
        endgameTimer.reset();
    }

    public void playSounds(Gamepad gamepad, HardwareMap hardwareMap) {
        //----------------------------------------------=+(Horn)+=----------------------------------------------\\
        if ((gamepad.start)&&(!startState)) {
            horn.reset();
            horn = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.horn);
            horn.start();
        }
        startState = gamepad.start;

        if(horn.isPlaying()&&!startState){
            horn.stop();
        }
        //----------------------------------------------=+(Horn)+=----------------------------------------------\\


        //----------------------------------------------=+(Myaah)+=----------------------------------------------\\
        if ((gamepad.back)&&(!backState)) {
            myaah.reset();
            myaah = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.myaah);
            myaah.start();
        }

        backState = gamepad.back;

        if(myaah.isPlaying()&&!backState){
            myaah.stop();
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
        if ((gamepad.left_bumper)&&(!lBumperState)) {
            spinMe.reset();
            spinMe = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.rightround);
            spinMe.start();
            lBumperState = true;
        }

        lBumperState = gamepad.left_bumper;

        if (spinMe.isPlaying() && lBumperState) {
            spinMe.start();
        }
        //----------------------------------------------=+(Right Round)+=----------------------------------------------\\
    }
    //----------------------------------------------=+(Right Round Only)+=----------------------------------------------\\
    public void rightRound(boolean play) {//This might keep getting reset. Needs Testing.

        if (play) {
            spinMe.reset();
//            spinMe = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.rightround);
            spinMe.start();
        } else if (!play) {
            spinMe.stop();
        }
    }
    //----------------------------------------------=+(Right Round Only)+=----------------------------------------------\\

}
