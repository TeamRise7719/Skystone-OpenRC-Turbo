package org.firstinspires.ftc.teamcode.SeansSpace.Subsystems.Memes;

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
    private boolean startState = false;
    private boolean backState = false;
    private boolean endgamePlayed = false;

    public RobotMedia(HardwareMap hardwareMap) {
        horn = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.horn);
        myaah = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.myaah);
        endgame = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.endgame);
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
        if (endgameTimer.seconds() >= 90 && endgamePlayed == false) {
            endgame.start();//We are in the endgame now
            endgamePlayed = true;
        }
        //----------------------------------------------=+(Endgame)+=----------------------------------------------\\
    }
}
