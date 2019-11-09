package org.firstinspires.ftc.teamcode.SeansPlayground.Subsystems;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * @Author Sean Cardosi
 * @Date 11/8/19
 */
public class RobotMedia {

    MediaPlayer horn;
    MediaPlayer myaah;
    private boolean startState = false;
    private boolean backState = false;

    public RobotMedia(HardwareMap hardwareMap) {
        horn = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.horn);
        myaah = MediaPlayer.create(hardwareMap.appContext,com.qualcomm.ftcrobotcontroller.R.raw.myaah);
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

        backState = gamepad.a;

        if(myaah.isPlaying()&&!backState){
            myaah.stop();
        }
        //----------------------------------------------=+(Myaah)+=----------------------------------------------\\
    }
}
