package org.firstinspires.ftc.teamcode.Tank.Modded;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by nikhi on 2/3/2019.
 */

public class AdvancedTest extends LinearOpMode {

    DigitalChannel d1 = null;

    public void runOpMode() throws InterruptedException{

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        d1 = hardwareMap.get(DigitalChannel.class,"d1");

        while(opModeIsActive()){

            relativeLayout.setBackgroundColor(Color.BLACK);
            Thread.sleep(2000);
            relativeLayout.setBackgroundColor(Color.CYAN);
            Thread.sleep(2000);

            if(d1.getState()){
                relativeLayout.setBackgroundColor(Color.RED);
                while(d1.getState()){
                    idle();
                }
            }

        }
    }

}
