package org.firstinspires.ftc.teamcode.Tank.Modded;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Tank.hardwareInit;

/** Sample test of using separate class for Auto commands
 *  @author Nikhil Narvekar
 */
//@Autonomous(name="Command Auto",group = "Modded")

public class AutoCommandBased extends LinearOpMode {


    hardwareInit robo = new hardwareInit();
    double coeff = 1.0;

    public void runOpMode() throws InterruptedException{

        robo.initializeAuto(hardwareMap);
        robo.initGyro();


        while(opModeIsActive()){

            robo.spinner.setPower(1);
            robo.delay(5);
            robo.spinner.setPower(0);
            robo.tele("Done with first step",1,1);
            robo.delay(2);

            robo.encoderMoveForward(10,0.4);
            robo.encoderMoveBack(10,0.4);
            robo.tele("Done with second step",1,1);
            robo.delay(2);

            robo.imuTurnClock(0.4,-90,coeff);
            robo.tele("Ang:",robo.returnAngles(),0);
            robo.delay(2);

            robo.imuTurnCounter(0.4,90,coeff);
            robo.tele("Ang:",robo.returnAngles(),0);
            robo.delay(2);

            robo.imuTurnClock(0.4,0,coeff);
            robo.tele("Ang:",robo.returnAngles(),0);

            robo.delay(30);

        }


    }




}
