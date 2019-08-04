package org.firstinspires.ftc.teamcode.Tank.Modded;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tank.hardwareInit;

/** Sample TeleOp based on separate hardware file
 *  @author Nikhil Narvekar
 */

//@TeleOp(name="Automated Tele",group="Modded")

public class AutomatedTele extends LinearOpMode {

    hardwareInit robo = new hardwareInit();

    public void runOpMode(){

        robo.initializeTele(hardwareMap);

        while(opModeIsActive()){

            robo.leftDrive.setPower(gamepad1.right_stick_y);
            robo.rightDrive.setPower(gamepad1.left_stick_y);


        }

    }

}
