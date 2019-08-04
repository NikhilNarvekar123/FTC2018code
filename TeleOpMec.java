package org.firstinspires.ftc.teamcode.Mecanum;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TeleOpMec extends LinearOpMode {

    hardwareMec robo = new hardwareMec();

    public void runOpMode() throws InterruptedException{

        robo.initializeTele(hardwareMap);

        while(opModeIsActive()){

            //Mecanum drive control on one joystick
            robo.LMoto1.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            robo.LMoto2.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
            robo.RMoto1.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
            robo.RMoto2.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);

        }



    }


}
