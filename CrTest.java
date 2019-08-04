/**package org.firstinspires.ftc.teamcode.Tank.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="CRTest")

public class CrTest extends LinearOpMode {


    CRServo intakeWheel;

    public void runOpMode() throws InterruptedException{

        intakeWheel = hardwareMap.crservo.get("crServo");


        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.a) {
                intakeWheel.setPower(1);
            } else {
                intakeWheel.setPower(-1);
            }
        }

    }

}
 **/
