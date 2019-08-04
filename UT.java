package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "DHDHDD")
public class UT extends LinearOpMode {


    private DcMotor m1;
    private DcMotor m2;
    private DcMotor swing;
    private DcMotor lift;
    private Servo s1;
    private Servo s2;


    public void runOpMode(){

        m2 = hardwareMap.dcMotor.get("m2");
        swing = hardwareMap.dcMotor.get("swing");
        lift = hardwareMap.dcMotor.get("lift");
        s1 = hardwareMap.servo.get("s1");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        while(opModeIsActive()){


            m2.setPower(gamepad1.right_stick_y);

            if(gamepad1.a)
                swing.setPower(1);
            else if(gamepad1.b)
                swing.setPower(-1);
            else
                swing.setPower(0);


            if(gamepad1.left_trigger > 0)
                lift.setPower(1);
            else if(gamepad1.right_trigger > 0)
                lift.setPower(-1);
            else
                lift.setPower(0);



            if(gamepad1.dpad_up)
                s1.setPosition(1);
            else if(gamepad1.dpad_down)
                s1.setPosition(-1);


        }



    }


}
