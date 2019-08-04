package org.firstinspires.ftc.teamcode.Tank.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp (Fresh Rice)")

public class MainTeleOp extends LinearOpMode {

    // Declare OpMode members.

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private CRServo intakeWheel = null;
    private DcMotor pivotMotor = null;

    private int count = 1;

    @Override
    public void runOpMode() {


        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        intakeWheel = hardwareMap.get(CRServo.class,"crServo");
        leftDrive = hardwareMap.get(DcMotor.class,"MotorLeft");
        rightDrive = hardwareMap.get(DcMotor.class,"MotorRight");

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double verRotate = gamepad2.right_stick_y * 0.8;

            //Pinion arm
            pivotMotor.setPower(verRotate);

            // Tank Control
            if(gamepad1.a){
             count++;
             }


             if(count % 2 == 0){
                leftDrive.setPower(gamepad1.left_stick_y * 0.3);
                rightDrive.setPower(gamepad1.right_stick_y * 0.3);
             }else {
                leftDrive.setPower(gamepad1.left_stick_y * 0.6);
                rightDrive.setPower(gamepad1.right_stick_y * 0.6);
             }

             intakeWheel.setPower(-gamepad2.left_stick_y);

        }

    }

}