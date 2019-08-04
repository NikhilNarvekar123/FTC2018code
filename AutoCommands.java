package org.firstinspires.ftc.teamcode.Tank.TestFiles;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by nikhi on 1/17/2019.
 */

public class AutoCommands {


    //All class components (soon to extend lowering motor/marker servo)
    public static DcMotor motorL;
    public static DcMotor motorR;
    public static BNO055IMU imu;
    public static CRServo markerServo;
    public static DcMotor latchM;

    //Encoder values, might need to account for gear train reduction
    private static double inchesPerRot = (228) / (5.5 * Math.PI);
    public static double target;
    private static int tarPosL;
    private static int tarPosR;

    //IMU setup and values
    static Orientation angles;
    static double heading = 0.0;
    static BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    //CV components
    public static int xCor;
    public static int yCor;



    public static void mapper(HardwareMap hw){

        //Hardware Maps
        motorL = hw.dcMotor.get("MotorLeft");
        motorR = hw.dcMotor.get("MotorRight");
        imu = hw.get(BNO055IMU.class, "imu");
        latchM = hw.get(DcMotor.class, "pivotMotor");
        markerServo = hw.get(CRServo.class,"crServo");

        //Set reverse on left motor to get motors to have same polarity
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    public static void init() throws InterruptedException {

    }


}
