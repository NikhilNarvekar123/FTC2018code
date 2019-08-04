package org.firstinspires.ftc.teamcode.Mecanum;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/** UNLISTED OPMODE, CONTAINS METHODS/SETUP
 *  @author NIKHIL NARVEKAR               */

public class hardwareMec extends LinearOpMode  {

    public void runOpMode(){}

    /* Hardware Vars */
    public DcMotor LMoto1 = null;
    public DcMotor LMoto2 = null;
    public DcMotor RMoto1 = null;
    public DcMotor RMoto2 = null;
    public static BNO055IMU imu = null;

    /* Drive Vars */
    int mode = 1;

    /* IMU Vars */
    private static Orientation angles;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    /* Encoder Vars */
    private static final double INCHES_PER_ROT = (288) / (5.5 * Math.PI);

    /* Sample Multi-Thread - CURRENTLY DISABLED */
    //private Secondary sc = new Secondary();


    /** TeleOp methods.
     *  TBD
     */

    /* Init Methods */
    public void initializeTele(HardwareMap hw) {

        LMoto1 = hw.get(DcMotor.class, "MotorLeft");
        LMoto2 = hw.get(DcMotor.class,"MotorLeft2");
        RMoto1 = hw.get(DcMotor.class,"MotorRight");
        RMoto2 = hw.get(DcMotor.class,"MotorRight2");

    }



    public void encoderMove(){

    }





}
