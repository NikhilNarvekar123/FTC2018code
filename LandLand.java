/**
 *  Working branch paths (probably) + value tuning
 */

package org.firstinspires.ftc.teamcode.Tank.Auto;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Tank.OpenCV.AutoCVAlgorithm;
import org.firstinspires.ftc.teamcode.Tank.vals;

import java.util.Locale;


@Autonomous(name = "Sample Land")

public class LandLand extends LinearOpMode implements vals {

    //All class components (soon to extend lowering motor/marker servo)
    public DcMotor motorL;
    public DcMotor motorR;
    public static BNO055IMU imu;
    public CRServo markerServo;
    public DcMotor latchM;

    //Encoder values, might need to account for gear train reduction
    private static double inchesPerRot = (228) / (5.5 * Math.PI);
    private static double target;
    private static int tarPosL;
    private static int tarPosR;
    int dick = 69;

    //IMU setup and values
    static Orientation angles;
    static double heading = 0.0;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    //CV components
    private AutoCVAlgorithm c1 = new AutoCVAlgorithm();
    private static int xCor;
    private static int yCor;





    public void runOpMode() throws InterruptedException {


        //Hardware Maps
        motorL = hardwareMap.dcMotor.get("MotorLeft");
        motorR = hardwareMap.dcMotor.get("MotorRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        latchM = hardwareMap.get(DcMotor.class, "pivotMotor");
        //markerServo = hardwareMap.get(CRServo.class,"crServo");

        //Set reverse on left motor to get motors to have same polarity
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Get OpenCV initialized
        imuSetUp();
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        waitForStart();



        while (opModeIsActive()) {

            latchM.setPower(0.8);
            Thread.sleep(3000);
            latchM.setPower(0);

            imuTurnCounter(0.4,15);
            encoderMove(2,0.3,false);
            telemetry.addLine("Done moving");
            telemetry.update();
            imuTurnClock(0.4,0);





            Thread.sleep(500);
            telemetry.addLine("test");
            telemetry.update();
            Thread.sleep(30000);




        }




    }





    //Movement
    private void encoderMove(double inches,double speed,boolean movingBackwards){

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        if(!movingBackwards){
            tarPosR = motorR.getCurrentPosition() + (int)(inches * inchesPerRot);
            tarPosL = motorL.getCurrentPosition() + (int)(inches * inchesPerRot);
        } else {
            tarPosR = motorR.getCurrentPosition() - (int)(inches * inchesPerRot);
            tarPosL = motorL.getCurrentPosition() - (int)(inches * inchesPerRot);
        }


        motorR.setTargetPosition(-tarPosR);
        motorL.setTargetPosition(-tarPosL);


        motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(!movingBackwards) {
            do {
                motorR.setPower(speed);
                motorL.setPower(speed);
            }
            while (motorR.isBusy() && motorL.isBusy());
        }else {
            do {
                motorR.setPower(speed);
                motorL.setPower(speed);
            }
            while (motorR.isBusy() && motorL.isBusy());
        }


        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorR.setPower(0);
        motorL.setPower(0);
    }


    //Turning Methods
    private void imuTurnCounter (double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle)) {
            telemetry.addData("A:",heading);
            telemetry.update();
        }
    }

    private void imuTurnClock (double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeadingNegative(speed, angle)) {
            telemetry.addData("A:",heading);
            telemetry.update();
        }
    }

    private boolean onHeading(double speed, double angle) {

        boolean onTarget = false ;

        returnAngles();

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorL.setPower(-speed);
        motorR.setPower(speed);

        if(heading >= angle){
            onTarget = true;
            motorL.setPower(0);
            motorR.setPower(0);
        }

        return onTarget;
    }

    private boolean onHeadingNegative(double speed, double angle) {

        boolean onTarget = false ;

        returnAngles();

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorL.setPower(speed);
        motorR.setPower(-speed);

        if(heading <= angle){
            onTarget = true;
            motorR.setPower(0);
            motorL.setPower(0);
        }

        return onTarget;
    }

    private static String formatAngle(AngleUnit angleUnit, double angle) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    private static void returnAngles(){
        //starting IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return necessary angle
        heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    private void imuSetUp() {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }

    //Maintenance Methods
    private void altBranch() {

    }











}

