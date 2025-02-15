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

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Tank.OpenCV.AutoCVAlgorithm;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;


//@Autonomous(name = "Crater Auto (ALL)")
public class CraterAutoNOW extends LinearOpMode {

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
        markerServo = hardwareMap.get(CRServo.class,"collector");

        //Set reverse on left motor to get motors to have same polarity
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Get OpenCV initialized
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        waitForStart();



        while (opModeIsActive()) {


            //OpenCV detection of mineral
            c1.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            c1.setShowCountours(true);
            c1.enable();


            /**
             *   LATCHING CODE GOES HERE
             *
             *
             */


            Thread.sleep(1500);

            List<MatOfPoint> contours = c1.getContours();
            for (int i = 0; i < contours.size(); i++) {
                Rect boundingRect = Imgproc.boundingRect(contours.get(i));
                xCor = ((boundingRect.x + boundingRect.width) / 2);
                yCor = ((boundingRect.y + boundingRect.height) / 2);
            }

            telemetry.addLine("xCor: " + xCor);
            telemetry.update();

            c1.disable();

            Thread.sleep(500);


            if(xCor < 150 && xCor != 0){

                //Turn counter-clockwise (positive) to hit ball and move past
                imuTurnCounter(0.4,18);
                target = 52;

                Thread.sleep(500);

                //Move to hit ball
                encoderMove(target,0.4,false);
                Thread.sleep(500);

                target = 10;
                encoderMove(target,0.4,true);

                //Turn perpendicular to depot
                imuTurnClock(0.3,-60);
                target = 10;

                Thread.sleep(500);

                //Go forward to depot
                encoderMove(target,0.3,false);
                Thread.sleep(500);

                //run servo code
                markerServo.setPower(-1);
                Thread.sleep(1500);
                markerServo.setPower(0);

                //Move to crater
                target = 58;
                encoderMove(target,0.5,true);


            } else if(xCor >= 150){

                //Move straight ahead to hit cube and get to depot
                target = 50;
                encoderMove(target,0.45,false);
                Thread.sleep(500);

                //Servo deposit code
                markerServo.setPower(-1);
                Thread.sleep(1500);
                markerServo.setPower(1);
                Thread.sleep(1250);

                //Turn parallel to depot
                imuTurnCounter(0.35,36);
                target = 12;
                Thread.sleep(500);

                //Move slightly downward to avoid hitting ball
                encoderMove(target,0.3,false);
                Thread.sleep(500);

                //Turn to be perpendicular to depot
                imuTurnCounter(0.35,110);
                Thread.sleep(500);

                //Move to crater
                target = 55;
                encoderMove(target,0.5,false);

            } else {

                //Turn clockwise (negative) to left
                imuTurnClock(0.4, -15);
                target = 52;

                Thread.sleep(500);

                //Start moving to hit cube given set target distance to cover
                encoderMove(target, 0.4, false);
                Thread.sleep(500);

                //Turn to be perpendicular to depot
                imuTurnCounter(0.35, 30);
                Thread.sleep(500);

                //Move towards depot to deposit marker
                target = 16;
                encoderMove(target,0.4, false);
                Thread.sleep(500);

                markerServo.setPower(-1);
                Thread.sleep(1500);
                markerServo.setPower(0);

                //Move towards crater
                target = 50;
                encoderMove(target, 0.4, true);

            }


            Thread.sleep(10000);

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
            while (motorR.getCurrentPosition() >= motorR.getTargetPosition() && motorL.getCurrentPosition() >= motorL.getTargetPosition());
        }else {
            do {
                motorR.setPower(speed);
                motorL.setPower(speed);
            }
            while (motorR.getCurrentPosition() <= motorR.getTargetPosition() && motorL.getCurrentPosition() <= motorL.getTargetPosition());
        }


        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorR.setPower(0);
        motorL.setPower(0);
    }


    //Turning Methods
    private void imuTurnCounter (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, 0.1)) {
            // Update telemetry & Allow time for other processes to run.
            idle();
        }

    }

    private void imuTurnClock (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeadingNegative(speed, angle, 0.1)) {
            // Update telemetry & Allow time for other processes to run.
            idle();
        }

    }

    private boolean onHeading(double speed, double angle, double PCoeff) {

        boolean onTarget = false ;

        returnAngles();

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorL.setPower(-speed);
        motorR.setPower(speed);

        if(heading >= angle){
            onTarget = true;
            motorL.setPower(0);
            motorR.setPower(0);
        }

        return onTarget;
    }

    private boolean onHeadingNegative(double speed, double angle, double PCoeff) {


        boolean onTarget = false ;


        returnAngles();

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private static String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private static void returnAngles(){
        //starting IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return necessary angle
        heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }


    //OpenCV Methods
    private void show(){
        telemetry.addData("xCor",xCor);
        telemetry.addData("yCor",yCor);
        telemetry.update();
    }


    //Maintenance Methods
    private void altBranch() {}











}

