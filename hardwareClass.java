/** The class manages the hardware maps in all of the opmodes.
 *  @status THEORETICAL
 *  @author Free of Charge Programming Team
 *  @dateCreated 11/18/2018
 *  @dateModified 11/18/2018
 */

package org.firstinspires.ftc.teamcode.Tank.TestFiles;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class hardwareClass {

    //Main components
    public DcMotor motorL = null;
    public DcMotor motorR = null;
    public DcMotor pivotMotor = null;
    public DcMotor latchM = null;


    public CRServo markerServo = null;
    public BNO055IMU imu = null;


    //Hardware map components
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    //Constructor (buffer)
    public hardwareClass(){}

    //Standard interface
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        //Hardware Maps
        motorL = hwMap.dcMotor.get("MotorLeft");
        motorR = hwMap.dcMotor.get("MotorRight");
        imu = hwMap.get(BNO055IMU.class, "imu");
        latchM = hwMap.dcMotor.get("pivotMotor");
        markerServo = hwMap.crservo.get("crServo");

        //Set reverse on left motor to get motors to have same polarity
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Define and initialize ALL installed servos.
        markerServo = hwMap.get(CRServo.class, "crServo");

    }


}
