package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for our robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case, with words separated by underscores.
 *
 * Motor channel:  Left  drive motor:             "leftDrive"
 * Motor channel:  Right drive motor:             "rightDrive"
 * Motor channel:  Linear lift motor:             "liftMotor"
 * Servo channel:  Servo to drop the team market: "markerServo"
 * Servo channel:  Servo the push the gold cube:  "samplingServo"
 * Servo channel:  Servo to insert the minerals:  "mineralServo"
 */
public class HardwareBot
{
    /* Public OpMode members. */
    // DC motors
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor liftMotor = null;
    // Servos
    public Servo markerServo = null;
    public Servo samplingServo = null;
    public Servo mineralServo = null;
    // Color sensor
    public ColorSensor colourSensor;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    public float hsvValues[] = {0F,0F,0F};
    // values is a reference to the hsvValues array.
    public final float values[] = hsvValues;
    // IMU
    public BNO055IMU imu;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // Constants
    public static final double MARKER_SERVO_HOLD = 0;
    public static final double MARKER_SERVO_DROP = 1;
    public static final double SAMPLING_SERVO_PUSH = 0;
    public static final double SAMPLING_SERVO_TUCK = 1;
    public static final double MINERAL_SERVO_COLLECT = 0;
    public static final double MINERAL_SERVO_INSERT = 1;

    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    public ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public HardwareBot(){

    }
    /**
     * Initialize standard Hardware interfaces for teleOP, using method overloading
     */

    public void init(HardwareMap ahardwareMap) {
        // Save reference to Hardware map
        hardwareMap = ahardwareMap;

        // Define and Initialize DC Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "MotorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorRight");
        liftMotor = hardwareMap.get(DcMotor.class, "MotorLift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        liftMotor.setPower(0);

        // Set all motors to run without encoders increase performance in TeleOp situations
        // Other encoder related tasks will set the motor RunMode if necessary
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos
        markerServo = hardwareMap.get(Servo.class, "markerServo");
        samplingServo = hardwareMap.get(Servo.class, "samplingServo");
        mineralServo = hardwareMap.get(Servo.class, "mineralServo");
        markerServo.setPosition(MARKER_SERVO_HOLD);
        samplingServo.setPosition(SAMPLING_SERVO_TUCK);
        mineralServo.setPosition(MINERAL_SERVO_COLLECT);
    }

        /**
         * Initialize standard Hardware interfaces based on parameter input
         * if the relevant parameter is 1, that device will be initialised
         */
    public void init(HardwareMap ahardwareMap, boolean useIMU, boolean useColour) {
        // Save reference to Hardware map
        hardwareMap = ahardwareMap;

        // Define and Initialize DC Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "MotorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorRight");
        liftMotor = hardwareMap.get(DcMotor.class, "MotorLift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        liftMotor.setPower(0);

        // Set all motors to run without encoders increase performance in TeleOp situations
        // Other encoder related tasks will set the motor RunMode if necessary
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos
        markerServo = hardwareMap.get(Servo.class, "markerServo");
        samplingServo = hardwareMap.get(Servo.class, "samplingServo");
        mineralServo = hardwareMap.get(Servo.class, "mineralServo");
        markerServo.setPosition(MARKER_SERVO_HOLD);
        samplingServo.setPosition(SAMPLING_SERVO_TUCK);
        mineralServo.setPosition(MINERAL_SERVO_COLLECT);

        if(useIMU) {
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
        if(useColour) {
            colourSensor = hardwareMap.colorSensor.get("sensor_color");
        }
    }
 }

