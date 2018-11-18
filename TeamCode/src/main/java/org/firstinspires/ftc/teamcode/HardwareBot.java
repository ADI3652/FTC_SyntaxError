package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

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
 * Sensor channel: Inertial measurement unit:     "imu" - 12C port 0
 * Sensor channel: Colour sensor for sampling:    "colourSensor" - 12C port 1
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
    // Distance sensor
    public DistanceSensor distanceSensor;
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

    public static final double COUNTS_PER_MOTOR_REV    = 1440;
    public static final double DRIVE_GEAR_REDUCTION    = 1.4;
    public static final double WHEEL_DIAMETER_CENTIMETERS   = 5;
    public static final double COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CENTIMETERS * 3.1415);

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
            colourSensor = hardwareMap.colorSensor.get("colourSensor");
        }

    }

    // Custom methods
    // This method allows the robot to drive forward or backward for a given distance at a given speed
    public void encoderDrive(double speed, double distanceCM, int timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        /*
        If the robot needs to move forwards the distance input will be a positive number,
        therefore the new position of the motors will be greater than the current position.
        Alternatively, if the robot needs to move backwards the distance input will be a negative
        number, therefore the new position of the motors will be lower than the current position.
         */
        newLeftTarget = leftDrive.getCurrentPosition() - (int) (distanceCM * COUNTS_PER_CM);
        newRightTarget = rightDrive.getCurrentPosition() - (int) (distanceCM * COUNTS_PER_CM);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        The absolute value of speed is set to the motors because
        the motors will automatically rotate backwards if the target position is less than
        the current position
         */
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        double startTime = System.currentTimeMillis() / 1000; // Dividing by 1000 converts it to seconds
        double currentTime = 0;
        while ((currentTime - startTime < timeoutS) && (leftDrive.isBusy() || rightDrive.isBusy())) {
            currentTime = SystemClock.currentThreadTimeMillis() / 1000; // Dividing by 1000 converts it to seconds
        }
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // This method allows the robot to drive to a given position
    public void encoderDriveToPosition(double speed, int leftMotorPosition, int rightMotorPosition, int timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        /*
        If the robot needs to move forwards the distance input will be a positive number,
        therefore the new position of the motors will be greater than the current position.
        Alternatively, if the robot needs to move backwards the distance input will be a negative
        number, therefore the new position of the motors will be lower than the current position.
         */
        // newLeftTarget = leftDrive.getCurrentPosition() + (rightMotorPosition - rightDrive.getCurrentPosition());
        newLeftTarget = leftMotorPosition;
        newRightTarget = rightMotorPosition;

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        The absolute value of speed is set to the motors because
        the motors will automatically rotate backwards if the target position is less than
        the current position
         */
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        double startTime = System.currentTimeMillis() / 1000; // Dividing by 1000 converts it to seconds
        double currentTime = 0;
        while ((currentTime - startTime < timeoutS) && (leftDrive.isBusy() || rightDrive.isBusy())) {
            currentTime = SystemClock.currentThreadTimeMillis() / 1000; // Dividing by 100 converts it to seconds

        }
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // This method allows the robot to turn to its left on the spot for a given amount of degrees
    public void imuTurnLeft(double speed, double rotationD, int timeoutS) {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetAngle = imu.getAngularOrientation().firstAngle + rotationD;

        double startTime = System.currentTimeMillis() / 1000; // Dividing by 1000 converts it to seconds
        double currentTime = 0;
        /*
        Keep on turning left on the spot until the difference between the
        desired angle and current angle is less than 2.5 degrees
         */
        // This means that the robot's turning has a precision of 1.5 degrees
        while ((currentTime - startTime < timeoutS) &&
                (Math.abs(targetAngle - imu.getAngularOrientation().firstAngle) > 1.5)) {
            leftDrive.setPower(-Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            currentTime = SystemClock.currentThreadTimeMillis() / 1000; // Dividing by 1000 converts it to seconds
        }
        // When the robot has turned to the desired orientation brake both motors to prevent coasting
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // This method allows the robot to turn to its right on the spot for a given amount of degrees
    public void imuTurnRight(double speed, double rotationD, int timeoutS) {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        The intended rotation value is subtracted from the initial angle because as the robot
        turns right the gyroscope reading decreases
         */
        double targetAngle = imu.getAngularOrientation().firstAngle - rotationD;

        double startTime = System.currentTimeMillis() / 1000; // Dividing by 1000 converts it to seconds
        double currentTime = 0;
        while ((currentTime - startTime < timeoutS) &&
                (Math.abs(targetAngle - imu.getAngularOrientation().firstAngle) > 1.5)) {
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(-Math.abs(speed));
            currentTime = SystemClock.currentThreadTimeMillis() / 1000; // Dividing by 1000 converts it to seconds
        }
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // This method return if a white colour (white mineral) has been detected
    public boolean isWhiteMineral() {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // Converts the raw RGB values to HSV values, which are easier to use to detect ball colours
        Color.RGBToHSV((colourSensor.red() * 255) / 800, (colourSensor.green() * 255) / 800, (colourSensor.blue() * 255) / 800, hsvValues);
        if (hsvValues[1] < 0.25 && hsvValues[2] > 0.9) {
            return true;
        }
        else {
            return false;
        }
    }

    // This method return if a gold colour (gold cube) has been detected
    public boolean isGoldMineral() {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // Converts the raw RGB values to HSV values, which are easier to use to detect ball colours
        Color.RGBToHSV((colourSensor.red() * 255) / 800, (colourSensor.green() * 255) / 800, (colourSensor.blue() * 255) / 800, hsvValues);
        if ((colourSensor.red() > colourSensor.blue()) && (colourSensor.green() > colourSensor.blue())) {
            return true;
        }
        else {
            return false;
        }
    }

 }

