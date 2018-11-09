package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Tele Op - Single Player", group="SyntaxError")
public class TeleOp_Single extends OpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftMotor = null;
    private Servo servo0 = null;
    private Servo servo5 = null;

    private final double DRIVE_MOTOR_MULTIPLIER = 0.90; // sets the overall motor speed for the drive motors
    private final double SLOW_TURN              = 0.35; // sets the slow turn speed used through gamepad 1 dpad
    private final double servo5_down = 0.1;
    private final double servo5_up = 1;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "MotorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorRight");
        liftMotor = hardwareMap.get(DcMotor.class, "MotorLift");
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo5 = hardwareMap.get(Servo.class, "servo5");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Servo 5 stuff
        servo5.setPosition(servo5_down);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
////////////////////


        // Wait for the start button
        telemetry.addData(">", "Press Start" );
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double throttle;
        double steering;
        double left;
        double right;

        // Get values from triggers for throttle control
        throttle = - gamepad1.right_trigger + gamepad1.left_trigger;

        // Get steering values from the left stick x axis
        steering = -gamepad1.left_stick_x;
        // Smooth the range with an exponential equation
        throttle = smoothCurve(throttle);
        steering = smoothCurve(steering);

        // Actual steering logic
        right = Range.clip((throttle+steering), -1, 1);
        left = Range.clip((throttle-steering), -1, 1);
        right = right * DRIVE_MOTOR_MULTIPLIER;
        left = left * DRIVE_MOTOR_MULTIPLIER;

        // If gamepad 1 A button is pressed halve the motor speeds for ulta high precision
        if(gamepad1.a) {
            right = right / 2;
            left  = left  / 2;
        }
        if(!gamepad1.dpad_right && !gamepad1.dpad_left) {
            leftDrive.setPower(left);
            rightDrive.setPower(right);
        }
        else {
            // Make the robot turn slowly if dpad buttons are pressed on gamepad 1
            if(gamepad1.dpad_left) {
                rightDrive.setPower(SLOW_TURN);
                leftDrive.setPower(-SLOW_TURN);
            }
            else if(gamepad1.dpad_right) {
                rightDrive.setPower(-SLOW_TURN);
                leftDrive.setPower(SLOW_TURN);
            }
        }

        // Controlling the linear lift using buttons
        String liftMotorStatus;
        if (gamepad1.dpad_up) {
            liftMotor.setPower(1);
            liftMotorStatus = "going up";
        }
        else if (gamepad1.dpad_down) {
            liftMotor.setPower(-1);
            liftMotorStatus = "going down";
        }
        else {
            liftMotor.setPower(0);
            liftMotorStatus = "still";
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", left, right);
        telemetry.addData("Lift", liftMotorStatus);

        telemetry.addData("Status", "Running");
        telemetry.update();


    }

    /*f
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    // Additional method
    private double smoothCurve(double power) {
        return Math.pow(power,3);
    }
}
