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
    HardwareBot robot = new HardwareBot();

    private final double DRIVE_MOTOR_MULTIPLIER = 0.90; // sets the overall motor speed for the drive motors
    private final double SLOW_TURN              = 0.35; // sets the slow turn speed used through gamepad 1 dpad


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry to signify robot initialized and ready;
        telemetry.addData("Robot", "Ready to start");
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
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);
        }
        else {
            // Make the robot turn slowly if dpad buttons are pressed on gamepad 1
            if(gamepad1.dpad_left) {
                robot.rightDrive.setPower(SLOW_TURN);
                robot.leftDrive.setPower(-SLOW_TURN);
            }
            else if(gamepad1.dpad_right) {
                robot.rightDrive.setPower(-SLOW_TURN);
                robot.leftDrive.setPower(SLOW_TURN);
            }
        }

        // Controlling the linear lift using buttons
        String liftMotorStatus;
        if (gamepad1.dpad_up) {
            robot.liftMotor.setPower(1);
            liftMotorStatus = "going up";
        }
        else if (gamepad1.dpad_down) {
            robot.liftMotor.setPower(-1);
            liftMotorStatus = "going down";
        }
        else {
            robot.liftMotor.setPower(0);
            liftMotorStatus = "still";
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + robot.period.toString());
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
