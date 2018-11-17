package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareBot;


@TeleOp(name="Tele Op - Two Player", group="SyntaxError")
public class TeleOp_Dual extends OpMode{

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
        double liftPower;
        double left;
        double right;

        // Get values from triggers for throttle control
        throttle = - gamepad1.right_trigger + gamepad1.left_trigger;

        // Get steering values from the left stick x axis
        steering = - gamepad1.left_stick_x;
        // Smooth the range with an exponential equation
        throttle = smoothCurve(throttle);
        steering = smoothCurve(steering);

        // Actual steering logic
        right = Range.clip((throttle+steering), -1, 1);
        left = Range.clip((throttle-steering), -1, 1);
        right = right * DRIVE_MOTOR_MULTIPLIER;
        left = left * DRIVE_MOTOR_MULTIPLIER;

        String driveMotorsStatus;

        // If the dpad turn buttons are nor pressed drive the robot at normal speed
        if(!gamepad1.dpad_right && !gamepad1.dpad_left) {
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);

            /*
            Brake both motors if the power sent to both motors is zero
            so that the robot doesn't coast
             */
            if (left == 0 && right == 0) {
                robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Telemetry stuff - does not affect the functionality of the robot
            if (left == right) {
                if (left == 0) {
                    driveMotorsStatus = "staying still";
                }
                else if (left > 0) {
                    driveMotorsStatus = "going forward straight";
                }
                else {
                    driveMotorsStatus = "going backward straight";
                }

            }
            else if (left > right) {
                driveMotorsStatus = "turning right";
            }
            else {
                driveMotorsStatus = "turning left";
            }
        }
        else {
            // Make the robot turn slowly if dpad left or right buttons are pressed on gamepad 1
            if(gamepad1.dpad_left) {
                robot.rightDrive.setPower(SLOW_TURN);
                robot.leftDrive.setPower(-SLOW_TURN);
                driveMotorsStatus = "turning left slowly";
            }
            else { // Basically - else if (game1.dpad.right)
                robot.rightDrive.setPower(-SLOW_TURN);
                robot.leftDrive.setPower(SLOW_TURN);
                driveMotorsStatus = "turning right slowly";
            }
        }

        // Controlling the linear lift using buttons
        String liftMotorStatus;
        if (gamepad1.dpad_up) {
            liftPower = 1;
            liftMotorStatus = "going up";
        }
        else if (gamepad1.dpad_down) {
            liftPower = -1;
            liftMotorStatus = "going down";
        }
        else {
            liftPower = 0;
            robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotorStatus = "still";
        }

        // Set the lift motor's power
        robot.liftMotor.setPower(liftPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + robot.period.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f), lift (%.2f)", left, right, liftPower);
        telemetry.addData("Drive train", driveMotorsStatus);
        telemetry.addData("Linear lift", liftMotorStatus);

        telemetry.addData("Status", "Running");
        telemetry.update();


    }

    /*f
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Brake all motors to prevent any motion
        // Brake the linear lift motor
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Brake
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Additional method
    private double smoothCurve(double power) {
        return Math.pow(power,3);
    }
}
