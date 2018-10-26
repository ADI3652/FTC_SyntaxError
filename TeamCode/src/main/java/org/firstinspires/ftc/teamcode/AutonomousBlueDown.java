package org.firstinspires.ftc.teamcode;
// Linear autonomous program imports

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

// Allows for conversion to HSV (hue, saturation, value) from RGB (red, green blue)
// Required imports for program specific methods
// Used with Vuforia to track the vuMarks

@Autonomous(name = "BlueLeftVuforia", group = "Blue")
public class AutoBlueLeftVuforiaHWBot extends LinearOpMode {
    HardwareBot  robot        = new HardwareBot();
    private ElapsedTime runtime = new ElapsedTime();

    double distanceOffset = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 1440;
    static final double     DRIVE_GEAR_REDUCTION    = 1.333333;
    static final double     WHEEL_DIAMETER_CENTIMETERS   = 10.16;
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CENTIMETERS * 3.1415);

    @Override
    public void runOpMode() {
        /* When initialise button is pressed: */
        // Initialise relevant devices with IMU, color, vuforia and the jewel arm enabled
        robot.init(hardwareMap,1,1,1,1);

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        
        /* Wait for start button to be pressed */
        waitForStart();
        /* When start button is pressed; run main code */
        // Close the claw to grasp the preloaded block, ready for lifting after knocking off the jewel
        robot.leftClaw.setPosition(robot.LEFT_CLAW_OPEN + 0.6);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_OPEN - 0.6);

        // Slowly move the jewel arm up to a middle position, so as to not unintentionally knock off any balls
        // Uses custom method (below) to add delay in between each movement
        slowServoMove(robot.jewelArm2,0.01,5,robot.JEWEL_ARM_UP_2, robot.JEWEL_ARM_MIDDLE_2);
        // Rotates the jewel arm to the central position
        slowServoMove(robot.jewelArm1,0.01,10,robot.JEWEL_ARM_UP_1,robot.JEWEL_ARM_DOWN_1);
        // Lowers the jewel arm, ready to read the ball's colour
        slowServoMove(robot.jewelArm2,0.01,40,robot.JEWEL_ARM_MIDDLE_2,robot.JEWEL_ARM_DOWN_2);

        // Loop the colour sensor readings to display to telementry and provide time for readings to settle
        for(int i =0; i<20; i++) {
            // Converts the raw RGB values to HSV values, which are easier to use to detect ball colours
            Color.RGBToHSV((robot.sensorRGB.red() * 255) / 800, (robot.sensorRGB.green() * 255) / 800, (robot.sensorRGB.blue() * 255) / 800, hsvValues);
            telemetry.addData("Clear", robot.sensorRGB.alpha());
            telemetry.addData("Red  ", robot.sensorRGB.red());
            telemetry.addData("Green", robot.sensorRGB.green());
            telemetry.addData("Blue ", robot.sensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
            if(!opModeIsActive()) break;
        }
        // If the detected ball is red, move the servo away from it to knock the other one off
        if(hsvValues[0] > 340 || hsvValues[0] < 25) {
            robot.jewelArm1.setPosition(robot.JEWEL_ARM_DOWN_1 - 0.2);
        }
        // If the detected ball is blue, move the servo towards it to knock it off
        else if (hsvValues[0] > 200 && hsvValues[0] < 250){
            robot.jewelArm1.setPosition(robot.JEWEL_ARM_DOWN_1 + 0.2);
        }
        // A delay to allow time for servos to move
        sleep(600);
        // Move servos to a safe position, where the other ball will not be knocked off
        robot.jewelArm2.setPosition(robot.JEWEL_ARM_MIDDLE_2);
        robot.jewelArm1.setPosition(robot.JEWEL_ARM_UP_1);
        sleep(300);
        // Stow the jewel arm away
        robot.jewelArm2.setPosition(robot.JEWEL_ARM_UP_2);
        sleep(1000);

        // Pick up the glyph a little so as not to drag it along the floor
        robot.arm.setPower(-0.5);
        sleep(250);
        robot.arm.setPower(0);

        // Track the pictographs (vuMarks)
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(robot.relicTemplate);
        // Check if one is found, if not, aim for centre column (default offset of 0)
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            // Centre vuMark does not need to change the distance offset from the default of 0
            // If left vuMark, add 17.2 cm to the distance towards the glyph box
            if(vuMark == RelicRecoveryVuMark.LEFT) {
                distanceOffset = -17.45;
            }
            // If right vuMark, subtract 17.45 cm from the distance towards the glyph box
            else if(vuMark == RelicRecoveryVuMark.RIGHT) {
                distanceOffset = 17.3;
            }
        }

        // Move the robot forward a little (negative is backwards when using encoders) to make it easier to turn
        encoderDrive(0.4,-3.2, -3.2,9);

        // Turn the robot about 90 degrees, angle is less due to overshoot (no feedback loop)
        while(robot.imu.getAngularOrientation().firstAngle < 85 && opModeIsActive()) {
            robot.rightDrive.setPower(-0.3);
            robot.leftDrive.setPower(0.3);
        }

        // Drive towards the glyph box
        encoderDrive(0.8,(-63.5), (-63.5),9);

        // Turn away from the glyph box, back to the original 0 degree heading
        while(robot.imu.getAngularOrientation().firstAngle > 5 && opModeIsActive()) {
            robot.rightDrive.setPower(0.3);
            robot.leftDrive.setPower(-0.3);
        }

        // Drive towards the glyph columns, taking into account the offsets
        encoderDrive(0.8,(-25 - distanceOffset), (-25 - distanceOffset),9);

        // Turn the robot about 90 degrees, angle is less due to overshoot (no feedback loop)
        while(robot.imu.getAngularOrientation().firstAngle < 85 && opModeIsActive()) {
            robot.rightDrive.setPower(-0.3);
            robot.leftDrive.setPower(0.3);
        }

        // Drive towards the cryptobox
        encoderDrive(0.8,-36,-36,5);

        // Open the claws, dropping the glyph hopefully into (a or the correct) column
        robot.leftClaw.setPosition(robot.LEFT_CLAW_OPEN);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_OPEN);
        encoderDrive(0.8,3,3,5);
        sleep(500);

        // Drive robot backwards so as to not be touching the glyph, but remaining in the safe zone
        encoderDrive(0.8,7,7,5);
    }

    // An encoder drive method, modified from a sample method by FIRST to use CM instead of inches
    private void encoderDrive(double speed,
                             double leftCM, double rightCM,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftCM * COUNTS_PER_CM);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightCM * COUNTS_PER_CM);

            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Completely custom method to slowly move servos
    public void slowServoMove(Servo servo, double stepSize, int delayStep, double currentPosition, double targetPosition) {
        // Continuously stores the position the servo is set to
        double a = currentPosition;
        // Should move in the positive direction
        if(currentPosition < targetPosition) {
            // Includes && opModeIsActive() for safety, and parameters / a variable for different movement pattens
            while(a < targetPosition && opModeIsActive()) {
                servo.setPosition(a);
                a += stepSize;
                sleep(delayStep);
            }
        }
        // Should move in the negative direction
        else {
            // Includes && opModeIsActive() for safety, and parameters / a variable for different movement pattens
            while(a > targetPosition && opModeIsActive()) {
                servo.setPosition(a);
                a -= stepSize;
                sleep(delayStep);
            }
        }
    }
}
