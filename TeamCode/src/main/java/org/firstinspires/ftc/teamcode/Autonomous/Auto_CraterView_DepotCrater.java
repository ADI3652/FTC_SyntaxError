package org.firstinspires.ftc.teamcode.Autonomous;
// Linear autonomous program imports

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareBot;

import java.util.Locale;

@Autonomous(name = "Auto - Crater view - Depot Crater", group = "SyntaxError")
public class Auto_CraterView_DepotCrater extends LinearOpMode {
    HardwareBot robot = new HardwareBot();
    private ElapsedTime runtime = new ElapsedTime();

    double distanceOffset = 0;

    @Override
    public void runOpMode() {
        /* When initialise button is pressed: */
        robot.init(hardwareMap, true, true);

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        
        /* Wait for start button to be pressed */
        waitForStart();
        /* When start button is pressed; run main code */

        /*
        Stage 1 - Latching off
         */
        telemetry.addData("Task", "Stage 1 - Latching off");
        telemetry.update();
        // Lower to the ground
        robot.liftMotor.setPower(0.5);
        sleep(2000);
        robot.liftMotor.setPower(0);

        // Escape the latch`
        // Move forward for a tiny little distance
        robot.encoderDrive(0.2, 2, 2);
        // Do a on the spot left turn
        robot.imuTurnLeft(0.2, 5, 2);

        /*
        Stage 2 - Getting to the minerals
         */
        telemetry.addData("Task", "Stage 2 - Getting to the minerals");
        telemetry.update();
        // Move backward so that the robot can get away from the lander
        robot.encoderDrive(0.5, 5, 2);
        // Turn right so that the robot is parallel to the lander
        robot.imuTurnRight(0.2, 5, 2);
        // Move backward so that the robot is close to the minerals and it's back is facing them
        robot.encoderDrive(0.5, 50, 5);
        // Turn left for 90 degrees in a zero point turn so that the
        // left of the robot is facing the minerals
        robot.imuTurnLeft(0.2, 90, 3);
        // Move backward so that the colour sensor can only see the first mineral at the moment
        robot.encoderDrive(0.2, 15, 3);

        /*
        Stage 3 - Sampling
         */
        /*
        telemetry.addData("Task", "Sampling");
        telemetry.update();
        // Loop the colour sensor readings to display to telementry and provide time for readings to settle
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        for(int i=0; i<20; i++) {
            // Converts the raw RGB values to HSV values, which are easier to use to detect ball colours
            Color.RGBToHSV((robot.colourSensor.red() * 255) / 800, (robot.colourSensor.green() * 255) / 800, (robot.colourSensor.blue() * 255) / 800, hsvValues);
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.distanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Clear", robot.colourSensor.alpha());
            telemetry.addData("Red  ", robot.colourSensor.red());
            telemetry.addData("Green", robot.colourSensor.green());
            telemetry.addData("Blue ", robot.colourSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            telemetry.update();
            if(!opModeIsActive()) break;
        }
        // Check if the first mineral is a gold cube using the hue value from the colour sensor
        if (hsvValues[0] >= 45 && hsvValues[0] <= 75) {
            robot.samplingServo.setPosition(robot.SAMPLING_SERVO_PUSH);
            robot.samplingServo.setPosition(robot.SAMPLING_SERVO_TUCK);
            distanceOffset = -15;
        }
        else {
            // Loop the colour sensor readings to display to telementry and provide time for readings to settle
            for(int i=0; i<20; i++) {
                // Converts the raw RGB values to HSV values, which are easier to use to detect ball colours
                Color.RGBToHSV((robot.colourSensor.red() * 255) / 800, (robot.colourSensor.green() * 255) / 800, (robot.colourSensor.blue() * 255) / 800, hsvValues);
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", robot.distanceSensor.getDistance(DistanceUnit.CM)));
                telemetry.addData("Clear", robot.colourSensor.alpha());
                telemetry.addData("Red  ", robot.colourSensor.red());
                telemetry.addData("Green", robot.colourSensor.green());
                telemetry.addData("Blue ", robot.colourSensor.blue());
                telemetry.addData("Hue", hsvValues[0]);
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });
                telemetry.update();
                if(!opModeIsActive()) break;
            }
            // Move forward so that the colour sensor can only see the second mineral at the moment
            robot.encoderDrive(0.2, 15, 3);
            // Check if the second mineral is a gold cube using the hue value from the colour sensor
            if (hsvValues[0] >= 45 && hsvValues[0] <= 75) {
                robot.samplingServo.setPosition(robot.SAMPLING_SERVO_PUSH);
                robot.samplingServo.setPosition(robot.SAMPLING_SERVO_TUCK);
                distanceOffset = 0;
            }
            else {
                // Loop the colour sensor readings to display to telementry and provide time for readings to settle
                for(int i=0; i<20; i++) {
                    // Converts the raw RGB values to HSV values, which are easier to use to detect ball colours
                    Color.RGBToHSV((robot.colourSensor.red() * 255) / 800, (robot.colourSensor.green() * 255) / 800, (robot.colourSensor.blue() * 255) / 800, hsvValues);
                    telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", robot.distanceSensor.getDistance(DistanceUnit.CM)));
                    telemetry.addData("Clear", robot.colourSensor.alpha());
                    telemetry.addData("Red  ", robot.colourSensor.red());
                    telemetry.addData("Green", robot.colourSensor.green());
                    telemetry.addData("Blue ", robot.colourSensor.blue());
                    telemetry.addData("Hue", hsvValues[0]);
                    relativeLayout.post(new Runnable() {
                        public void run() {
                            relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                        }
                    });
                    telemetry.update();
                    if(!opModeIsActive()) break;
                }
                // Move forward so that the colour sensor can only see the third mineral at the moment
                robot.encoderDrive(0.2, 15, 3);
                // Check if the third mineral is a gold cube using the hue value from the colour sensor
                if (hsvValues[0] >= 45 && hsvValues[0] <= 75) {
                    robot.samplingServo.setPosition(robot.SAMPLING_SERVO_PUSH);
                    robot.samplingServo.setPosition(robot.SAMPLING_SERVO_TUCK);
                    distanceOffset = 15;
                }
            }
        }
        // Move backwards so that the robot is clear of the minerals
        robot.encoderDrive(0.5, (50 + distanceOffset), 5);
        */
        /*
        Stage 3 - Sampling (alternative)
         */
        telemetry.addData("Task", "Sampling");
        telemetry.update();
        int rightMotorInitialPosition = robot.rightDrive.getCurrentPosition();
        boolean stop = false;
        while (stop == false) {
            // Converts the raw RGB values to HSV values, which are easier to use to detect ball colours
            Color.RGBToHSV((robot.colourSensor.red() * 255) / 800, (robot.colourSensor.green() * 255) / 800, (robot.colourSensor.blue() * 255) / 800, hsvValues);
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.distanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Clear", robot.colourSensor.alpha());
            telemetry.addData("Red  ", robot.colourSensor.red());
            telemetry.addData("Green", robot.colourSensor.green());
            telemetry.addData("Blue ", robot.colourSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            if (hsvValues[0] >= 45 && hsvValues[0] <= 75) {
                stop = true;
            }
            if(!opModeIsActive()) break;
        }
        // Turn left for 90 degrees to knock off the gold cube out of its original place
        robot.imuTurnLeft(0.3, 90, 5);
        // Move forward a little bit
        robot.encoderDrive(0.5, 5, 4);
        // Move backward a little bit
        robot.encoderDrive(0.5, -5, 4);
        // Turn right for 90 degrees to return to the orginal angle before knocking oof the gold cube
        robot.imuTurnRight(0.3, 90, 5);
        robot.encoderDriveToRightPosition(0.4, rightMotorInitialPosition, 6);

        /*
        Stage 4 - Team marker in Depot
         */
        telemetry.addData("Task", "Team marker in Depot");
        telemetry.update();
        // Turn right for 135 degrees in a zero point turn so that the robot is facing the depot
        robot.imuTurnRight(0.3, 135, 5);
        // Drive forwards so that the robot is in the depot
        robot.encoderDrive(0.7, 200, 10);
        // Change the servo position to drop off the team marker
        robot.markerServo.setPosition(robot.MARKER_SERVO_DROP);

        /*
        Stage 5 - Park in Crater
         */
        telemetry.addData("Task", "Park in Crater");
        telemetry.update();
        // Drive backwards so that the robot goes into the crater
        robot.encoderDrive(1, 300, 10);

    }

}
