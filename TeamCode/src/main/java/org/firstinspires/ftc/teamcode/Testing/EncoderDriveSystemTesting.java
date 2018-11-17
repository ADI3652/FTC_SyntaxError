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
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.HardwareBot;

import java.util.Locale;

// Allows for conversion to HSV (hue, saturation, value) from RGB (red, green blue)
// Required imports for program specific methods

@Autonomous(name = "Encoder drive system testing", group = "SyntaxError")
// @Disabled
public class EncoderDriveSystemTesting extends LinearOpMode {
    HardwareBot robot = new HardwareBot();
    private ElapsedTime runtime = new ElapsedTime();

    double distanceOffset = 0;

    @Override
    public void runOpMode() {
        /* When initialise button is pressed: */
        // Initialise relevant devices with IMU, color, vuforia and the jewel arm enabled
        robot.init(hardwareMap, true, false);

        /* Wait for start button to be pressed */
        waitForStart();

        /* When start button is pressed; run main code */
        // Move forward 50 cm
        robot.encoderDrive(0.5, 50, 10);
        // Move backward 50 cm
        robot.encoderDrive(0.5, 50, 10);
        //
    }
}