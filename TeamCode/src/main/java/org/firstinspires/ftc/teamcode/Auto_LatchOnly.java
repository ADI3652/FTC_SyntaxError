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

@Autonomous(name = "Auto - Latch Only", group = "SyntaxError")
public class Auto_LatchOnly extends LinearOpMode {
    HardwareBot  robot        = new HardwareBot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        /* When initialise button is pressed: */
        // Initialise relevant devices with IMU, color, vuforia and the jewel arm enabled
        robot.init(hardwareMap, true, true);

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        
        /* Wait for start button to be pressed */
        waitForStart();
        /* When start button is pressed; run main code */

        // Lower to the ground
        robot.liftMotor.setPower(-1);
        sleep(3200);
        robot.liftMotor.setPower(0);

        // Escape the latch
        // Move forward for a tiny little distance
        robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);
        sleep(250);
        // Do a on the spot left turn
        robot.leftDrive.setPower(-0.3);
        robot.rightDrive.setPower(0.3);
        sleep(350);
    }
}
