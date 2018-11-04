package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Linear Autonomous", group="Linear Opmode")

public class LinearAutonomous2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor liftMotor = null;
    private Servo servo0 = null;
    private Servo servo5 = null;
    private long runUntil = 0;
    int extend = 1;
    int retract = -1;
    int still = 0;

    @Override
    public void runOpMode() {

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        leftMotor  = hardwareMap.get(DcMotor.class, "MotorLeft");
        rightMotor = hardwareMap.get(DcMotor.class, "MotorRight");
        liftMotor = hardwareMap.get(DcMotor.class, "MotorLift");
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo5 = hardwareMap.get(Servo.class, "servo5");

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        double retract = -0.2;
        double still = 0;

        // Lower to the ground
        liftMotor.setPower(-1);
        sleep(3850);
        liftMotor.setPower(0);

        // Escape the latch
        // Move forward for a tiny little distance
        leftMotor.setPower(-0.5);
        rightMotor.setPower(-0.5);
        sleep(250);

        // Do a on the spot left turn
        leftMotor.setPower(-0.5);
        rightMotor.setPower(0.5);
        sleep(10000);

        // Move backward all the way into the crater / depo
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        sleep(2000);


        // Es

        //detaching from lander

        /*
        liftMotor.setPower(retract);
        sleep(4500);

        liftMotor.setPower(still);
        sleep(1500);
/*
        rightMotor.setPower(0.4);
        sleep(600);

        rightMotor.setPower(0);
        leftMotor.setPower(0.4);
        sleep(600);
*/


        /*
        // Unlatch from the lander
        rightMotor.setPower(0.5);
        sleep(500);
        leftMotor.setPower(0.5);
        sleep(500);

        //drive forward
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        sleep(3000);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        */
        /*
        if(button is one){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        */
        //if touch sensor = high, stop, place marker.



        }
    }

