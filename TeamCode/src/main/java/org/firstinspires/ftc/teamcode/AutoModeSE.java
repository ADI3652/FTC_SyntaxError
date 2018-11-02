package org.firstinspires.ftc.teamcode;


import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/*
<<<<<<< HEAD

=======
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static java.util.concurrent.TimeUnit.*;
>>>>>>> aa52579daf026eadf069dca2318fc59efdd2f5db
*/

@Autonomous(name="AutoMode_SE", group="SyntaxError")
public class AutoModeSE extends OpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftMotor = null;
    private Servo servo0 = null;
    private long runUntil = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized2");


        leftDrive  = hardwareMap.get(DcMotor.class, "MotorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorRight");
        liftMotor = hardwareMap.get(DcMotor.class, "MotorLift");
        servo0 = hardwareMap.get(Servo.class, "servo0");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


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
        // Setup a variable for each drive wheel to save power level for telemetry

        /*
        Robot detaches from  lander
         */
        int extend = 1;
        int retract = -1;
        int still = 0;
        /*
        liftMotor.setPower(extend);
        Thread.sleep(2000);
        liftMotor.setPower(still);
        */


        // New code
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 5) {
            liftMotor.setPower(-1);
        }
        liftMotor.setPower(0);

        /*

        // Drive for 2 seconds
        if (runUntil > System.currentTimeMillis())
            return;

        // Do other stuff
        liftMotor.setPower(extend);
        // If need to drive for 5 seconds
        runUntil = System.currentTimeMillis() + 5000;
        liftMotor.setPower(still);

        */

        /*
        Robot samples
         */
        /*
        Robot orients itself
         */
        /*
        Robot places marker
         */
        /*
        Robot goes to crater
         */
    }

    /*f
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
