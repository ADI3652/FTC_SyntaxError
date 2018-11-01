package org.firstinspires.ftc.teamcode;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.util.concurrent.TimeUnit.*;


@Autonomous(name="AutoMode_SE", group="SyntaxError")
public class AutoModeSE extends OpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftMotor = null;
    private Servo servo0 = null;
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
        liftMotor.setPower(extend);
        TimeUnit.MILLISECONDS(3000);
        liftMotor.setPower(still);




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
