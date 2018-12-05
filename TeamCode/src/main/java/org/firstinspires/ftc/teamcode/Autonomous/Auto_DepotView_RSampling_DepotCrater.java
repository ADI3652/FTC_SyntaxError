package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HardwareBot;

@Autonomous(name = "Auto - Crater Jack view", group = "SyntaxError")
public class Auto_DepotView_RSampling_DepotCrater extends LinearOpMode {
    HardwareBot robot = new HardwareBot();
    private ElapsedTime runtime = new ElapsedTime();

    public static final double MARKER_SERVO_CLOSE = 0;
    public static final double MARKER_SERVO_OPEN = 1;
    public static final double MINERAL_SERVO_DOWN= 0;
    public static final double MINERAL_SERVO_UP = 1;

    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    /**
     * Variables that you should be changing
     */
    public static final int linear_lift_extension_amount = 4600;
    public static final int turn_off_latch_amount = 800;
    public static final int move_off_latch_amount = 200;
    public static final int turn_back_from_latch = 1600;
    public static final int align_with_center_amount = 800;
    public static final int straighten_for_hit_gold = 850;
    public static final int hit_gold_and_into_depot = 3000;
    public static final int align_with_wall_after_marker = 820;
    public static final int final_run_depot_to_crater = 10000;
    public static final int turning_to_wall = 1800;
    public static final int move_to_edge = 3000;
    public static final int face_depot = 990;
    public static final int depot_to_crater_time = 10000;
    public static final int from_depot_to_crater_halfway = 3000;
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, true, true);
        int NewCM = 1;
        int NewD = 1;

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
        telemetry.addData("Task", "Stage 1 - Lowering");
        telemetry.update();
        // Lower to the ground

        robot.liftMotor.setPower(0.9);
        sleep(linear_lift_extension_amount);
        robot.liftMotor.setPower(0);
        m_Left(0.5, turn_off_latch_amount);
        m_Backwards(1, move_off_latch_amount);
        m_Right(0.5, turn_back_from_latch);
        m_Backwards(1, align_with_center_amount);
        m_Left(0.5, straighten_for_hit_gold);


        // punching mineral
        m_Backwards(1,1500);
        m_Forward(1,1100);

        //turn 90 degrees to the left
        m_Left(0.7,turning_to_wall);

        //move to edge
        m_Backwards(1, move_to_edge);

        //turn 45 degrees to the left
        m_Left(0.7,face_depot);

        //move to depot
        m_Backwards(1,4000);

        //place marker
        robot.markerServo.setPosition(robot.MARKER_SERVO_OPEN);

        //move to crater
        m_Forward(1,from_depot_to_crater_halfway);

        //re-adjust
        m_Left(0.7,200);

        //forward again
        m_Forward(1,depot_to_crater_time);
        /*
        telemetry.addData("Task", "Stage 2 - Unlatching");
        // escaping latch
        robot.leftDrive.setPower(-0.5);
        robot.rightDrive.setPower(0.5);
        sleep(800);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // moving backward
        m_Backwards(1,500);

        // aligning
        m_Left(0.5,500);
        */

    }

    public void m_Backwards(double pwr, int amt) {
        robot.leftDrive.setPower(pwr);
        robot.rightDrive.setPower(pwr);
        sleep(amt);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    public void m_Forward(double pwr, int amt) {
        robot.leftDrive.setPower(-pwr);
        robot.rightDrive.setPower(-pwr);
        sleep(amt);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    public void m_Left(double pwr, int amt) {
        robot.leftDrive.setPower(-pwr);
        robot.rightDrive.setPower(pwr);
        sleep(amt);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    public void m_Right(double pwr, int amt) {
        robot.leftDrive.setPower(pwr);
        robot.rightDrive.setPower(-pwr);
        sleep(amt);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

}
