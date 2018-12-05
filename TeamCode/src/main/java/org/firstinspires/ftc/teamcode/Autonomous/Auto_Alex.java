package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HardwareBot;
@Autonomous(name = "Auto_Alex", group = "SyntaxError")
public class Auto_Alex extends LinearOpMode {
    HardwareBot robot = new HardwareBot();
    private ElapsedTime runtime = new ElapsedTime();
    double distanceOffset = 0;
    public static final double MARKER_SERVO_CLOSE = 0;
    public static final double MARKER_SERVO_OPEN = 1;

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
    public static final int hit_gold_and_into_depot = 3400;
    public static final int align_with_wall_after_marker = 800;
    public static final int final_run_depot_to_crater = 10000;
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, true, true);
        waitForStart();
        robot.liftMotor.setPower(0.9);
        sleep(linear_lift_extension_amount);
        robot.liftMotor.setPower(0);
        m_Left(0.5, turn_off_latch_amount);
        m_Backwards(1, move_off_latch_amount);
        m_Right(0.5, turn_back_from_latch);
        m_Backwards(1, align_with_center_amount);
        m_Left(0.5, straighten_for_hit_gold);
        m_Backwards(1, hit_gold_and_into_depot);
        robot.markerServo.setPosition(robot.MARKER_SERVO_OPEN);
        m_Right(1, align_with_wall_after_marker);
        m_Forward(1, final_run_depot_to_crater);
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
