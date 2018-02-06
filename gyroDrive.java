package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by itay on 20/11/2017.
 */
@Autonomous(name="gyroDrive", group="Connection")
public class gyroDrive extends AutoMain {
    //  todo IMPORTANT - placement is important! we need to code the position when the robot is on the correct position.
    @Override
    public void runOpMode() throws InterruptedException {
        connectionInit();
        robot.setMotorDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int angle = 90;
        //blue right - works!
        waitForStart();
        /*
        driveStraitWithEncoder(0.2, 1000);
        gyroDrive(-0.3, 3730, 0);
        gyroTurn(0.3, 90);
        gyroHold(0.3, 90, 0.5);
        gyroDrive(-0.3, 800, 90);
        gyroDrive(-0.3, -570, 90);
        robot.motor_elevator_twist.setPower(-0.5);
        sleep(1000);
        robot.motor_elevator_twist.setPower(0.5);
        sleep(1000);
        robot.motor_elevator_twist.setPower(0.0);
        sleep(1000);
        gyroDrive(-0.3, -200, 90);
        */

        //blue left - not tested
        /*
        gyroDrive(-0.3, 2500, 0);
        gyroTurn(0.3, 90);
        gyroHold(0.3, 90, 0.5);
        gyroDrive(-0.3, 1500, 90);
        gyroTurn(0.3, 180);
        gyroHold(0.3, 180, 0.5);
        gyroDrive(-0.3, 1000, 180);
        robot.motor_elevator_twist.setPower(-0.5);
        sleep(1000);
        robot.motor_elevator_twist.setPower(0.5);
        sleep(1000);
        robot.motor_elevator_twist.setPower(0.0);
        sleep(1000);
        gyroDrive(-0.3, -200, 90);
        */

        //red left - not tested
        /*
        driveStraitWithEncoder(0.2, 1000);
        gyroDrive(-0.3, -3715, 0);
        gyroTurn(0.3, 80);
        gyroHold(0.3, 80, 0.5);
        gyroDrive(-0.3, 800, 85);
        gyroDrive(-0.3, -500, 85);
        robot.motor_elevator_twist.setPower(-0.5);
        sleep(1000);
        robot.motor_elevator_twist.setPower(0.5);
        sleep(1000);
        robot.motor_elevator_twist.setPower(0.0);
        sleep(1000);
        gyroDrive(-0.3, -200, 80);
*/

        //red right
        /*
        gyroDrive(-0.3, 2500, 0);
        gyroTurn(0.3, 90);
        gyroHold(0.3, 90, 0.5);
        gyroDrive(-0.3, 1500, 90);
        gyroTurn(0.3, 180);
        gyroHold(0.3, 180, 0.5);
        gyroDrive(-0.3, 1000, 180);
        robot.motor_elevator_twist.setPower(-0.5);
        sleep(1000);
        robot.motor_elevator_twist.setPower(0.5);
        sleep(1000);
        robot.motor_elevator_twist.setPower(0.0);
        sleep(1000);
        gyroDrive(-0.3, -200, 90);
        */

        //new method try
        driveStraitWithEncoder(0.2, 1000);
        gyroDrive(-0.3, -3715, 0);
        gyroTurn(0.3, 90);
        gyroHold(0.3, 90, 0.5);
        gyroDrive(-0.3, 600, 0);
        robot.motor_elevator_twist.setPower(-0.3);
        sleep(1000);
        robot.motor_elevator_twist.setPower(0.3);
        sleep(1000);
        robot.motor_elevator_twist.setPower(0.0);
        sleep(1000);
        gyroDrive(-0.3, 400, 90);
    }
}
