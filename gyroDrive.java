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
        final int MIDDLE_TICKS_LEFT_R = -3710;// works for red left
        final int LEFT_TICKS_LEFT_R = -4420;// works for red left
        final int RIGHT_TICKS_LEFT_R = -2800;//works red left
        final int MIDDLE_TICKS_RIGHT_R = -1760;// works for red left
        final int LEFT_TICKS_RIGHT_R = -2485;// works for red left
        final int RIGHT_TICKS_RIGHT_R = -800;//works red left

        final int MIDDLE_TICKS_RIGHT_B = -3710;//
        final int LEFT_TICKS_RIGHT_B = -4420;//
        final int RIGHT_TICKS_RIGHT_B = -2800;//
        final int MIDDLE_TICKS_LEFT_B = -1760;// works in blue left
        final int LEFT_TICKS_LEFT_B = -2485;//  works in blue left
        final int RIGHT_TICKS_LEFT_B = -800;//



        waitForStart();
        //Red Left - works!
        /*
        gyroDrive(-0.3, LEFT_TICKS, 0);
        gyroTurn(0.3, -90);
        gyroHold(0.3, -90, 0.5);
        gyroDrive(-0.3, -600, -90);
        robot.cubePickUpSpeed(-1.0);
        gyroDrive(-0.3, -1100, -90);
        robot.cubePickUpSpeed(0.0);
        gyroDrive(-0.3, 600, -90);
        */

        //red right - works!
        /*
        gyroDrive(-0.3, -2500, 0);
        gyroTurn(0.3, 90);
        gyroHold(0.3, 90, 0.5);
        gyroDrive(-0.3, LEFT_TICKS_RIGHT, 90);
        gyroTurn(0.3, 0);
        gyroHold(0.3, 0, 0.5);
        gyroDrive(-0.3, -700, 90);
        robot.cubePickUpSpeed(-1.0);
        gyroDrive(-0.3, -800, 0);
        robot.cubePickUpSpeed(0.0);
        gyroDrive(-0.3, 480, 0);
        robot.cubePickUpSpeed(0.0);
        */

        gyroDrive(-0.3, 3400, 0);
        gyroTurn(0.3, 90);
        gyroHold(0.3, 90, 0.5);
        gyroDrive(-0.3, MIDDLE_TICKS_LEFT_B - 190, 90);
        gyroTurn(0.3, 180);
        gyroHold(0.3, 180, 0.5);
        gyroDrive(-0.3, -800, 180);
        robot.cubePickUpSpeed(-1.0);
        gyroDrive(-0.3, -1000, 180);
        robot.cubePickUpSpeed(0.0);
        gyroDrive(-0.3, 480, 180);
        robot.cubePickUpSpeed(0.0);


    }
}
