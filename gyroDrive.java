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
        final int MIDDLE_TICKS = -3710;// works for red left
        final int LEFT_TICKS = -4420;// works for red left
        final int RIGHT_TICKS = -2840;//works but only tested once for red left
//

        waitForStart();
        //Red Left - works!

        gyroDrive(-0.3, RIGHT_TICKS, 0);
        gyroTurn(0.3, -90);
        gyroHold(0.3, -90, 0.5);
        gyroDrive(-0.3, -600, -90);
        robot.cubePickUpSpeed(-1.0);
        gyroDrive(-0.3, -1100, -90);
        robot.cubePickUpSpeed(0.0);
        gyroDrive(-0.3, 600, -90);
    }
}
