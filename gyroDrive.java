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

    @Override
    public void runOpMode() throws InterruptedException {
        connectionInit();
        robot.setMotorDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int angle = 90;
        waitForStart();
        gyroDrive(-0.3, 4730, 0);
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
    }
}
