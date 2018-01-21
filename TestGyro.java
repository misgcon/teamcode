package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by itay on 20/11/2017.
 */
@Autonomous(name="TestGyro", group="Connection")

public class TestGyro extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        connectionInit();
        waitForStart();

        while (opModeIsActive()) {
            Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("First: ", "%5.2f", orientation.firstAngle);
            telemetry.addData("Second: ", "%5.2f", orientation.secondAngle);
            telemetry.addData("Third: ", "%5.2f", orientation.thirdAngle);
            telemetry.update();
        }

    }
}
