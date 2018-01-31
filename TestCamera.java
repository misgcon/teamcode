package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by itay on 20/11/2017.
 */

@Autonomous(name="Camera", group="Connection")

public class TestCamera extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        waitForStart();

        while (opModeIsActive()) {
            RelicRecoveryVuMark column = readPhoto();
            telemetry.addData("VuMark", "%s visible", column);
            telemetry.update();
        }
    }
}
