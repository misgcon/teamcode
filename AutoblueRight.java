package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by itay on 20/11/2017.
 */
@Autonomous(name="AutoBlueCenter", group="Connection")

public class AutoblueRight extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        connectionRun(true, false);
    }
}
