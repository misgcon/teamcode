package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by itay on 20/11/2017.
 */
@Autonomous(name="autoBlueRight", group="Pushbot")

public class autoblueRight extends autoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        connectionRun(true, false);
    }
}
