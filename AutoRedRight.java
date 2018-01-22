package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by itay on 20/11/2017.
 */
@Autonomous(name="AutoRedCorner", group="Connection")

public class AutoRedRight extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        connectionRun(false, false);
    }
}
