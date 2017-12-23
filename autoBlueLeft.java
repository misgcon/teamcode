package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by itay on 20/11/2017.
 */
@Autonomous(name="Auto", group="Pushbot")
//@Autonomous(name="autoBlueLeft", group="Pushbot")

public class autoBlueLeft extends org.firstinspires.ftc.teamcode.autoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        connectionRun(true, true);
    }
}
