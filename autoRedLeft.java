package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autoMain;

/**
 * Created by itay on 20/11/2017.
 */
@Autonomous(name="autoRedLeft", group="Pushbot")

public class autoRedLeft extends autoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        connectionRun(false, true);
    }
}
