package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by itay on 20/11/2017.
 */
@Autonomous(name="AutoBlueCorner", group="Connection")

public class AutoBlueLeft extends AutoMain {

    @Override
    public void runOpMode() throws InterruptedException {
        connectionRun(true, true); //Defines the boolean values of the isBlue boolean and the leftSide boolean
    }
}
