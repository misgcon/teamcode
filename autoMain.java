package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by itay on 15/11/2017.
 *
 * The common class for all the auto modes.
 * each auto mode should call function apolloRun.
 */
public abstract class autoMain extends LinearOpMode {
    ColorSensor colorSensor;
    static private double FORWORD_SPEED = 0.2;
    static private double BACKWORD_SPEED = 0.2;
    private ElapsedTime runtime = new ElapsedTime();
    public enum Column {
        LEFT, CENTER, RIGHT
    }

    HardwareConnection robot = new HardwareConnection();

    void connectionRun(boolean isBlue, boolean leftSide){

        robot.init(hardwareMap);

        telemetry.addData("version: ", "3.4");

        telemetry.update();

        waitForStart();

        //driveStraitWithEncoder(FORWORD_SPEED, 1000);
        driveStrait(FORWORD_SPEED, 4.5);

        robot.setGripPosition(0);
        driveStrait(-FORWORD_SPEED, 0.5);



         /*
        dropBall(isBlue, leftSide);

        Column column = readPhoto();
        moveToCryptoBox(isBlue, leftSide);
        putCube (column);
        goToSafeZone();
        */
    }

    // Balls task: Move the ball with the other color aside.
    private void dropBall(boolean isBlue, boolean leftSide) {
        colorSensor = hardwareMap.get(ColorSensor.class, "cSensor_ballArm");
        // TODO(): Avital.

        /*
        robot.ball_hand.setPosition(0.5);
        sleep(1000);
        robot.ball_hand.setPosition(0.95);


        sleep(1000);

        runtime.reset();
        boolean isBallColorDetected = false;
        boolean isBallBlue = false;
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            if (robot.colorSensor.blue() >= 26) {
                isBallBlue = true;
                isBallColorDetected = true;
                break;
            }
            if (robot.colorSensor.red() >= 24) {
                isBallBlue = false;
                isBallColorDetected = true;
                break;
            }
            idle();
        }

        if (isBallColorDetected) {
            if (isBlue == isBallBlue) {
                driveStraitWithEncoder(FORWORD_SPEED,-150);
            } else {
                driveStraitWithEncoder(BACKWORD_SPEED,150);
            }
        }
        robot.ball_hand.setPosition(0.0);
        sleep(2000);

        */
        if (isBlue){
            driveStraitWithEncoder(FORWORD_SPEED, 1000);
            //driveStraitLEFT(FORWORD_SPEED, 0.07);
        }
        else {
            driveStraitWithEncoder(FORWORD_SPEED, -1000);
        }
    }

    // Read photo and return the column to put the cube in.
    private Column readPhoto(){
        // TODO(): implement.
        return Column.RIGHT; // Place holder.
    }

    // Move to crypto box
    private void moveToCryptoBox(boolean isBlue, boolean leftSide){
        // TODO(): implement.
    }

    // Put the cube
    private void putCube (Column column){
        // TODO(): implement.
    }

    // Park the robot
    private void goToSafeZone (){
        // TODO(): Itay.S

        robot.setALLMotorDrivePower(1.0);


        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            idle();
        }
        robot.setALLMotorDrivePower(0.0);
    }

    void driveStrait(double speed, double seconds) {
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setDriveMotorsPowerNoMiddle(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            idle();
        }
        robot.setALLMotorDrivePower(0);
    }

    void driveStraitLEFT (double speed, double seconds){
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setLEFTMotorDrivePower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            idle();
        }
        robot.setLEFTMotorDrivePower(0);
    }

    public void driveStraitToTargetWithEncoder (double speed, int newTarget){
        setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoder();
        int driveTarget = motor_left.getCurrentPosition() + newTarget;
        motor_left.setTargetPosition(driveTarget);
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())){

        }

    }
        robot.setALLMotorDrivePower(0);
    }


}
