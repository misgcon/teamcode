package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.net.PortUnreachableException;

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

        telemetry.addData("version: ", "4");

        telemetry.update();

        waitForStart();


        dropBall(isBlue, leftSide);

        Column column = readPhoto();
        moveToCryptoBox(isBlue, leftSide);
        putCube (column);
        goToSafeZone();

    }

    // Balls task: Move the ball with the other color aside.
    private void dropBall(boolean isBlue, boolean leftSide) {
        colorSensor = hardwareMap.get(ColorSensor.class, "cSensor_ballArm");
        // TODO(): implement.
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
        // TODO(): implement.




        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            idle();
        }
        robot.setALLMotorDrivePower(0.0);
    }

    void driveStrait(double speed, double seconds) {
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setALLMotorDrivePower(speed);
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
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetEncoder();
        int driveTarget = robot.motor_left.getCurrentPosition() + newTarget;
        robot.motor_left.setTargetPosition(driveTarget);
        robot.setALLMotorDrivePower(speed);
        while (opModeIsActive() && (robot.motor_left.isBusy())){
        idle();
        }
        robot.setALLMotorDrivePower(0);
    }
    // just as a placeHolder, 1 degree of spin is 5 ticks, to turn left its positive degrees and to turn right its negative.
    public void turnWithEncoder (int degree, double speed){
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetEncoder();
        int leftTarget = -5*degree;
        int rightTarget = 5*degree;
        robot.motor_left.setTargetPosition(leftTarget);
        robot.motor_right.setTargetPosition(rightTarget);
        robot.setALLMotorDrivePower(speed);
        while (opModeIsActive() && (robot.motor_left.isBusy()) && (robot.motor_right.isBusy())){
            idle();
        }
        robot.setALLMotorDrivePower(0);
    }
}



