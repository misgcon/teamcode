package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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
    static private double FORWORD_SPEED = 0.4;
    private ElapsedTime runtime = new ElapsedTime();
    boolean isBallBlue;
    public enum Column {
        LEFT, CENTER, RIGHT
    }

    HardwareConnection robot = new HardwareConnection();

    void connectionRun(boolean isBlue, boolean leftSide){

        robot.init(hardwareMap);

        telemetry.addData("version: ", "4");

        telemetry.update();

        waitForStart();
        dropBall(isBlue);
        waitForStart();
        dropBall(isBlue);
        Column column = readPhoto();
        moveToCryptoBox(isBlue, leftSide);
        putCube (column);
        goToSafeZone();

    }

    // Balls task: Move the ball with the other color aside.
    private void dropBall(boolean isBlue) {
        colorSensor = hardwareMap.get(ColorSensor.class, "cSensor_ballArm");
        // TODO(): itay.s.
        robot.ballHand_X.setPosition(0.3);
        sleep(100);
        robot.ballHand_X.setPosition(0.8);
        sleep(1000);
        if (colorSensor.blue() >= 24) {
            isBallBlue = true;
        }
        else {
            robot.ballHand_Y.setPosition(0.08);
            sleep(100);
            if (colorSensor.blue() >= 24) {
                isBallBlue = true;
            }
            else {
                isBallBlue = false;
            }
        }

        //=====================================
        if (isBlue == isBallBlue){
            robot.ballHand_Y.setPosition(0.7);
            sleep(1000);
        }
        else {
            robot.ballHand_Y.setPosition(-0.7);
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
        // TODO(): implement.

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

     void driveStraitEncoder (double speed, int newTarget){
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetEncoder();
        int leftTargetBack = robot.motor_left_back.getCurrentPosition() + newTarget;
        int leftTargetFront = robot.motor_left_front.getCurrentPosition() + newTarget;
        int rightTargetBack = robot.motor_left_back.getCurrentPosition() + newTarget;
        int rightTargetFront = robot.motor_left_back.getCurrentPosition() + newTarget;

        robot.motor_left_back.setTargetPosition(leftTargetBack);
        robot.motor_left_front.setTargetPosition(leftTargetFront);
        robot.motor_right_back.setTargetPosition(rightTargetBack);
        robot.motor_right_front.setTargetPosition(rightTargetFront);
        robot.setALLMotorDrivePower(speed);
        while (opModeIsActive() && (robot.motor_left_back.isBusy()) && (robot.motor_right_front.isBusy()) && (robot.motor_right_back.isBusy())
                && (robot.motor_left_front.isBusy())){
        idle();
        }
        robot.setALLMotorDrivePower(0);
    }
    // just as a placeHolder, 1 degree of spin is 5 ticks, to turn left its positive degrees and to turn right its negative.
     void turnWithEncoder (int degree, double speed){
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetEncoder();
        int leftTargetBack = -5*degree;
        int rightTargetBack = 5*degree;
        int leftTargetFront = -5*degree;
        int rightTargetFront = 5*degree;
        robot.motor_left_back.setTargetPosition(leftTargetBack);
        robot.motor_left_front.setTargetPosition(leftTargetFront);
        robot.motor_right_front.setTargetPosition(rightTargetBack);
        robot.motor_right_back.setTargetPosition(rightTargetFront);
        robot.setALLMotorDrivePower(speed);
        while (opModeIsActive() && (robot.motor_left_back.isBusy()) && (robot.motor_right_front.isBusy()) && (robot.motor_right_back.isBusy())
                && (robot.motor_left_front.isBusy())){
        idle();
        }
        robot.setALLMotorDrivePower(0.0);
    }

}



