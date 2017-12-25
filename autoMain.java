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
    static private double FORWORD_SPEED = 0.4;
    private ElapsedTime runtime = new ElapsedTime();
    public enum Column {
        LEFT, CENTER, RIGHT
    }

    HardwareConnection robot = new HardwareConnection();

    void connectionRun(boolean isBlue, boolean leftSide){

        robot.init(hardwareMap);

<<<<<<< HEAD
        telemetry.addData("version: ", "4");

        telemetry.update();

        waitForStart();


        dropBall(isBlue, leftSide);

=======
        waitForStart();

        dropBall(isBlue);
>>>>>>> parent of 65036c2... final code before 1 comp
        Column column = readPhoto();
        moveToCryptoBox(isBlue, leftSide);
        putCube (column);
        goToSafeZone();
<<<<<<< HEAD

=======
>>>>>>> parent of 65036c2... final code before 1 comp
    }

    // Balls task: Move the ball with the other color aside.
    private void dropBall(boolean isBlue) {
        colorSensor = hardwareMap.get(ColorSensor.class, "cSensor_ballArm");
<<<<<<< HEAD
        // TODO(): implement.
=======
        // TODO(): Avital.
        robot.ball_hand.setPosition(1);
        sleep(1000);
        robot.ball_hand.setPosition(2);
        /*for (int i = 0; i >= 100; i++){
           int x = 1;
           robot.ball_hand.setPosition(x);
           x++;
           sleep(100);
        }
        */

        sleep(1000);

        runtime.reset();
        boolean isBallColorDetected = false;
        boolean isBallBlue = false;
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            if (robot.colorSensor.blue() >= 27) {
                isBallBlue = true;
                isBallColorDetected = true;
                break;
            }
            if (robot.colorSensor.red() >= 27) {
                isBallBlue = false;
                isBallColorDetected = true;
                break;
            }
            idle();
        }

        if (isBallColorDetected) {
            if (isBlue == isBallBlue) {
                driveStrait(-FORWORD_SPEED, 0.3);
            } else {
                driveStrait(FORWORD_SPEED, 0.3);
            }
        }
        robot.ball_hand.setPosition(0.0);
        sleep(2000);
        driveStrait(FORWORD_SPEED, 3);
>>>>>>> parent of 65036c2... final code before 1 comp
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

<<<<<<< HEAD
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

    void driveStraitWithEncoder(double speed, int ticks) {
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setALLMotorDrivePower(ticks > 0 ? speed : -speed);

        int targetPosition = robot.motor_left_back.getCurrentPosition() + ticks;
        while (opModeIsActive()) {
            if (ticks > 0 && robot.motor_left_back.getCurrentPosition() >= targetPosition) {
                break;
            }
            if (ticks < 0 && robot.motor_left_back.getCurrentPosition() <= targetPosition) {
                break;
            }

        }
        robot.setALLMotorDrivePower(0);
    }
}



