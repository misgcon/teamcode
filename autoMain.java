package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by itay on 15/11/2017.
 *
 * The common class for all the auto modes.
 * each auto mode should call function apolloRun.
 */
public abstract class AutoMain extends LinearOpMode {
    ColorSensor colorSensor;
    static private double FORWORD_SPEED = 0.4;
    private ElapsedTime runtime = new ElapsedTime();

    HardwareConnection robot = new HardwareConnection();

    void connectionRun(boolean isBlue, boolean leftSide){
        robot.init(hardwareMap);

        telemetry.addData("version: ", "4");
        telemetry.update();

        waitForStart();

        dropBall(isBlue);
        RelicRecoveryVuMark column = readPhoto();
        moveToCryptoBox(isBlue, leftSide);
        putCube(column);
        goToSafeZone();
    }

    // Balls task: Move the ball with the other color aside.
    private void dropBall(boolean isBlue) {
        boolean isBallBlue = false;
        colorSensor = hardwareMap.get(ColorSensor.class, "cSensor_ballArm");
        // TODO(): itay.s.
        robot.ballHandLift.setPosition(0.3);
        sleep(100);
        robot.ballHandLift.setPosition(0.8);
        sleep(1000);

        boolean foundColor = false;
        for (int i = 0; i < 3 ; i++) {
            if (colorSensor.blue() >= 24) {
                isBallBlue = true;
                foundColor = true;
                break;
            } else if (colorSensor.red() >= 24) {
                isBallBlue = false;
                foundColor = true;
                break;
            }

            robot.ballHandTurn.setPosition(-0.08);
            sleep(100);
        }

        if (foundColor) {
            if (isBlue == isBallBlue) {
                robot.ballHandTurn.setPosition(-0.7);
            } else {
                robot.ballHandTurn.setPosition(0.7);
            }
            sleep(1000);
        }

        robot.ballHandTurn.setPosition(0);
        sleep(1000);

        robot.ballHandLift.setPosition(0.3);
        sleep(100);
        robot.ballHandLift.setPosition(0);
        sleep(1000);
    }

    // Read photo and return the column to put the cube in.
    private RelicRecoveryVuMark readPhoto(){
        // TODO(): implement.
        return RelicRecoveryVuMark.RIGHT; // Place holder.
    }

    // Move to crypto box
    private void moveToCryptoBox(boolean isBlue, boolean leftSide){
        // TODO(): implement.
    }

    // Put the cube
    private void putCube (RelicRecoveryVuMark column){
        // TODO(): implement.
    }

    // Park the robot
    private void goToSafeZone (){
        // TODO(): implement.
    }

    void driveStraitWithEncoder(double speed, int ticks) {
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetEncoder();
        int leftTargetBack = robot.motor_left_back.getCurrentPosition() + ticks;
        int leftTargetFront = robot.motor_left_front.getCurrentPosition() + ticks;
        int rightTargetBack = robot.motor_left_back.getCurrentPosition() + ticks;
        int rightTargetFront = robot.motor_left_back.getCurrentPosition() + ticks;

        robot.motor_left_back.setTargetPosition(leftTargetBack);
        robot.motor_left_front.setTargetPosition(leftTargetFront);
        robot.motor_right_back.setTargetPosition(rightTargetBack);
        robot.motor_right_front.setTargetPosition(rightTargetFront);
        robot.setAllMotorDrivePower(speed);
        while (opModeIsActive() &&
                (robot.motor_left_back.isBusy()) &&
                (robot.motor_right_front.isBusy()) &&
                (robot.motor_right_back.isBusy()) &&
                (robot.motor_left_front.isBusy())) {
            idle();
        }
        robot.setAllMotorDrivePower(0);
    }

    // just as a placeHolder, 1 degree of spin is 5 ticks, to turn left its positive degrees and to turn right its negative.
    void turnWithEncoder(int degree, double speed) {
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetEncoder();
        int leftTargetBack = -5 * degree;
        int rightTargetBack = 5 * degree;
        int leftTargetFront = -5 * degree;
        int rightTargetFront = 5 * degree;
        robot.motor_left_back.setTargetPosition(leftTargetBack);
        robot.motor_left_front.setTargetPosition(leftTargetFront);
        robot.motor_right_front.setTargetPosition(rightTargetBack);
        robot.motor_right_back.setTargetPosition(rightTargetFront);
        robot.setAllMotorDrivePower(speed);
        while (opModeIsActive() &&
                (robot.motor_left_back.isBusy()) &&
                (robot.motor_right_front.isBusy()) &&
                (robot.motor_right_back.isBusy()) &&
                (robot.motor_left_front.isBusy())) {
            idle();
        }
        robot.setAllMotorDrivePower(0.0);
    }

}



