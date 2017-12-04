package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by itay on 15/11/2017.
 *
 * The common class for all the auto modes.
 * each auto mode should call function apolloRun.
 */
public abstract class autoMain extends LinearOpMode {
    ColorSensor colorSensor;
    static private double FORWORD_SPEED = 1.0;
    private ElapsedTime runtime = new ElapsedTime();
    public enum Column {
        LEFT, CENTER, RIGHT
    }

    HardwareConnection robot = new HardwareConnection();

    void connectionRun(boolean isBlue, boolean leftSide){
        dropBall(isBlue);
        Column column = readPhoto();
        moveToCryptoBox(isBlue, leftSide);
        putCube (column);
        goToSafeZone();
    }

    // Balls task: Move the ball with the other color aside.
    private void dropBall(boolean isBlue) {
        // TODO(): Avital.
        robot.ball_hand.setPosition(0.0);
        robot.ball_hand.setPosition(0.6);
        int howBlue = colorSensor.blue();
        int howRed = colorSensor.red();
        if (isBlue) {
            if (colorSensor.blue() >= howBlue){
                robot.motor_right_back.setPower(FORWORD_SPEED);
                robot.motor_left_back.setPower(FORWORD_SPEED);
                robot.motor_right_front.setPower(FORWORD_SPEED);
                robot.motor_left_front.setPower(FORWORD_SPEED);

                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    idle();
                }
                robot.motor_right_back.setPower(0);
                robot.motor_left_back.setPower(0);
                robot.motor_right_front.setPower(0);
                robot.motor_left_front.setPower(0);
            }
            else if (colorSensor.blue() < howBlue) {
                robot.motor_right_back.setPower(-FORWORD_SPEED);
                robot.motor_left_back.setPower(-FORWORD_SPEED);
                robot.motor_right_front.setPower(-FORWORD_SPEED);
                robot.motor_left_front.setPower(-FORWORD_SPEED);

                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    idle();
                }
                robot.motor_right_back.setPower(0);
                robot.motor_left_back.setPower(0);
                robot.motor_right_front.setPower(0);
                robot.motor_left_front.setPower(0);
            }
        }
        else if (!isBlue) {
            if (colorSensor.red() >= howRed){
                robot.motor_right_back.setPower(FORWORD_SPEED);
                robot.motor_left_back.setPower(FORWORD_SPEED);
                robot.motor_right_front.setPower(FORWORD_SPEED);
                robot.motor_left_front.setPower(FORWORD_SPEED);

                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    idle();
                }
                robot.motor_right_back.setPower(0);
                robot.motor_left_back.setPower(0);
                robot.motor_right_front.setPower(0);
                robot.motor_left_front.setPower(0);
            }
            else if (colorSensor.blue() < howBlue) {
                robot.motor_right_back.setPower(-FORWORD_SPEED);
                robot.motor_left_back.setPower(-FORWORD_SPEED);
                robot.motor_right_front.setPower(-FORWORD_SPEED);
                robot.motor_left_front.setPower(-FORWORD_SPEED);

                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    idle();
                }
                robot.motor_right_back.setPower(0);
                robot.motor_left_back.setPower(0);
                robot.motor_right_front.setPower(0);
                robot.motor_left_front.setPower(0);
            }

        }

        robot.ball_hand.setPosition(0.0);

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


}
