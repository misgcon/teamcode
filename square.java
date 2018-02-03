/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is a testing code.
 * It tests the robot to see if it can make a square.
 * 'Making a square' is making the robot drive 2000
 * ticks and turn 90 degrees and repeat this 4 times.
 * The code's purposes is to test the encoders in the robot.
 * This code will not be used in any tournament so we will
 * disable it when we won't need it.
 * >This code uses LinearOpMode<
*/

@Autonomous(name="square", group="Pushbot")
public class square extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareConnection robot   = new HardwareConnection();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.3;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.setMotorDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Step through each leg of the path,todo!!! are we suposed to keep this comment or delete it
        /** Note: Reverse movement is obtained by setting a negative distance (not speed)*/
        for (int i = 2; i > 0; i--){//repeats the code twice
            driveStraitWithEncoder(DRIVE_SPEED, 2000);//This makes the robot drive strait
            turnWithEncoder(90, TURN_SPEED);//This turns the robot 90 degrees
            driveStraitWithEncoder(DRIVE_SPEED, 2000);
            turnWithEncoder(90, TURN_SPEED);
            driveStraitWithEncoder(DRIVE_SPEED, 2000);
            turnWithEncoder(90, TURN_SPEED);
            //driveStraitWithEncoder(DRIVE_SPEED, 2000);
        }


        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");//sends a message to show that the code has been acted out
        telemetry.update();
    }

//this part utilizes what we have stated above
    void driveStraitWithEncoder(double speed, int ticks) {
        //robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetEncoders();
        /* sets the distance that is desired to drive */
        int leftTargetBack = robot.motor_left_back.getCurrentPosition() + ticks;
        int leftTargetFront = robot.motor_left_front.getCurrentPosition() + ticks;
        int rightTargetBack = robot.motor_left_back.getCurrentPosition() + ticks;
        int rightTargetFront = robot.motor_left_back.getCurrentPosition() + ticks;

        robot.motor_left_back.setTargetPosition(leftTargetBack);
        robot.motor_left_front.setTargetPosition(leftTargetFront);
        robot.motor_right_back.setTargetPosition(rightTargetBack);
        robot.motor_right_front.setTargetPosition(rightTargetFront);
      /*dries towards the desired position */
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setAllMotorDrivePower(speed);
        /*waits for the motors to finish running*/
        while (opModeIsActive() &&
                (robot.motor_left_back.isBusy()) &&
                (robot.motor_right_front.isBusy()) &&
                (robot.motor_right_back.isBusy()) &&
                (robot.motor_left_front.isBusy())) {
            idle();
        }
        robot.setAllMotorDrivePower(0);//stops all motors
    }
     /*this is the code that allows the robot to turn using encoder*/
    /** just as a placeHolder, 1 degree of spin is 5 ticks, to turn left its positive degrees and to turn right its negative.*/
    void turnWithEncoder(int degree, double speed) {
        //robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetEncoders();//resets all encoders
        /*sets a target position to where to turn*/
        int leftTarget = -18 * degree;//we times the amount of degrees we want to spin by 18 to make the amount of tick nessesary to do so
        int rightTarget = 18 * degree;
        robot.motor_left_back.setTargetPosition(leftTarget);
        robot.motor_left_front.setTargetPosition(leftTarget);
        robot.motor_right_back.setTargetPosition(rightTarget);
        robot.motor_right_front.setTargetPosition(rightTarget);
       /*goes to the position that we assigned it*/
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setAllMotorDrivePower(speed);
      /*doen't do anything until the robot finished its cource*/
        while (opModeIsActive() &&
                (robot.motor_left_back.isBusy()) &&
                (robot.motor_right_front.isBusy()) &&
                (robot.motor_right_back.isBusy()) &&
                (robot.motor_left_front.isBusy())) {
            idle();
        }
        robot.setAllMotorDrivePower(0.0);//stops the motors
    }
}
