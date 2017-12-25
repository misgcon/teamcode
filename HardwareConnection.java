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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareConnection {
    /* Public OpMode members. */
    public DcMotor motor_left;
    public DcMotor motor_right;
    public Servo ball_hand;
    public ColorSensor colorSensor;


    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareConnection() {
    }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motor_left = hwMap.get(DcMotor.class, "motor_left");
        motor_right = hwMap.get(DcMotor.class, "motor_right");
        ball_hand = hwMap.get(Servo.class, "ball_hand");

        // define and Initialize sensors
        colorSensor = hwMap.get(ColorSensor.class, "cSensor_ballArm");

        motor_left.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors(we have placed it in reverse)
        motor_right.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power

        setALLMotorDrivePower(0);

        ball_hand.setPosition(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setMotorDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void setALLMotorDrivePower(double speed) {  //short cut to power all the DC_motors
<<<<<<< HEAD
        motor_left.setPower(speed);
        motor_right.setPower(speed);
=======
        motor_left_front.setPower(speed);
        motor_left_back.setPower(speed);
        motor_right_front.setPower(speed);
        motor_right_back.setPower(speed);
        motor_middle.setPower(speed);
    }

    public void setLEFTMotorDrivePower(double speed) {  //short cut to power all the DC_motors
        motor_left_front.setPower(speed);
        motor_left_back.setPower(speed);
>>>>>>> parent of 65036c2... final code before 1 comp

    }

    public void setLEFTMotorDrivePower(double speed) {  //short cut to power the left DC_motors
        motor_left.setPower(speed);

    }

    public void setRIGHTMotorDrivePower(double speed) {  //short cut to power the right DC_motors
        motor_right.setPower(speed);
    }

    public void setMotorDriveMode(DcMotor.RunMode runMode) {
        motor_right.setMode(runMode);
        motor_left.setMode(runMode);
    }

    public void resetEncoder () {
        motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



}

