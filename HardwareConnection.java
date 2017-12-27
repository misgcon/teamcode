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
 * This class can be used to define all the specific hardware for the Connection robot.
 */
public class HardwareConnection {
    /* Public OpMode members. */
    public DcMotor motor_left_front;
    public DcMotor motor_left_back;
    public DcMotor motor_right_front;
    public DcMotor motor_right_back;
    //public DcMotor motor_elevator;
    //public DcMotor left_grip;
    //public DcMotor right_grip;
    public Servo ballHandLift;
    public Servo ballHandTurn;
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
        motor_left_back = hwMap.get(DcMotor.class, "dlb");
        motor_left_front = hwMap.get(DcMotor.class, "dlf");
        motor_right_back = hwMap.get(DcMotor.class, "drb");
        motor_right_front = hwMap.get(DcMotor.class, "drf");
        //motor_elevator = hwMap.get(DcMotor.class, "motor_grip_lifter");
        //left_grip = hwMap.get(DcMotor.class, "upper_grip");
        //right_grip = hwMap.get(DcMotor.class, "lower_grip");
        ballHandLift = hwMap.get(Servo.class, "bx");
        ballHandTurn = hwMap.get(Servo.class, "by");

        // define and Initialize sensors
        colorSensor = hwMap.get(ColorSensor.class, "cSensor_ballArm");

        motor_left_back.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor_left_front.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors(we have placed it in reverse)
        motor_right_back.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motor_right_front.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //motor_elevator.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

        setAllMotorDrivePower(0);
        //motor_elevator.setPower(0);
        //setGripSpeed(0.0);

        setMotorDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor_elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setAllMotorDrivePower(double speed) {  //short cut to power all the DC_motors
        motor_left_back.setPower(speed);
        motor_right_front.setPower(speed);
        motor_left_front.setPower(speed);
        motor_right_back.setPower(speed);
    }

    public void setLeftMotorDrivePower(double speed) {  //short cut to power all the DC_motors
        motor_left_front.setPower(speed);
        motor_left_back.setPower(speed);
    }
    public void setRightDrivePower(double speed) {  //short cut to power all the DC_motors
        motor_right_front.setPower(speed);
        motor_right_back.setPower(speed);
    }

    /*public void setGripSpeed(double speed) {
        left_grip.setPower(speed);
        right_grip.setPower(speed);

    }
    */

    public void setMotorDriveMode(DcMotor.RunMode runMode) {
        motor_right_front.setMode(runMode);
        motor_right_back.setMode(runMode);
        motor_left_front.setMode(runMode);
        motor_left_back.setMode(runMode);
    }

    public void resetEncoder () {
        motor_right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

