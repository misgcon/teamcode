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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Connection robot.
 */
public class HardwareConnection {
    /* Public OpMode members. */
    public DcMotor motor_left_front;
    public DcMotor motor_left_back;
    public DcMotor motor_right_front;
    public DcMotor motor_right_back;
    public DcMotor motor_elevator;
    public DcMotor motor_elevator_twist;
    public Servo ballHandLift;
    public Servo ballHandTurn;
    public ColorSensor colorSensor;
    public DcMotor cubePickUp_left;
    public DcMotor cubePickUp_right;
    BNO055IMU imu;
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
        motor_left_back = hwMap.get(DcMotor.class, "dlb"); // R1 M2
        motor_left_front = hwMap.get(DcMotor.class, "dlf"); // R1 M0
        motor_right_back = hwMap.get(DcMotor.class, "drb");  // R1 M3
        motor_right_front = hwMap.get(DcMotor.class, "drf");  // R1 M1
        motor_elevator = hwMap.get(DcMotor.class, "lift");
        motor_elevator_twist = hwMap.get(DcMotor.class, "twist");  // R2 M2
        cubePickUp_left = hwMap.get(DcMotor.class, "cal");  // R2 M0
        cubePickUp_right = hwMap.get(DcMotor.class, "car");  // R2 M1
        ballHandLift = hwMap.get(Servo.class, "bl"); // R1 P0
        ballHandTurn = hwMap.get(Servo.class, "bt"); // R1 P1

        // define and Initialize sensors
        colorSensor = hwMap.get(ColorSensor.class, "bcs");  // R1 I2C 0 REV Color
        //some motors needs to be reverse to drive strait - sets the motors to their identification position
        motor_left_back.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motor_left_front.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors(we have placed it in reverse)
        motor_right_back.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor_right_front.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motor_elevator.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        cubePickUp_left.setDirection(DcMotorSimple.Direction.REVERSE);
        cubePickUp_right.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_elevator_twist.setDirection(DcMotorSimple.Direction.FORWARD);

        //sets All Motor Drive Power to zero
        motor_elevator.setPower(0);
        cubePickUp_right.setPower(0);
        cubePickUp_left.setPower(0);
        motor_elevator_twist.setPower(0);
        //sets the dcmotors that don't use incoder to 'RUN_WITHOUT_ENCODER'
        //setMotorDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cubePickUp_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cubePickUp_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        motor_elevator_twist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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


    public void setMotorDriveMode(DcMotor.RunMode runMode) {//sets the drive mode of all the motors and makes them drive without encoder
        motor_right_back.setMode(runMode);
        motor_left_back.setMode(runMode);
        motor_right_front.setMode(runMode);
        motor_left_front.setMode(runMode);
    }

    public void resetEncoder () {//resets the ecoder in the motors that have encoder
        motor_right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void prepareForStart() {//prepares the robot and or code for start
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void cubePickUpSpeed(double Speed) {
        cubePickUp_right.setPower(Speed);
        cubePickUp_left.setPower(-Speed);
    }
}

