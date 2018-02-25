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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

@TeleOp(name = "driveCode", group = "Connection")
public class ConnectionTeleop extends OpMode  {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    HardwareConnection robot = new HardwareConnection();
    private double speedDecrease = 1.5;
    private boolean reverse = false;
    private boolean reverese_pressed = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);//adds all the hardware

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Connection", "Starting");

    }

    public void loop() {
        double left;
        double right;
        double up;
        double twist;
        boolean spin = false;
        boolean spin_pressed = false;



        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        right = gamepad1.left_stick_y;
        left = gamepad1.right_stick_y;
        up = gamepad2.right_stick_y;
        final double Fleft = left;//this is a variable that is supposed to be Forever-left

        if (gamepad2.dpad_up) {//is true if the upper button in dpad is pressed
            twist = -0.3;//this drops the twister
        } else if (gamepad2.dpad_down) {//is true if the lower button in dpad is pressed
            twist = 0.3;//this elevates the twister
        } else {//if none of the above buttons are pressed
            twist = 0;//set two starting possition
        }
        robot.motor_elevator_twist.setPower(twist);//this changes the position of the thing that throws the cubes into the crypto box

        //reverse toggle
        if (gamepad1.y) {//if the 'y' button is pressed
            if (!reverese_pressed) { //if this is the first milisecond that it is pressed
                reverse = !reverse; //reverse equals to the false if true and true if false
                reverese_pressed = true; //this is equivalent to:"this isn't the first milisecond that 'y' is pressed"
            }
        } else {//if 'y' is not pressed
            reverese_pressed = false;//reset to this statement:"this is the first milisecond that it is pressed"
        }
       /* reverses the controls*/
        if (reverse) {
            left = -right;
            right = -Fleft;
        }

        //sets the power of the motors to the position of the joysticks
        robot.setLeftMotorDrivePower(left/speedDecrease);
        robot.setRightDrivePower(right/speedDecrease);
        robot.motor_elevator.setPower(-up);

        //toggles the spin
        if (gamepad2.right_bumper){ //if the right bumper pressed then you reverse the spin this is the same like the reverse ubove
            if (!spin_pressed){
                spin = !spin;
                spin_pressed = true;
            }
        }
        else {
            spin_pressed = false;
        }

        /*this changes the left_pickup to suck in or to throw out the cube*/
        double pick_left = 0;
        if (gamepad1.left_trigger > 0.3) {
            pick_left = -0.5;
        } else if (gamepad1.left_bumper) {
            pick_left = 0.5;
        }
        robot.cubePickUp_left.setPower(pick_left);
        /*this changes the right_pickup to suck in or to throw out the cube*/
        double pick_right = 0;
        if (gamepad1.right_trigger > 0.3) {
            pick_right = 0.5;
        } else if (gamepad1.right_bumper) {
            pick_right = -0.5;
        }
        robot.cubePickUp_right.setPower(pick_right);



        telemetry.update();
        telemetry.addData("left motor front", robot.motor_left_front.getCurrentPosition());
        telemetry.addData("left motor back", robot.motor_left_back.getCurrentPosition());
        telemetry.addData("right motor front", robot.motor_right_front.getCurrentPosition());
        telemetry.addData("right motor back", robot.motor_right_back.getCurrentPosition());
        //telemetry.addData("elevator", robot.motor_elevator_twist.getCurrentPosition());
        telemetry.addData("left stick", gamepad1.left_stick_y);
        telemetry.addData("right stick", gamepad1.right_stick_y);
    }

}

