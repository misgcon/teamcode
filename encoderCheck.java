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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
//@Disabled
@TeleOp(name = "encoderCheck", group = "Connection")
public class encoderCheck extends OpMode {
    /* Declare OpMode members. */
    HardwareConnection robot = new HardwareConnection();


    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.update();
        robot.resetEncoder();
        robot.motor_right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motor_elevator_twist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Connection", "Starting");
    }

    public void loop() {
        double left;
        double right;
        //double up;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        //up = gamepad1.right_stick_x;


        robot.setRightDrivePower(right/2);
        robot.setLeftMotorDrivePower(left/2);

        //robot.motor_elevator_twist.setPower(up/2);

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

