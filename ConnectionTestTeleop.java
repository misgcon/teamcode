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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Pushbot: Teleop Tank", group = "Pushbot")
//@Disabled
public class ConnectionTestTeleop extends OpMode {


    static final double INCREMENT = 0.01; //The increment of speed in the servo
    double position = 0.0;
    double speedDecrease = 2.0;
    boolean reverse = false;
    boolean reverese_pressed = false;

    /* Declare OpMode members. */
    HardwareConnection robot = new HardwareConnection(); // use the class created to define a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        /*robot.upper_grip.setPosition(0);
        robot.lower_grip.setPosition(0);


        if (robot.upper_grip.getPosition() != 0.00){
            telemetry.addData("upper servo", "not at zero");

        }

        if (robot.lower_grip.getPosition() != 0.00){
            telemetry.addData("lower servo", "not at zero");

        }
    */
    }

    public void loop() {
        double left;
        double right;
        double sides;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        sides = -gamepad1.left_stick_x;

        if (gamepad1.y) {
            if (!reverese_pressed) {
                reverse = !reverse;
                reverese_pressed = true;
            }
        } else {
            reverese_pressed = false;
        }


        if (reverse) {
            left = -left;
            right = -right;
        }

        robot.motor_left_front.setPower(left / speedDecrease);
        robot.motor_left_back.setPower(left / speedDecrease);
        robot.motor_right_front.setPower(right / speedDecrease);
        robot.motor_right_back.setPower(right / speedDecrease);
        robot.motor_middle.setPower(sides / speedDecrease);

        // Use gamepad up and down buttons to open and close the grip
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            robot.upper_grip.setPosition(1.0);
            robot.lower_grip.setPosition(1.0);
        }

        if (!gamepad1.dpad_up && gamepad1.dpad_down) {
            robot.upper_grip.setPosition(0.0);
            robot.lower_grip.setPosition(0.0);
        }

        telemetry.addData("upper grip position ", robot.upper_grip.getPosition());
        telemetry.addData("lower grip position ", robot.lower_grip.getPosition());
        telemetry.addData("left ", left);
        telemetry.addData("right ", right);
        telemetry.addData("sides ", sides / speedDecrease);
        telemetry.addData("reverse ", reverse);
        telemetry.update();

    }

}

