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
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;



@TeleOp(name = "driveCode", group = "Pushbot")

public class ConnectionTeleop extends OpMode {


    static final double INCREMENT = 0.01; //The increment of speed in the servo
    double position = 0.0;
    private double speedDecrease = 2.0;
    private boolean reverse = false;
    private boolean reverese_pressed = false;

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
        telemetry.addData("Connection", "Starting");
    }

    public void loop() {
        double left;
        double right;
        double up;
       // double sides;


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        up = gamepad2.right_stick_y;
        //sides = -gamepad1.left_stick_x;

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

        robot.motor_left_front.setPower(-left / speedDecrease);
        robot.motor_left_back.setPower(-left / speedDecrease);
        robot.motor_right_front.setPower(-right / speedDecrease);
        robot.motor_right_back.setPower(-right / speedDecrease);
        robot.motor_elevator.setPower(up);
        //robot.motor_middle.setPower(sides);

        // Use gamepad up and down buttons to open and close the grip
        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.upper_grip.setPosition(0.0);//to be lucky this time
            robot.lower_grip.setPosition(0.0);
            //robot.closeGrip();
        }

        if (!gamepad2.dpad_up && gamepad2.dpad_down) {
            robot.upper_grip.setPosition(1.0);
            robot.lower_grip.setPosition(1.0);
            //robot.openGrip();
        }

        if (gamepad1.left_bumper && !gamepad1.right_bumper){
            robot.motor_middle.setPower(1.0);
        }

        if (!gamepad1.left_bumper && gamepad1.right_bumper){
            robot.motor_middle.setPower(-1.0);
        }
        if (!gamepad1.right_bumper && !gamepad1.right_bumper) {
            robot.motor_middle.setPower(0.0);
        }

        telemetry.addData("upper grip position ", robot.upper_grip.getPosition());
        telemetry.addData("lower grip position ", robot.lower_grip.getPosition());
        telemetry.addData("left ", left);
        telemetry.addData("right ", right);
        //telemetry.addData("sides ", sides / speedDecrease);
        telemetry.addData("reverse ", reverse);
        telemetry.update();

    }

}

