
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by itay on 15/11/2017.
 * The common class for all the auto modes.
 * each auto mode should call function apolloRun.
 */
public abstract class AutoMain extends LinearOpMode {
    static private double FORWORD_SPEED = 0.5; // A constant value for the speed of the DC motors.
    static final double HEADING_THRESHOLD = 1 ; // For the Gyro sensor.
    static final double P_TURN_COEFF = 0.1; // For the Gyro sensor.
    static final double P_DRIVE_COEFF = 0.15; // For the Gyro sensor.

    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables; // Initialises the PictoGraph reader.

    HardwareConnection robot = new HardwareConnection(); // Calls for a hardware.

    // A function to initialize the hardware and the photo reader.
    public void connectionInit() {
        robot.init(hardwareMap);
        initVuforia();
    }

    // The main autonomous code - uses the boolean values from the "sub" classes and the code uses them to modify the autonomous code.
    void connectionRun(boolean isBlue, boolean leftSide) {

        telemetry.addData("version: ", "6"); // Just to know when the code is ready to run.
        telemetry.update();

        waitForStart();
        robot.prepareForStart();
        dropBall(isBlue); // The first step in autonomous - dropping the ball of the opposite color.
        RelicRecoveryVuMark column = readPhoto(); // Second step in autonomous - reading the pictograph for putting the cube in the Cryptobox
        moveToCryptoBox(isBlue, leftSide, column); // Third step in autonomous - driving to the correct column
        putCube(column); // Last step in autonomous - dropping the cube in the correct column and getting ready for teleop

    }

    // Move the ball with the other color aside.
    public void dropBall(boolean isBlue) {
        boolean isBallBlue = false;
        // TODO(): itay.s.
        robot.ballHandTurn.setPosition(0.55);// makes sure that the hand is in the middle.
        sleep(500); // Delay for making sure its not going to much.
        robot.ballHandLift.setPosition(0.3); // Drops the ball hand with slowly with delay between for a slow drop.
        sleep(500);
        robot.ballHandLift.setPosition(0.6);
        sleep(500);
        robot.ballHandLift.setPosition(0.8);
        sleep(500);
        robot.ballHandLift.setPosition(0.9);
        sleep(500);
        robot.ballHandLift.setPosition(1.0); //Now its between the jewels.
        sleep(1000); // Waits for the sensors to see color.

        boolean foundColor = false; // For making sure the sensor saw the color
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        // Waits another second for the sensor to see the color.
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            // Uses two Conditions for making sure the sensor saw the correct color and sent correct value.
            if (robot.colorSensor.red() > robot.colorSensor.blue() && robot.colorSensor.red() > 30){
                isBallBlue = false; // For dropping the ball.
                foundColor = true; // For knowing that the sensor saw color
            }
            //Same for blue.
            else if (robot.colorSensor.blue() > robot.colorSensor.red() && robot.colorSensor.blue() > 30){
                isBallBlue = true;
                foundColor = true;
            }
        }
        idle();

        if (foundColor) {
            if (isBlue == isBallBlue) {//if you are blue and the ball is blue
                robot.ballHandTurn.setPosition(0.7);//drop the blue ball that ur facing
            } else {
                robot.ballHandTurn.setPosition(0.4);//drop the red ball
            }
            sleep(1000);
        }


        //robot.ballHandTurn.setPosition(0.4);
        //sleep(500);
        //robot.ballHandTurn.setPosition(0.5);
        //sleep(500);
        robot.ballHandLift.setPosition(0.5);
        sleep(2000);
        robot.ballHandTurn.setPosition(0.55);

        sleep(1000);

        robot.ballHandLift.setPosition(0.3);
        sleep(1000);
        robot.ballHandTurn.setPosition(0.05);
        sleep(1000);
        robot.ballHandLift.setPosition(0.2);
        sleep(1000);
        robot.ballHandLift.setPosition(0.1);
        sleep(1000);
    }

    // Read photo and return the column to put the cube in.
    public RelicRecoveryVuMark readPhoto() {
        //relicTrackables.activate();
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive()) {
            RelicRecoveryVuMark column = readPhoto();
            telemetry.addData("VuMark", "%s visible", column);
            telemetry.update();
        }
        return RelicRecoveryVuMark.UNKNOWN;
    }

    // Move to crypto box
    private void moveToCryptoBox(boolean isBlue, boolean leftSide, RelicRecoveryVuMark column) {
        // TODO(): implement.
        robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (isBlue) {
            if (leftSide) {

            }
            else {
                gyroDrive(-0.3, 3000, 0);
                gyroTurn(-0.3, -90);
                gyroHold(-0.3, -90, 0.5);
                gyroDrive(-0.3, 1500, 90);
            }

        }
        else //if not blue aka;<red> {
            if (leftSide) {

            }
            else {

            }
    }

    // Put the cube
    private void putCube(RelicRecoveryVuMark column) {
        // TODO(): implement.


    }

    //init vuforia
    public void initVuforia() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ac7hmPf/////AAAAmY1DFiTs+kenqbX9NbXujRAvP71bvNIwaSEsWB5HTXOb74TMHFnW9TqU/HACnA2BirmMCxCdqFJ0+Wby1+PpOLEUIjc7aSMOFF0/BUClZ5OEVeGvvfEBH4G2EkIt5tfGYhX9S4V+rnlTV6uBjSdRF8hh2XSK2oXkWWvnOGaoOJU+ku+QVwMQS/Gk4JyX0bLbgAIqGjJ3+y2Vwlqzui41Kzbc9zJgjugdvIFrOUE74mxhsEOTO7qwf6V+jeUURInrek5ycrp2weRWjJoZON0p3m1XQ/G0KwL3gTz+KMGQeoVNA76IcwRjXQBPBNKACJyCCQ29JryL84Qvf3FOll2nD5VNGL7j29wYXS01CmuaOFk0\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

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
        int leftTargetBack = -28 * degree;
        int rightTargetBack = 28 * degree;
        robot.motor_left_back.setTargetPosition(leftTargetBack);
        robot.motor_right_front.setTargetPosition(rightTargetBack);
        robot.motor_right_front.setPower(speed);
        robot.motor_left_front.setPower(speed);
        while (opModeIsActive() &&
                (robot.motor_right_front.isBusy()) &&
                (robot.motor_left_front.isBusy())) {
            idle();
        }
        robot.setAllMotorDrivePower(0.0);
    }

    void twistElevator (int degree, double speed){
        robot.motor_elevator_twist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_elevator_twist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int targetTwist = 5 * degree;
        robot.motor_elevator_twist.setTargetPosition(targetTwist);
        robot.motor_elevator_twist.setPower(speed);
        while (opModeIsActive() && robot.motor_elevator_twist.isBusy()){
            idle();
        }
        robot.motor_elevator_twist.setPower(0);
    }


    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distanceTick   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distanceTick,
                            double angle) {
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
          //  robot.setMotorDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            //moveCounts = (int)(distance * COUNTS_PER_INCH);
            moveCounts = (int)(distanceTick);
            newBackLeftTarget = robot.motor_left_back.getCurrentPosition() + moveCounts;
            newBackRightTarget = robot.motor_right_back.getCurrentPosition() + moveCounts;
            newFrontRightTarget = robot.motor_right_front.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = robot.motor_left_front.getCurrentPosition() + moveCounts;



            // Set Target and Turn On RUN_TO_POSITION
            robot.motor_left_back.setTargetPosition(newBackLeftTarget);
            robot.motor_right_back.setTargetPosition(newBackRightTarget);
            robot.motor_right_front.setTargetPosition(newFrontRightTarget);
            robot.motor_left_front.setTargetPosition(newFrontLeftTarget);


            robot.setMotorDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.setAllMotorDrivePower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    robot.motor_left_back.isBusy() &&
                    robot.motor_right_back.isBusy() &&
                    robot.motor_right_front.isBusy() &&
                    robot.motor_left_front.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distanceTick < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                robot.setLeftMotorDrivePower(leftSpeed);
                robot.setRightDrivePower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.motor_left_back.getCurrentPosition(),
                        robot.motor_right_back.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.setAllMotorDrivePower(0);

            // Turn off RUN_TO_POSITION
            robot.setMotorDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.setAllMotorDrivePower(0);
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setLeftMotorDrivePower(leftSpeed);
        robot.setRightDrivePower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
