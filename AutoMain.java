
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by itay on 15/11/2017.
 * <p>
 * The common class for all the auto modes.
 * each auto mode should call function apolloRun.
 */
public abstract class AutoMain extends LinearOpMode {
    ColorSensor colorSensor;
    static private double FORWORD_SPEED = 0.4;
    private ElapsedTime runtime = new ElapsedTime();
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;
    HardwareConnection robot = new HardwareConnection();

    void connectionRun(boolean isBlue, boolean leftSide) {
        robot.init(hardwareMap);
        initVuforia();
        telemetry.addData("version: ", "4");
        telemetry.update();

        waitForStart();

        dropBall(isBlue);
        RelicRecoveryVuMark column = readPhoto();
        moveToCryptoBox(isBlue, leftSide);
        putCube(column);
        goToSafeZone();
    }

    // Balls task: Move the ball with the other color aside.
    private void dropBall(boolean isBlue) {
        boolean isBallBlue = false;
        // TODO(): itay.s.
        robot.ballHandLift.setPosition(0.3);
        sleep(1000);
        robot.ballHandLift.setPosition(0.6);
        sleep(1000);

        boolean foundColor = false;
        for (int i = 0; i < 3; i++) {
            if (colorSensor.blue() >= 100) {
                isBallBlue = true;
                foundColor = true;
                break;
            } else if (colorSensor.red() >= 100) {
                isBallBlue = false;
                foundColor = true;
                break;
            }

            robot.ballHandTurn.setPosition(-0.08);
            sleep(100);
        }

        if (foundColor) {
            if (isBlue == isBallBlue) {
                robot.ballHandTurn.setPosition(-0.7);
            } else {
                robot.ballHandTurn.setPosition(0.7);
            }
            sleep(1000);
        }

        robot.ballHandTurn.setPosition(0);
        sleep(1000);

        robot.ballHandLift.setPosition(0.3);
        sleep(100);
        robot.ballHandLift.setPosition(0);
        sleep(1000);
    }

    // Read photo and return the column to put the cube in.
    private RelicRecoveryVuMark readPhoto() {
        // TODO(): implement.

        relicTrackables.activate();
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                return vuMark;
            }
        }
        return RelicRecoveryVuMark.UNKNOWN;
    }

    // Move to crypto box
    private void moveToCryptoBox(boolean isBlue, boolean leftSide) {
        // TODO(): implement.
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

    // Park the robot
    private void goToSafeZone() {
        // TODO(): implement.
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
        int leftTargetBack = -5 * degree;
        int rightTargetBack = 5 * degree;
        int leftTargetFront = -5 * degree;
        int rightTargetFront = 5 * degree;
        robot.motor_left_back.setTargetPosition(leftTargetBack);
        robot.motor_left_front.setTargetPosition(leftTargetFront);
        robot.motor_right_front.setTargetPosition(rightTargetBack);
        robot.motor_right_back.setTargetPosition(rightTargetFront);
        robot.setAllMotorDrivePower(speed);
        while (opModeIsActive() &&
                (robot.motor_left_back.isBusy()) &&
                (robot.motor_right_front.isBusy()) &&
                (robot.motor_right_back.isBusy()) &&
                (robot.motor_left_front.isBusy())) {
            idle();
        }
        robot.setAllMotorDrivePower(0.0);
    }

}
