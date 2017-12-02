package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareBucketBrigade;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Drive_Forward")
public class Drive_Forward extends LinearOpMode {


    HardwareBucketBrigade hw = new HardwareBucketBrigade();
    ElapsedTime timer = new ElapsedTime();


    final static int RED = -1;
    final static int BLUE = 1;
    final static int FORWARD = 1;
    final static int BACKWARD = -1;
    int teamColor = RED;

    double jewelSpeed = 0.5;
    final int TIME_TO_KNOCK_OFF = 3000; // ms


    public void driveDirection(int forwardOrBackward) {
        hw.leftDrive1.setPower(jewelSpeed * forwardOrBackward);
        hw.rightDrive1.setPower(jewelSpeed * forwardOrBackward);
        hw.leftDrive2.setPower(jewelSpeed * forwardOrBackward);
        hw.rightDrive2.setPower(jewelSpeed * forwardOrBackward);

    }

    int colorSeen;
    @Override
    public void runOpMode() {

        int driveDirection = teamColor * colorSeen;
        driveDirection(driveDirection);




    }


}