/* Copyright (c) 2017 FIRST. All rights reserved
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

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class ParametricAutonomous {

    LinearOpMode shell;
    HardwareBucketBrigade hw = new HardwareBucketBrigade();
    ElapsedTime timer = new ElapsedTime();

    // constants that identify which team we are;
    public final static int RED = -1;
    public final static int BLUE = 1;
    public final static int FORWARD = 1;
    public final static int BACKWARD = -1;

    // contest-time parameters depending on team color and orientation
    int teamColor;
    public enum Orientation {BOX_FRONT, BOX_L_PATH};
    Orientation orientation;

    // constant parameters for tuning the robot
    public final double jewelSpeed = 0.5;
    public final int TIME_TO_KNOCK_OFF = 400; // ms
    public final int TIME_TO_DRIVE_TO_BASE = 4250;
    public final int HYSTERISIS_TIME = 500;
    public final int ARM_DROP_TIME = 1500;
    public final int CLAW_RAISE_TIME = 500;
    public final double ARM_RAISE_SPEED = 0.5;

    // Constructor
    ParametricAutonomous(LinearOpMode shell,
                         int teamColor,
                         Orientation orientation) {
        this.teamColor = teamColor;
        this.orientation = orientation;
        this.shell = shell;
    }

    public void driveDirection(double forwardOrBackward) {
        hw.leftDrive1.setPower(jewelSpeed*forwardOrBackward);
        hw.rightDrive1.setPower(jewelSpeed*forwardOrBackward);
        hw.leftDrive2.setPower(jewelSpeed*forwardOrBackward);
        hw.rightDrive2.setPower(jewelSpeed*forwardOrBackward);

    }


    public void runOpMode() {
        shell.telemetry.addData("Init", "Hello! It's me");
        hw.init(shell.hardwareMap);


        // Send telemetry message to signify robot waiting;
        shell.telemetry.addData("Status", "Ready to run");    //
        shell.telemetry.update();

        hw.clawServo1.setPower(0.7);
        hw.armMotor.setPower(ARM_RAISE_SPEED);
        shell.sleep(CLAW_RAISE_TIME);
        hw.armMotor.setPower(0.0);

        // Wait for the game to start (driver presses PLAY)
        shell.waitForStart();

        // drop the arm

        hw.colorServo.setPosition(0.8);




        shell.sleep(ARM_DROP_TIME);
        // check color.
        // do we need to get hysteresis?

        int colorSeen;
        if (hw.colorSensor.red()>hw.colorSensor.blue() ){
            shell.telemetry.addData("COLOR:", "More Red");
            colorSeen = RED;
        } else {
            shell.telemetry.addData("COLOR", "More Blue");
            colorSeen = BLUE;
        }

        //drive forward/backward
        int driveDirection = teamColor * colorSeen;
        driveDirection(driveDirection);

        shell.sleep(TIME_TO_KNOCK_OFF);
        driveDirection(0);

        //raise the arm
        hw.colorServo.setPosition(0.0);

        shell.sleep(ARM_DROP_TIME);
        //back to center
        driveDirection(-driveDirection);
        shell.sleep(TIME_TO_KNOCK_OFF);
        driveDirection(0);

        shell.sleep(HYSTERISIS_TIME);
        driveDirection(0.5);
        shell.sleep(TIME_TO_DRIVE_TO_BASE);
        driveDirection(0.0);
    }

}
