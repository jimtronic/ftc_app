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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="TestMotorCalibration")
public class TestMotorCalibration extends LinearOpMode {

    HardwareBucketBrigade hw = new HardwareBucketBrigade();
    ElapsedTime timer = new ElapsedTime();

    final static int RED = -1;
    final static int BLUE = 1;
    final static int FORWARD = 1;
    final static int BACKWARD = -1;
    int teamColor = RED;


    // distance to move block
    // number of positions for motor in full rotation
    // size of the wheel.

    // These together should tell us how many positions to count.

    // Next we need to determine good speed.

    // set calibration values:

    double leftDrive1Calibrate = 1.0;
    double leftDrive2Calibrate = 1.0;
    double rightDrive1Calibrate = 1.0;
    double rightDrive2Calibrate = 1.0;


    // Real.min(<val1>, 1.0);
    public void setMotorPowers(double amount) {
        hw.leftDrive1.setPower(amount*leftDrive1Calibrate);
        hw.leftDrive2.setPower(amount*leftDrive2Calibrate);
        hw.rightDrive1.setPower(amount*rightDrive1Calibrate);
        hw.rightDrive2.setPower(amount*rightDrive2Calibrate);
    }

    public void setEncoderRevolutions(int revs, double power) {
        hw.leftDrive1.setTargetPosition(revs);
        hw.leftDrive1.setPower(power);
        hw.leftDrive2.setTargetPosition(revs);
        hw.leftDrive2.setPower(power);
        hw.rightDrive1.setTargetPosition(revs);
        hw.rightDrive1.setPower(power);
        hw.rightDrive2.setTargetPosition(revs);
        hw.rightDrive2.setPower(power);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Init", "Hello! It's me");
        hw.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        setMotorPowers(0.25);

        telemetry.addData("trying speed test", "dummy");
        telemetry.update();
        while (true) {
            telemetry.addData("Left Motor", "Calibration " + leftDrive1Calibrate);
            telemetry.update();
            sleep(200);
            if (gamepad1.a) {
                break;
            }
            if (gamepad1.x) {
                leftDrive1Calibrate = leftDrive1Calibrate - 0.02;
            }
            if (gamepad1.b) {
                leftDrive1Calibrate = leftDrive1Calibrate + 0.02;
            }
            setMotorPowers(0.25);

        }

    }
}
