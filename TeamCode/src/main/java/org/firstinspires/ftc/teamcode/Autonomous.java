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

//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous")
public class Autonomous extends LinearOpMode {

    HardwareBucketBrigade hw = new HardwareBucketBrigade();
    ElapsedTime timer = new ElapsedTime();


    final static int RED = -1;
    final static int BLUE = 1;
    final static int FORWARD = 1;
    final static int BACKWARD = -1;
    enum Orientation {Straight, L_Shaped};

    // constants for autonomous mode
    final int TIME_TO_KNOCK_OFF = 400; // ms
    final int TIME_TO_DRIVE_TO_BASE = 4500;
    final int TIME_FOR_HYSTERISIS = 500;
    final int TIME_FOR_ARM_DROP = 1500;
    final int TIME_FOR_CLAW_RAISE = 500;
    final double SPEED_TO_KNOCK_OFF = 0.7;
    final double SPEED_TO_DRIVE = 0.5;
    final double SPEED_FOR_ARM_RAISE = 0.5;

    // parameters for autonomous mode
    int teamColor = RED;
    Orientation orientation = Orientation.Straight;
    double jewelSpeed = 0.5;

    public void driveDirection(double forwardOrBackward) {
        hw.leftDrive1.setPower(jewelSpeed*forwardOrBackward);
        hw.leftDrive2.setPower(jewelSpeed*forwardOrBackward);
        hw.rightDrive1.setPower(jewelSpeed*forwardOrBackward);
        hw.rightDrive2.setPower(jewelSpeed*forwardOrBackward);
    }

    public void strafeDirection(double forwardOrBackward) {
        hw.strafeWheel.setPower(jewelSpeed*forwardOrBackward);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Init", "Hello! It's me");
        hw.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        hw.clawServo1.setPower(0.7);
        hw.armMotor.setPower(SPEED_FOR_ARM_RAISE);
        sleep(TIME_FOR_CLAW_RAISE);
        hw.armMotor.setPower(0.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // drop the arm

        hw.colorServo.setPosition(0.8);


        sleep(TIME_FOR_ARM_DROP);
        // check color.
        // do we need to get hysteresis?

        int colorSeen;
        if (hw.colorSensor.red()>hw.colorSensor.blue() ){
            telemetry.addData("COLOR:", "More Red");
            colorSeen = RED;
        } else {
            telemetry.addData("COLOR", "More Blue");
            colorSeen = BLUE;
        }

        //drive forward/backward
        double driveDirection = teamColor * colorSeen * SPEED_TO_KNOCK_OFF;
        driveDirection(driveDirection);

        sleep(TIME_TO_KNOCK_OFF);
        driveDirection(0);

        //raise the arm
        hw.colorServo.setPosition(0.0);

        sleep(TIME_FOR_ARM_DROP);
        //back to center
        driveDirection(-driveDirection);
        sleep(TIME_TO_KNOCK_OFF);
        driveDirection(0);

        sleep(TIME_FOR_HYSTERISIS);

        // The direction to drive is based on orientation
        // When on the red platform, the robot orientation is "backwards"
        // of the crypto box.  When on the blue platform, the robot orientation
        // is forward to the crypto box.

        driveDirection(-1.0*teamColor*SPEED_TO_DRIVE);
        sleep(TIME_TO_DRIVE_TO_BASE);
        driveDirection(0.0);

        if (orientation == Orientation.Straight) {
            strafeDirection(-1.0*teamColor);
            sleep(400);
            strafeDirection(0.0);
        }
    }




}
