package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: FourMotorTest", group="Iterative Opmode")
//@Disabled
public class FourMotorTest extends OpMode {

    private DcMotor leftdrive1;
    private DcMotor rightdrive1;
    private DcMotor leftdrive2;
    private DcMotor rightdrive2;

    @Override
    public void init() {
        telemetry.addData("Init" , "Hello! It's me");

        leftdrive1 = hardwareMap.get(DcMotor.class, "testmotor");
       /* rightdrive1 = hardwareMap.get(DcMotor.class, "rightdrivefront");
        leftdrive2 = hardwareMap.get(DcMotor.class, "leftdriveback");
        rightdrive2 = hardwareMap.get(DcMotor.class, "rightdriveback"); */

        leftdrive1.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void start() {
        super.start();
        leftdrive1.setPower(0.75);
    }

    @Override
    public void loop() {
//        double power;
//        double drive = gamepad1.right_stick_y;
//
//        power = Range.clip(drive, -1.0, 1.0);
    }
}
