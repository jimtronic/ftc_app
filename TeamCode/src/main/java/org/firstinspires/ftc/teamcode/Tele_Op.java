package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "", group = "Iterative Opmode")
//@Disabled
public class Tele_Op extends OpMode {


    HardwareBucketBrigade hw = new HardwareBucketBrigade();

    @Override
    public void init() {
        telemetry.addData("Init", "Hello! It's me");
        hw.init(hardwareMap);


    }

    @Override
    public void loop() {


        hw.leftDrive1.setPower(gamepad1.left_stick_y);
        hw.rightDrive1.setPower(gamepad1.right_stick_y);
        hw.leftDrive2.setPower(gamepad1.left_stick_y);
        hw.rightDrive2.setPower(gamepad1.right_stick_y);

        if (gamepad1.dpad_right){
            hw.strafeWheel.setPower(1.0);
        } else if (gamepad1.dpad_left){
            hw.strafeWheel.setPower(-1.0);
        } else {
            hw.strafeWheel.setPower(0.0);
        }

        if (gamepad2.right_bumper){
            hw.armMotor.setPower(1.0);
        } else if (gamepad2.left_bumper){
            hw.armMotor.setPower(-1.0);
        } else {
            hw.armMotor.setPower(0.0);
        }


        telemetry.addData("LEFT_FRONT", hw.leftDrive1.getPower());

        telemetry.addData("RIGHT_FRONT", hw.rightDrive1.getPower());

        telemetry.addData("LEFT_BACK", hw.leftDrive2.getPower());

        telemetry.addData("RIGHT_FRONT", hw.rightDrive2.getPower());

        telemetry.addData("STRAFE", hw.strafeWheel.getPower());

        telemetry.addData("ARM_MOTOR", hw.armMotor.getPower());

        telemetry.addData("RED", hw.colorSensor.red());

        telemetry.addData("GREEN", hw.colorSensor.green());

        telemetry.addData("BLUE", hw.colorSensor.blue());

        telemetry.addData("ALPHA", hw.colorSensor.alpha());


        if (hw.colorSensor.red()>hw.colorSensor.blue() ){
            telemetry.addData("COLOR:", "More Red");
        } else {
            telemetry.addData("COLOR", "More Blue");
        }



    }
}
