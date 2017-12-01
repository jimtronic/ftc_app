package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

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


        hw.leftDrive1.setPower(gamepad1.left_stick_y*.75);
        hw.rightDrive1.setPower(gamepad1.right_stick_y*.75);
        hw.leftDrive2.setPower(gamepad1.left_stick_y*.75);
        hw.rightDrive2.setPower(gamepad1.right_stick_y*.75);

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

        if (gamepad2.right_trigger > 0.8){
            hw.clawPower = hw.CLAW_SPEED;
        } else if (gamepad2.left_trigger > 0.8){
            hw.clawPower = -hw.CLAW_SPEED;
        } else {
            hw.clawPower = 0;
        }

        hw.clawPower = Range.clip(hw.clawPower, -1, 1);
        hw.clawServo1.setPower(hw.clawPower);

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
