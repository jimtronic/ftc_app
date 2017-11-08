package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "", group = "Iterative Opmode")
//@Disabled
public class FourMotorTest extends OpMode {

    private DcMotor leftDrive1;
    private DcMotor leftDrive2;
    private DcMotor rightDrive1;
    private DcMotor rightDrive2;
    private DcMotor strafeWheel;
    private DcMotor armMotor;
    private Servo clawServo;
    private final double CLAW_SPEED = 0.01;
    private double clawPosition = 0.2;
    private ColorSensor colorSensor;

    @Override
    public void init() {
        telemetry.addData("Init", "Hello! It's me");


        leftDrive1 = hardwareMap.get(DcMotor.class, "leftFront");
        leftDrive2 = hardwareMap.get(DcMotor.class, "leftBack");
        rightDrive1 = hardwareMap.get(DcMotor.class, "rightFront");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rightBack");
        strafeWheel = hardwareMap.get(DcMotor.class, "strafeWheel");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);

        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);


        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);

        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        strafeWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeWheel.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {


        leftDrive1.setPower(gamepad1.left_stick_y/2);
        rightDrive1.setPower(gamepad1.right_stick_y/2);
        leftDrive2.setPower(gamepad1.left_stick_y/2);
        rightDrive2.setPower(gamepad1.right_stick_y/2);

        if (gamepad1.dpad_right){
            strafeWheel.setPower(1.0);
        } else if (gamepad1.dpad_left){
            strafeWheel.setPower(-1.0);
        } else {
            strafeWheel.setPower(0.0);
        }

        if (gamepad2.right_bumper){
            armMotor.setPower(1.0);
        } else if (gamepad2.left_bumper){
            armMotor.setPower(-1.0);
        } else {
            armMotor.setPower(0.0);
        }

        if (gamepad2.right_trigger > 0.8){
            clawPosition += CLAW_SPEED;
        } else if (gamepad2.left_trigger > 0.8){
            clawPosition -= CLAW_SPEED;
        }

        clawPosition = Range.clip(clawPosition, 0.2, 0.9);
        clawServo.setPosition(clawPosition);

        telemetry.addData("LEFT_FRONT", leftDrive1.getPower());
        telemetry.addData("RIGHT_FRONT", rightDrive1.getPower());
        telemetry.addData("LEFT_BACK", leftDrive2.getPower());
        telemetry.addData("RIGHT_FRONT", rightDrive2.getPower());
        telemetry.addData("STRAFE", strafeWheel.getPower());
        telemetry.addData("ARM_MOTOR", armMotor.getPower());
        telemetry.addData("THOMAS'S_LEVEL_OF_CRINGINESS", colorSensor.red());
        telemetry.addData("GREEN", colorSensor.green());
        telemetry.addData("BLUE", colorSensor.blue());
        telemetry.addData("ALPHA", colorSensor.alpha());

        if (colorSensor.red()>colorSensor.blue() ){
            telemetry.addData("COLOR:", "More Red");
        } else {
            telemetry.addData("COLOR", "More Blue");
        }



    }
}
