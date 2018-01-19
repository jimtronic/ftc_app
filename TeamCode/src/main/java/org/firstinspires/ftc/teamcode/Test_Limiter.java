package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.TextTracker;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "", group = "Iterative Opmode")
//@Disabled
public class Test_Limiter extends OpMode {


    HardwareBucketBrigade hw = new HardwareBucketBrigade();

    @Override
    public void init() {
        telemetry.addData("Init", "Hello! It's me");
        hw.init(hardwareMap);


    }

    enum Direction {Up, Down, Level};

    @Override
    public void loop() {


        hw.armMotor.getCurrentPosition();
        if (gamepad2.right_bumper){
            hw.armMotor.setPower(-0.75);
        } else if (gamepad2.left_bumper){
            hw.armMotor.setPower(0.75);
        } else {
            hw.armMotor.setPower(0.0);
        }
        int revolutions = 0;

        int motorPosition = hw.armMotor.getCurrentPosition();
        int motorTarget = hw.armMotor.getTargetPosition();
        double motorPower = hw.armMotor.getPower();

        Direction motorDirection;

        if (motorPower < 0) motorDirection = Direction.Up;
        else if (motorPower > 0) motorDirection = Direction.Down;
        else motorDirection = Direction.Level;

        telemetry.addData("CURRENT_POSITION", motorPosition);

        telemetry.addData("TARGET_POSITION", motorTarget);

        telemetry.addData("MOTOR_DIRECTION", hw.armMotor.getPower());

        telemetry.addData("DIRECTION", motorDirection);

    }
}
