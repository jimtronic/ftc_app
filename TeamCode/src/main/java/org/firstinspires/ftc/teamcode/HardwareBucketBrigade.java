package org.firstinspires.ftc.teamcode;

/**
 * Created by phinn on 11/18/2017.
 */
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
    Physical decsription and configuration of robot
 */
public class HardwareBucketBrigade {

    public DcMotor leftDrive1;
    public DcMotor leftDrive2;
    public DcMotor rightDrive1;
    public DcMotor rightDrive2;
    public DcMotor strafeWheel;
    public DcMotor armMotor;
    public Servo clawServo1;
    public Servo clawServo2;
    public Servo colorServo;

    // MWW NOTE: these are guesses, modify to fit actual robot.
    public final double COLOR_SERVO_MIN_POSITION = 0.5;
    public final double COLOR_SERVO_MAX_POSITION = 0.95;

    public ColorSensor colorSensor;
    public void init(HardwareMap hardwareMap) {

        leftDrive1 = hardwareMap.get(DcMotor.class, "leftFront");
        leftDrive2 = hardwareMap.get(DcMotor.class, "leftBack");
        rightDrive1 = hardwareMap.get(DcMotor.class, "rightFront");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rightBack");
        strafeWheel = hardwareMap.get(DcMotor.class, "strafeWheel");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        // clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
        // clawServo2 = hardwareMap.get(Servo.class, "clawServo2");
        colorServo = hardwareMap.get(Servo.class, "colorServo");
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

        colorServo.scaleRange(this.COLOR_SERVO_MIN_POSITION, this.COLOR_SERVO_MAX_POSITION);
    }
}
