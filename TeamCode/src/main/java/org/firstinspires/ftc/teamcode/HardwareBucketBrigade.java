package org.firstinspires.ftc.teamcode;

/**
 * Created by phinn on 11/18/2017.
 */
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
    Physical decsription and configuration of robot
 */
public class HardwareBucketBrigade {

    public DcMotor leftDriveFront;
    public DcMotor leftDriveBack;
    public DcMotor rightDriveFront;
    public DcMotor rightDriveBack;
    public DcMotor strafeWheel;
    public DcMotor armMotor;
    public CRServo clawServo1;
    public Servo clawServo2;
    public Servo colorServo;
    public double CLAW_SPEED = 1;
    public double clawPower = 1;


    public ColorSensor colorSensor;
    public void init(HardwareMap hardwareMap) {

        leftDriveFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftDriveBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightDriveBack = hardwareMap.get(DcMotor.class, "rightBack");
        strafeWheel = hardwareMap.get(DcMotor.class, "strafeWheel");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        clawServo1 = hardwareMap.crservo.get("clawServo1");
        colorServo = hardwareMap.get(Servo.class, "colorServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");


        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);

        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);


        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);

        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        strafeWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeWheel.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        ///colorServo.scaleRange(this.COLOR_SERVO_MIN_POSITION, this.COLOR_SERVO_MAX_POSITION);
    }

    public static final double LEFT_FRONT_CALIBRATION = 1.0;
    public static final double RIGHT_FRONT_CALIBRATION = 1.0;
    public static final double LEFT_BACK_CALIBRATION = 1.0;
    public static final double RIGHT_BACK_CALIBRATION = 1.0;

    public static final double MAX_RATIO = 0.75;

    void setMotorSpeed(double leftStick, double rightStick) {
        leftDriveFront.setPower(leftStick*MAX_RATIO*LEFT_FRONT_CALIBRATION);
        rightDriveFront.setPower(rightStick*MAX_RATIO*RIGHT_FRONT_CALIBRATION);
        leftDriveBack.setPower(leftStick*MAX_RATIO*LEFT_BACK_CALIBRATION);
        rightDriveBack.setPower(rightStick*MAX_RATIO*RIGHT_BACK_CALIBRATION);
    }
}
