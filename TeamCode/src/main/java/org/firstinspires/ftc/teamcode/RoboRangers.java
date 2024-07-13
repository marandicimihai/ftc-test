package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config

@TeleOp(name = "MAIN", group = "Robot")
public class RoboRangers extends OpMode
{
    private PIDController controller;

    public static double p = 0.01, i = 0.00001, d = 0.00005;
    public static double f = 0.01;

    private final double ticks_in_degree = 3360d / 360d;

    public static int axleTarget;
    private double axlePowerMultiplier = 1;
    private int extensionTarget;
    private double extensionVelocity = 2000;
    private double jointTarget;
    private String lastPos = "cross";

    private DcMotorEx wheel1;
    private DcMotorEx wheel2;
    private DcMotorEx wheel3;
    private DcMotorEx wheel4;
    private DcMotorEx extension;
    private DcMotorEx extensionAxle;
    public DcMotor arm = null;
    public Servo drone;
    private IMU imu;
    private double targetYaw = 0;
    private boolean ninety = false;
    public boolean walkMode = false;
    public boolean slowerMode = false;
    public boolean board = false;
    private boolean droneToggle = false;
    private boolean droneGate = false;
    double clawOffset = 0;
    ElapsedTime myElapsedTime = new ElapsedTime();
    private Servo FingerL;
    private Servo FingerR;
    private Servo JointL;
    private Servo JointR;
    private Thread thread;

    @Override
    public void init() {
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel3 = hardwareMap.get(DcMotorEx.class, "wheel3");
        wheel4 = hardwareMap.get(DcMotorEx.class, "wheel4");
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        extensionAxle = hardwareMap.get(DcMotorEx.class, "extensionAxle");
        imu = hardwareMap.get(IMU.class, "imu");
        drone = hardwareMap.get(Servo.class, "drone");
        //   imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(0);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionAxle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionAxle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myElapsedTime.reset();

        wheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        this part is for the second type holonomic drive;

//        wheel1.setDirection(DcMotor.Direction.FORWARD);
//        wheel3.setDirection(DcMotor.Direction.REVERSE);
//        wheel2.setDirection(DcMotor.Direction.REVERSE);
//        wheel4.setDirection(DcMotor.Direction.FORWARD);

        FingerL = hardwareMap.servo.get("FingerL");
        FingerR = hardwareMap.servo.get("FingerR");
        JointL = hardwareMap.servo.get("JointL");
        JointR = hardwareMap.servo.get("JointR");

        axleTarget = 0;
        extensionTarget = 0;
        jointTarget = 0;

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    //region Driver 1

    void waitt(double time)
    {
        double t = myElapsedTime.milliseconds();
        double t2 = myElapsedTime.milliseconds();
        while(t2 <= t+time)
        {
            t2 = myElapsedTime.milliseconds();
        }
    }
    private void snail() {
        if (gamepad1.ps) {
            slowerMode = !slowerMode;
        }
    }

    private void walk() {
        double length;
        double angle;
        double RightStickSim = 0.5;
        double motor0_velocity = 0;
        double motor1_velocity = 0;
        double motor2_velocity = 0;
        double motor3_velocity = 0;
        length = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        angle = Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x) / Math.PI * 180;
        angle = angle * -1;
        RightStickSim = gamepad1.right_stick_x;
        if(gamepad1.right_bumper)
        {
            RightStickSim*=0.5;
        }
        length/=2.7;
        if (ninety) {
            angle += 180;
            angle %= 360;
            length *= 2;
            RightStickSim *= 1;
        } else if (slowerMode) {
            length *= 0.25;
            RightStickSim *= 0.2;
        }
        if (board) {

            length *= 0.2;
            RightStickSim *= 0.1;
            gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else {
            gamepad1.stopRumble();
        }
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        angle = angle - yaw;
        if (gamepad1.left_stick_y > 0) {
            if (gamepad1.left_stick_x < 0) {
                angle = angle + 180;
                telemetry.addData("quadrant", 3);

            } else {
                angle = angle + 360;
                telemetry.addData("quadrant", 4);
            }
        } else if (gamepad1.left_stick_x < 0) {
            angle = angle + 180;
            telemetry.addData("quadrant", 2);
        }
        /*
        if(eighty){
          angle+=180;
          length*=0.6;
          RightStickSim*=0.6;
        }
        */
        if (length == 0) {
            angle = 0;
        }
        //length*=1.25;
        double motor0_ratio = Math.sin((angle - 135) / 180 * Math.PI) * -1;
        double motor1_ratio = Math.sin((angle - 45) / 180 * Math.PI) * -1;
        double motor2_ratio = Math.sin((angle - 315) / 180 * Math.PI) * -1;
        double motor3_ratio = Math.sin((angle - 225) / 180 * Math.PI) * -1;
        double motormax_ratio = Math.max(Math.max(motor0_ratio, motor1_ratio), Math.max(motor2_ratio, motor3_ratio));
        if (motor0_ratio != 0) {
            motor0_velocity = Math.abs(motor0_ratio / motormax_ratio) * length * motor0_ratio / Math.abs(motor0_ratio);
        }
        if (motor1_ratio != 0) {
            motor1_velocity = Math.abs(motor1_ratio / motormax_ratio) * length * motor1_ratio / Math.abs(motor1_ratio);
        }
        if (motor2_ratio != 0) {
            motor2_velocity = Math.abs(motor2_ratio / motormax_ratio) * length * motor2_ratio / Math.abs(motor2_ratio);
        }
        if (motor3_ratio != 0) {
            motor3_velocity = Math.abs(motor3_ratio / motormax_ratio) * length * motor3_ratio / Math.abs(motor3_ratio);
        }
//        if (RightStickSim != 0) {
//            targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        }
//        if ((gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
//            RightStickSim = (targetYaw - yaw) * 0.1;
//        }

        double maxSpeed = 3000;

        wheel1.setVelocity((motor0_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        wheel2.setVelocity((motor1_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        wheel3.setVelocity((motor2_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        wheel4.setVelocity((motor3_velocity * 1 + RightStickSim * 0.6) * maxSpeed);

//        double pwrMultiplier = 0.5;
//        wheel1.setPower(motor0_velocity + RightStickSim * pwrMultiplier);
//        wheel2.setPower(motor1_velocity + RightStickSim * pwrMultiplier);
//        wheel3.setPower(motor2_velocity + RightStickSim * pwrMultiplier);
//        wheel4.setPower(motor3_velocity + RightStickSim * pwrMultiplier);

        //telemetry.update();
    }

    private void walkDir(double length, double angle, double RightStickSim, double time) {
        double motor0_velocity = 0;
        double motor1_velocity = 0;
        double motor2_velocity = 0;
        double motor3_velocity = 0;
        angle = angle * -1;
        double t = myElapsedTime.milliseconds();
        length/=2.7;
        if (ninety) {
            angle += 180;
            angle %= 360;
            length *= 2;
            RightStickSim *= 1;
        } else if (slowerMode) {
            length *= 0.25;
            RightStickSim *= 0.2;
        }
        if (board) {

            length *= 0.2;
            RightStickSim *= 0.1;
            gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else {
            gamepad1.stopRumble();
        }
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        angle = angle - yaw;
        if (gamepad1.left_stick_y > 0) {
            if (gamepad1.left_stick_x < 0) {
                angle = angle + 180;
                telemetry.addData("quadrant", 3);

            } else {
                angle = angle + 360;
                telemetry.addData("quadrant", 4);
            }
        } else if (gamepad1.left_stick_x < 0) {
            angle = angle + 180;
            telemetry.addData("quadrant", 2);
        }
        /*
        if(eighty){
          angle+=180;
          length*=0.6;
          RightStickSim*=0.6;
        }
        */
        if (length == 0) {
            angle = 0;
        }
        //length*=1.25;
        double motor0_ratio = Math.sin((angle - 135) / 180 * Math.PI) * -1;
        double motor1_ratio = Math.sin((angle - 45) / 180 * Math.PI) * -1;
        double motor2_ratio = Math.sin((angle - 315) / 180 * Math.PI) * -1;
        double motor3_ratio = Math.sin((angle - 225) / 180 * Math.PI) * -1;
        double motormax_ratio = Math.max(Math.max(motor0_ratio, motor1_ratio), Math.max(motor2_ratio, motor3_ratio));
        if (motor0_ratio != 0) {
            motor0_velocity = Math.abs(motor0_ratio / motormax_ratio) * length * motor0_ratio / Math.abs(motor0_ratio);
        }
        if (motor1_ratio != 0) {
            motor1_velocity = Math.abs(motor1_ratio / motormax_ratio) * length * motor1_ratio / Math.abs(motor1_ratio);
        }
        if (motor2_ratio != 0) {
            motor2_velocity = Math.abs(motor2_ratio / motormax_ratio) * length * motor2_ratio / Math.abs(motor2_ratio);
        }
        if (motor3_ratio != 0) {
            motor3_velocity = Math.abs(motor3_ratio / motormax_ratio) * length * motor3_ratio / Math.abs(motor3_ratio);
        }
//        if (RightStickSim != 0) {
//            targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        }
//        if ((gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
//            RightStickSim = (targetYaw - yaw) * 0.1;
//        }

        double maxSpeed = 3000;

        wheel1.setVelocity((motor0_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        wheel2.setVelocity((motor1_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        wheel3.setVelocity((motor2_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        wheel4.setVelocity((motor3_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        waitt(time);
//        double pwrMultiplier = 0.5;
//        wheel1.setPower(motor0_velocity + RightStickSim * pwrMultiplier);
//        wheel2.setPower(motor1_velocity + RightStickSim * pwrMultiplier);
//        wheel3.setPower(motor2_velocity + RightStickSim * pwrMultiplier);
//        wheel4.setPower(motor3_velocity + RightStickSim * pwrMultiplier);
        //telemetry.update()
    }


    private void droneToggle()
    {
        if (gamepad1.dpad_down && !droneGate)
        {
            if (!droneToggle)
            {
                drone.setPosition(1);
                droneToggle = true;
            }
            else
            {
                drone.setPosition(0);
                droneToggle = false;
            }
            droneGate = true;
        }
        else if (!gamepad1.dpad_down)
        {
            droneGate = false;
        }
    }

    //endregion

    //region Driver 2

    private void armPositions()
    {
        //axle 0 - -1550
        //extension 0 - 850 * 1.66
        //joint 0 - 1

        //IDLE POSITION
        if (gamepad2.cross) {
            axleTarget = 0;
            axlePowerMultiplier = 0.1;
            extensionTarget = 0;
            extensionVelocity = 6000;
            jointTarget = 0;
            lastPos = "cross";
        }
        //EXTEND FOR INTAKE POSITION
        else if (gamepad2.square && lastPos.equals("cross")) {
            axleTarget = -80;
            axlePowerMultiplier = 1;
            extensionTarget = (int)(850d * 1.66d);
            extensionVelocity = 3000;
            jointTarget = 0.6;
            lastPos = "square";
        }
        //PLACE LOW
        else if (gamepad2.circle) {
            axleTarget = -1550;
            axlePowerMultiplier = 0.1;
            extensionTarget = (int)(850d * 1.66d);
            extensionVelocity = 800;
            jointTarget = 0.7;
            lastPos = "circle";
        }
        //PLACE HIGH
        else if (gamepad2.triangle) {
            axleTarget = -1300;
            axlePowerMultiplier = 0.1;
            extensionTarget = (int)(850d * 1.66d);
            extensionVelocity = 800;
            jointTarget = 0.6;
            lastPos = "triangle";
        }
        //HOOK
        else if (gamepad2.dpad_down) {
            axleTarget = -1100;
            axlePowerMultiplier = 0.1;
            extensionTarget = (int)(700d * 1.66d);
            extensionVelocity = 800;
            lastPos = "dpad_down";
        }
        else if (gamepad2.dpad_up)
        {
            extensionTarget = 0;
            extensionVelocity = 1600;
            jointTarget = 0;
            lastPos = "dpad_up";
        }
    }

    private void extensionPosition() {
        extension.setTargetPosition(extensionTarget);
        extension.setVelocity(extensionVelocity);
    }

    private void axlePosition() {
        controller.setPID(p, i, d);
        int armPos = extensionAxle.getCurrentPosition();
        double pid = controller.calculate(armPos, axleTarget);
        double ff = Math.cos(armPos / ticks_in_degree) * f;

        double power = pid + ff;

        extensionAxle.setPower(power * axlePowerMultiplier);
    }

    private void clawFingerLoop() {
        if (gamepad2.right_bumper) {
            FingerR.setPosition(0);
        } else {
            FingerR.setPosition(0.3);
        }
        if (gamepad2.left_bumper) {
            FingerL.setPosition(1);
        } else {
            FingerL.setPosition(0.7);
        }
    }

    private void jointPosition() {
        JointL.setPosition(jointTarget);
        //JointR.setPosition(1 - jointTarget);
    }

    //endregion

    void stop_m()
    {
        wheel1.setVelocity(0);
        wheel2.setVelocity(0);
        wheel3.setVelocity(0);
        wheel4.setVelocity(0);
    }

    @Override
    public void init_loop() {
        init();
        imu.resetYaw();
    }

    @Override
    public void start() {
//        //walkDir(0.75, 270, 0, 1000);
//        FingerL.setPosition(1);
//        waitt(500);
//        walkDir(0.25, 270, 0, 2800);
//        stop_m();
//        FingerR.setPosition(0.4);
//        double t = myElapsedTime.milliseconds();
//        double t2 = myElapsedTime.milliseconds();
//        axleTarget = -500;
//        axlePowerMultiplier = 0.1;
//        jointTarget = 0.1;
//        while(t2 < t+2000)
//        {
//            axlePosition();
//            jointPosition();
//            t2 = myElapsedTime.milliseconds();
//        }
//        walkDir(0.0, 0, 0.2, 1400);
//        stop_m();
//        waitt(1000);
//        walkDir(0.25, 180, 0, 1000);
//        stop_m();
//        waitt(1000);
//        walkDir(0.25, 90, 0, 3800);
//        stop_m();
//        t = myElapsedTime.milliseconds();
//        t2 = myElapsedTime.milliseconds();
//        axleTarget = -1550;
//        axlePowerMultiplier = 0.1;
//        jointTarget = 0.7;
//        while(t2 < t+2000)
//        {
//            axlePosition();
//            jointPosition();
//            t2 = myElapsedTime.milliseconds();
//        }
//        FingerL.setPosition(0.5);
//        waitt(1000);
//        walkDir(0.25, -90, 0, 1500);
//        stop_m();
//        waitt(1000);
//        t = myElapsedTime.milliseconds();
//        t2 = myElapsedTime.milliseconds();
//        axleTarget = 0;
//        axlePowerMultiplier = 0.1;
//        jointTarget = 0.5;
//        while(t2 < t+2000)
//        {
//            axlePosition();
//            jointPosition();
//            t2 = myElapsedTime.milliseconds();
//        }
//        walkDir(-0.25, 180, 0, 3100);
//        stop_m();
//        waitt(1000);
//        walkDir(0.25, 90, 0, 3000);
//        stop_m();
//        waitt(1000);
//
//
//
    }

    public void loop() {
        //driver 1
        walk();
        snail();
        //walkV2();
        droneToggle();
        gamepad1.setLedColor(0, 0, 1, 100);

        //driver 2
        armPositions();
        clawFingerLoop();
        extensionPosition();
        axlePosition();
        jointPosition();

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
/*
public  class Roborangers extends LinearOpMode
{
    private PIDController controller;

    public static double p = 0.01, i = 0.00001, d = 0.00005;
    public static double f = 0.01;

    private final double ticks_in_degree = 3360d / 360d;

    public static int axleTarget;
    private double axlePowerMultiplier = 1;
    private int extensionTarget;
    private double extensionVelocity = 2000;
    private double jointTarget;
    private String lastPos = "cross";

    private DcMotorEx wheel1;
    private DcMotorEx wheel2;
    private DcMotorEx wheel3;
    private DcMotorEx wheel4;
    private DcMotorEx extension;
    private DcMotorEx extensionAxle;
    public DcMotor arm = null;
    public Servo drone;
    private IMU imu;
    private double targetYaw = 0;
    private boolean ninety = false;
    public boolean walkMode = false;
    public boolean slowerMode = false;
    public boolean board = false;
    private boolean droneToggle = false;
    private boolean droneGate = false;
    double clawOffset = 0;
    ElapsedTime myElapsedTime = new ElapsedTime();
    private Servo FingerL;
    private Servo FingerR;
    private Servo JointL;
    private Servo JointR;
    private Thread thread;

    @Override
    public void init() {
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel3 = hardwareMap.get(DcMotorEx.class, "wheel3");
        wheel4 = hardwareMap.get(DcMotorEx.class, "wheel4");
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        extensionAxle = hardwareMap.get(DcMotorEx.class, "extensionAxle");
        imu = hardwareMap.get(IMU.class, "imu");
        drone = hardwareMap.get(Servo.class, "drone");
        //   imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(0);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionAxle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionAxle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myElapsedTime.reset();

        wheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        this part is for the second type holonomic drive;

//        wheel1.setDirection(DcMotor.Direction.FORWARD);
//        wheel3.setDirection(DcMotor.Direction.REVERSE);
//        wheel2.setDirection(DcMotor.Direction.REVERSE);
//        wheel4.setDirection(DcMotor.Direction.FORWARD);

        FingerL = hardwareMap.servo.get("FingerL");
        FingerR = hardwareMap.servo.get("FingerR");
        JointL = hardwareMap.servo.get("JointL");
        JointR = hardwareMap.servo.get("JointR");

        axleTarget = 0;
        extensionTarget = 0;
        jointTarget = 0;

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    //region Driver 1

    void waitt(double time)
    {
        double t = myElapsedTime.milliseconds();
        double t2 = myElapsedTime.milliseconds();
        while(t2 <= t+time)
        {
            t2 = myElapsedTime.milliseconds();
        }
    }

    private void walkDir(double length, double angle, double RightStickSim, double time) {
        double motor0_velocity = 0;
        double motor1_velocity = 0;
        double motor2_velocity = 0;
        double motor3_velocity = 0;
        angle = angle * -1;
        double t = myElapsedTime.milliseconds();
        length/=2.7;
        if (ninety) {
            angle += 180;
            angle %= 360;
            length *= 2;
            RightStickSim *= 1;
        } else if (slowerMode) {
            length *= 0.25;
            RightStickSim *= 0.2;
        }
        if (board) {

            length *= 0.2;
            RightStickSim *= 0.1;
            gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else {
            gamepad1.stopRumble();
        }
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        angle = angle - yaw;
        if (gamepad1.left_stick_y > 0) {
            if (gamepad1.left_stick_x < 0) {
                angle = angle + 180;
                telemetry.addData("quadrant", 3);

            } else {
                angle = angle + 360;
                telemetry.addData("quadrant", 4);
            }
        } else if (gamepad1.left_stick_x < 0) {
            angle = angle + 180;
            telemetry.addData("quadrant", 2);
        }
        if (length == 0) {
            angle = 0;
        }
        //length*=1.25;
        double motor0_ratio = Math.sin((angle - 135) / 180 * Math.PI) * -1;
        double motor1_ratio = Math.sin((angle - 45) / 180 * Math.PI) * -1;
        double motor2_ratio = Math.sin((angle - 315) / 180 * Math.PI) * -1;
        double motor3_ratio = Math.sin((angle - 225) / 180 * Math.PI) * -1;
        double motormax_ratio = Math.max(Math.max(motor0_ratio, motor1_ratio), Math.max(motor2_ratio, motor3_ratio));
        if (motor0_ratio != 0) {
            motor0_velocity = Math.abs(motor0_ratio / motormax_ratio) * length * motor0_ratio / Math.abs(motor0_ratio);
        }
        if (motor1_ratio != 0) {
            motor1_velocity = Math.abs(motor1_ratio / motormax_ratio) * length * motor1_ratio / Math.abs(motor1_ratio);
        }
        if (motor2_ratio != 0) {
            motor2_velocity = Math.abs(motor2_ratio / motormax_ratio) * length * motor2_ratio / Math.abs(motor2_ratio);
        }
        if (motor3_ratio != 0) {
            motor3_velocity = Math.abs(motor3_ratio / motormax_ratio) * length * motor3_ratio / Math.abs(motor3_ratio);
        }

        double maxSpeed = 3000;

        wheel1.setVelocity((motor0_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        wheel2.setVelocity((motor1_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        wheel3.setVelocity((motor2_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        wheel4.setVelocity((motor3_velocity * 1 + RightStickSim * 0.6) * maxSpeed);
        waitt(time);
    }

    private void axlePosition() {
        controller.setPID(p, i, d);
        int armPos = extensionAxle.getCurrentPosition();
        double pid = controller.calculate(armPos, axleTarget);
        double ff = Math.cos(armPos / ticks_in_degree) * f;

        double power = pid + ff;

        extensionAxle.setPower(power * axlePowerMultiplier);
    }

    private void jointPosition() {
        JointL.setPosition(jointTarget);
        //JointR.setPosition(1 - jointTarget);
    }


    void stop_m()
    {
        wheel1.setVelocity(0);
        wheel2.setVelocity(0);
        wheel3.setVelocity(0);
        wheel4.setVelocity(0);
    }

    @Override
    public void init_loop() {
        init();
        imu.resetYaw();
    }

    @Override
    public void start() {
        //walkDir(0.75, 270, 0, 1000);
        FingerL.setPosition(1);
        waitt(500);
        walkDir(0.25, 270, 0, 2800);
        stop_m();
        FingerR.setPosition(0.4);
        double t = myElapsedTime.milliseconds();
        double t2 = myElapsedTime.milliseconds();
        axleTarget = -500;
        axlePowerMultiplier = 0.1;
        jointTarget = 0.1;
        while(t2 < t+2000)
        {
            axlePosition();
            jointPosition();
            t2 = myElapsedTime.milliseconds();
        }
        walkDir(0.0, 0, 0.2, 1400);
        stop_m();
        waitt(1000);
        walkDir(0.25, 180, 0, 1000);
        stop_m();
        waitt(1000);
        walkDir(0.25, 90, 0, 3800);
        stop_m();
        t = myElapsedTime.milliseconds();
        t2 = myElapsedTime.milliseconds();
        axleTarget = -1550;
        axlePowerMultiplier = 0.1;
        jointTarget = 0.7;
        while(t2 < t+2000)
        {
            axlePosition();
            jointPosition();
            t2 = myElapsedTime.milliseconds();
        }
        FingerL.setPosition(0.5);
        waitt(1000);
        walkDir(0.25, -90, 0, 1500);
        stop_m();
        waitt(1000);
        t = myElapsedTime.milliseconds();
        t2 = myElapsedTime.milliseconds();
        axleTarget = 0;
        axlePowerMultiplier = 0.1;
        jointTarget = 0.5;
        while(t2 < t+2000)
        {
            axlePosition();
            jointPosition();
            t2 = myElapsedTime.milliseconds();
        }
        walkDir(-0.25, 180, 0, 3100);
        stop_m();
        waitt(1000);
        walkDir(0.25, 90, 0, 3000);
        stop_m();
        waitt(1000);



    }
}
 */