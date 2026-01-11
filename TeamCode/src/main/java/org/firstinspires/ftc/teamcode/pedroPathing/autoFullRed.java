package org.firstinspires.ftc.teamcode.pedroPathing;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Configurable
@Autonomous(name = "autoFullRed", group = "Autonomous")
public class autoFullRed extends LinearOpMode {


    // Hardware
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo kicker;


    private DcMotor flywheel1;
    private DcMotor transfer;

    private DcMotor Bootwheels;
    private DcMotor Intake;


    // Variables
    double backLeftPower = 0;
    double backRightPower = 0;


    // Base speed for servos
    final double BASE_SERVOSPEED = 0.01;


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, ("frontleft"));
        frontRight = hardwareMap.get(DcMotor.class, ("frontright"));
        backLeft = hardwareMap.get(DcMotor.class, ("backleft"));
        backRight = hardwareMap.get(DcMotor.class, ("backright"));


        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);


        flywheel1 = hardwareMap.get(DcMotor.class, "shooter");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Bootwheels = hardwareMap.get(DcMotor.class, "transferBootwheels");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);


        kicker = hardwareMap.get(Servo.class, "kicker");

        double shooterPower = 0.7;
        double intakePower = 1;
        double transferPower = 1;

        double transferPowerBootWheels = 1;

        double kickerDownPos = 0.7;
        double kickerUpPos = 0.4;


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();


        if (opModeIsActive()) {
            Intake.setPower(intakePower);
            flywheel1.setPower(shooterPower);
            transfer.setPower(transferPower);
            Bootwheels.setPower(transferPowerBootWheels);
            kicker.setPosition(kickerDownPos);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            kicker.setPosition(kickerDownPos);

            ElapsedTime time = new ElapsedTime();
            moveBot(0.7, 0, 0);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 1350) {
            }
            moveBot(0, 0, 0);

            ElapsedTime time1 = new ElapsedTime();
            moveBot(0, -0.3, 0);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 400) {
            }
            moveBot(0, 0, 0);

            ElapsedTime time2 = new ElapsedTime();
            moveBot(0, 0, 0);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 3000) {
            }
            moveBot(0, 0, 0);

            //Ball 1
            kicker.setPosition(kickerUpPos);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 500) {
                idle();
            }

            kicker.setPosition(kickerDownPos);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 200) {
                idle();
            }

            //Ball 2
            kicker.setPosition(kickerUpPos);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 500) {
                idle();
            }

            kicker.setPosition(kickerDownPos);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 200) {
                idle();
            }

            //Ball 3
            kicker.setPosition(kickerUpPos);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 500) {
                idle();
            }

            kicker.setPosition(kickerDownPos);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 200) {
                idle();
            }

            //Ball backup launch
            kicker.setPosition(kickerUpPos);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 500) {
                idle();
            }

            kicker.setPosition(kickerDownPos);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 200) {
                idle();
            }

            //ElapsedTime time3 = new ElapsedTime();
            moveBot(-0.2, 0.2, 0);
            time.reset();
            while (opModeIsActive() && time.milliseconds() < 1500) {
            }
            moveBot(0, 0, 0);
        }
    }


    // --- Movement helper function ---
    public void moveBot(double forwardPower, double turnPower, double strafePower) {
        frontLeft.setPower(forwardPower + turnPower + strafePower);
        frontRight.setPower(forwardPower - turnPower + strafePower);
        backLeft.setPower(forwardPower + turnPower - strafePower);
        backRight.setPower(forwardPower - turnPower - strafePower);
    }
}