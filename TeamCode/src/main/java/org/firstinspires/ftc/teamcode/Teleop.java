package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

@TeleOp
@Configurable
public class Teleop extends LinearOpMode {
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public Servo kickerServo;
    public DcMotor shooter, intake, transferMotor, transferBootWheels;


    public static double shooterPower = 0.65;
    public static double intakePower = 1;
    public static double transferPower = 1;

    public static double transferPowerBootWheels = 1;

    public static double kickerDownPos = 0.7;
    public static double kickerUpPos = 0.4;


    enum KickerStates {
        IDLE,

        Up1, Down1,

        Up2,Down2,

        Up3, Down3
    }
    boolean kick = false;
    boolean end = false;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        shooter = hardwareMap.get(DcMotor.class, "shooter");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        kickerServo = hardwareMap.get(Servo.class, "kicker");
        transferBootWheels = hardwareMap.get(DcMotor.class,"transferBootwheels");

        StateMachine kickerMachine = new StateMachineBuilder()
                .state(KickerStates.IDLE)
                .onEnter(()->kickerServo.setPosition(kickerDownPos))
                .transition(()->gamepad1.y)
                .state(KickerStates.Up1)
                .onEnter(()->{
                    kickerServo.setPosition(kickerUpPos);
                    kick=false;
                })
                .transitionTimed(0.15)
                .state(KickerStates.Down1)
                .onEnter(()->kickerServo.setPosition(kickerDownPos))
                .transitionTimed(0.15)
                .state(KickerStates.Up2)
                .onEnter(()->kickerServo.setPosition(kickerUpPos))
                .transitionTimed(0.15)
                .state(KickerStates.Down2)
                .onEnter(()->kickerServo.setPosition(kickerDownPos))
                .transitionTimed(0.15)
                .state(KickerStates.Up3)
                .onEnter(()->kickerServo.setPosition(kickerUpPos))
                .transitionTimed(0.15)
                .state(KickerStates.Down3)
                .onEnter(()->kickerServo.setPosition(kickerDownPos))
                .transitionTimed(0.15, KickerStates.IDLE)
                .build();



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        kickerMachine.start();
        while (opModeIsActive()) {
            shooter.setPower(shooterPower);
            transferMotor.setPower(transferPower);

            if (gamepad1.b) {
                intake.setPower(-intakePower);
                transferBootWheels.setPower(-transferPowerBootWheels);
            }else{
                intake.setPower(intakePower);
                transferBootWheels.setPower(transferPowerBootWheels);
            }

            double forwardPower = gamepad1.right_stick_x;
            double turnPower = -gamepad1.left_stick_y;
            double strafePower = gamepad1.left_stick_x;


            boolean sloweddown = gamepad1.right_bumper;
            if (sloweddown){
                forwardPower /= 3;
                turnPower /= 3;
                strafePower /= 3;
            }
            moveBot(forwardPower, turnPower, strafePower);
            telemetry.addData("kickerMachine", kickerMachine.getStateEnum());
            telemetry.update();
            kickerMachine.update();
        }
    }
    public void moveBot(double forwardPower, double turnPower, double strafePower) {
        frontLeft.setPower(forwardPower + turnPower + strafePower);
        frontRight.setPower(forwardPower - turnPower + strafePower);
        backLeft.setPower(forwardPower + turnPower - strafePower);
        backRight.setPower(forwardPower - turnPower - strafePower);
    }
}

