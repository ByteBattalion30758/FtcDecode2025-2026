package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.shooter;
@TeleOp
@Configurable
public class Teleop extends LinearOpMode {
    private shooter shooter;
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public Servo kickerServo1, kickerServo2;
    public DcMotorEx intake, transferMotor, transferBootWheels;


    public static double shooterPower = 0.65;
    public static double intakePower = 1;
    public static double transferPower = 1;

    public static double transferPowerBootWheels = 1;

    public static double kickerUpPos = 0.5;

    public static double kickerUpPos2= 0.7;
    public static double kickerDownPos = 0.4;


    enum KickerStates1 {
        IDLE,

        Up1, Down1,

        Up2,Down2,

        Up3, Down3,

        Up4, Down4,
    }

    enum KickerStates2 {
        IDLE,

        Up1, Down1,

        Up2,Down2,

        Up3, Down3,

        Up4, Down4,
    }

    boolean kick = false;
    boolean end = false;

    public void runOpMode() {
        this.shooter = new shooter();
        shooter.init();
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
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

        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        kickerServo1 = hardwareMap.get(Servo.class, "kicker1");
        kickerServo2 = hardwareMap.get(Servo.class, "kicker2");
        transferBootWheels = hardwareMap.get(DcMotorEx.class,"transferBootwheels");

        StateMachine kickerMachine1 = new StateMachineBuilder()
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates1.IDLE)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerDownPos))
                .transition(()->gamepad1.y)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates1.Up1)
                .onEnter(() -> {
                    kickerServo1.setPosition(Teleop.kickerUpPos);
                    kick = false;
                })
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates1.Down1)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates1.Up2)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerUpPos))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates1.Down2)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates1.Up3)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerUpPos))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates1.Down3)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates1.Up4)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerUpPos))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates1.Down4)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.2, org.firstinspires.ftc.teamcode.Teleop.KickerStates1.IDLE)
                .build();

        StateMachine kickerMachine2 = new StateMachineBuilder()
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates2.IDLE)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transition(()->gamepad1.y)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates2.Up1)
                .onEnter(() -> {
                    kickerServo2.setPosition(Teleop.kickerUpPos2);
                    kick = false;
                })
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates2.Down1)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates2.Up2)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerUpPos2))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates2.Down2)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates2.Up3)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerUpPos2))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates2.Down3)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates2.Up4)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerUpPos2))
                .transitionTimed(0.2)
                .state(org.firstinspires.ftc.teamcode.Teleop.KickerStates2.Down4)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.2, org.firstinspires.ftc.teamcode.Teleop.KickerStates2.IDLE)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        kickerMachine1.start();
        kickerMachine2.start();
        while (opModeIsActive()) {
            transferMotor.setPower(1);

            if (gamepad1.b) {
                intake.setPower(-intakePower);
                transferBootWheels.setPower(-transferPowerBootWheels);
            }else{
                intake.setPower(intakePower);
                transferBootWheels.setPower(transferPowerBootWheels);
            }
             if (gamepad1.a) {
                 shooter.stopShooter();
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
            telemetry.addData("kickerMachine", kickerMachine1.getStateEnum());
            //telemetry.addData("shooter Velo", shooter.shooter.getVelocity());
            //telemetry.addData("shooter Pos ", shooter.shooter.getCurrentPosition());


            telemetry.update();
            kickerMachine1.update();
            kickerMachine2.update();

            // Measure velocity
        }
    }
    public void moveBot(double forwardPower, double turnPower, double strafePower) {
        frontLeft.setPower(forwardPower + turnPower + strafePower);
        frontRight.setPower(forwardPower - turnPower + strafePower);
        backLeft.setPower(forwardPower + turnPower - strafePower);
        backRight.setPower(forwardPower - turnPower - strafePower);

        }


    public static class Shooter {

        private DcMotorEx shooter;

        VoltageSensor voltageSensor;


        private double targetVelocity = 0.0;
        private double currentVelocity = 0.0;

        // --- Flywheel PIDF coefficients ---
        public static double kP = 0.008;

        public static double kS = 0.115; // Static feedforward
        public static double kV = 0.00038; // Velocity feedforward

        public static boolean enablePIDF = true;

        public Shooter(HardwareMap hardwareMap) {
            shooter = hardwareMap.get(DcMotorEx.class, "shooter");

            shooter.setDirection(DcMotorEx.Direction.REVERSE);

            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        public void setTargetVelocity(double target) {
            targetVelocity = target;
        }

        public double getCurrentVelocity() {
            return currentVelocity;
        }

        public void setDirectPower(double power) {
            double voltage = voltageSensor.getVoltage();
            if (voltage == 0) voltage = 12.0;

            power = power * 12/voltage;
            shooter.setPower(power);
        }

        public void update() {
            // Measure velocity
            currentVelocity = Math.abs(shooter.getVelocity());
            double outputPower;

            if (targetVelocity <= 0) {
                outputPower = 0;
            } else {
                outputPower = kV*targetVelocity + kS;
                if (enablePIDF){
                    outputPower += kP * (targetVelocity - currentVelocity);
                }
            }

            setDirectPower(outputPower);
        }

        public double getTargetVelocity() {
            return targetVelocity;
        }
    }


}

