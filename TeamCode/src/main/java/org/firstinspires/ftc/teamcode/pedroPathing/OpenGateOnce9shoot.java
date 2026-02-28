package org.firstinspires.ftc.teamcode;

import android.media.AudioRouting;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class OpenGateOnce9shoot extends LinearOpMode {
    Servo kickerServo1, kickerServo2;
    DcMotor intake;
    DcMotor transferMotor, transferBootWheels;

    public Shooter1 shooter;

    public static double shooterVelocity = 1190;


    private Follower follower;
    private Pose startPose = new Pose(19.412, 120.821, Math.toRadians(135)); // Start Pose of our robot.
    private Pose scorePose = new Pose(48, 85, Math.toRadians(130));
    private Pose lastscorePose = new Pose(48, 85, Math.toRadians(130));

    private Pose intake2PoseAlignment = new Pose(53, 80, Math.toRadians(180));
    private Pose intake1Pose = new Pose(10, 82, Math.toRadians(180));
    private Pose intake1openGateControlPoint = new Pose(37,72, Math.toRadians(180));

    private Pose intake2openGateControlPoint = new Pose(67,72, Math.toRadians(180));



    private Pose openGate = new Pose(14,73, Math.toRadians(180));
    private Pose intake2Pose = new Pose(-4, 56, Math.toRadians(180));
    private Pose intake2PoseControlPoint = new Pose(56, 40, Math.toRadians(180));

    private Pose intake3PoseAlignment = new Pose(44, 35, Math.toRadians(180));
    private Pose intake3Pose = new Pose(6.5, 35, Math.toRadians(180));
    private Pose leave = new Pose(18, 89, Math.toRadians(180));

    enum KickerStates1 {
        IDLE,

        Up1, Down1,

        Up2,Down2,

        Up3, Down3,

        Up4, Down4,

        Up5, Down5,
    }

    enum KickerStates2 {
        IDLE,

        Up1, Down1,

        Up2,Down2,

        Up3, Down3,

        Up4, Down4,

        Up5,Down5,
    }

    enum AutoStates {
        AutoWait,
        MoveToShootPreload,
        ShootPreload,
        Intake1stSpike,
        OpenGate,

        OpenGate1Wait,
        MovetoShootspike1,
        ShootSpike1,
        MoveToSpike2,
        IntakeSpike2,
        MovetoShootspike2,

        ShootSpike2,
        Leave,

        End
    }

    boolean kick = false;
    boolean end = false;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);

        shooter = new Shooter1(hardwareMap);
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        kickerServo1 = hardwareMap.get(Servo.class, "kicker1");
        kickerServo2 = hardwareMap.get(Servo.class, "kicker2");
        transferBootWheels = hardwareMap.get(DcMotorEx.class, "transferBootwheels");


        boolean isAllianceBlue = true;

        while (opModeInInit()) {
            if (gamepad1.a) {
                //red
                isAllianceBlue = false;
            }
            if (gamepad1.b) {
                isAllianceBlue = true;
            }
            telemetry.addData("Alliance is blue", isAllianceBlue);
            telemetry.addData("Position", follower.getPose());
            follower.updatePose();
            telemetry.update();
        }
        if (!isAllianceBlue) {
            startPose = new Pose(-startPose.getX(), startPose.getY(), Math.toRadians(180 - Math.toDegrees(startPose.getHeading())));
            scorePose = new Pose(-scorePose.getX(), scorePose.getY(), Math.toRadians(180 - Math.toDegrees(scorePose.getHeading())));
            lastscorePose = new Pose(-lastscorePose.getX(), lastscorePose.getY(), Math.toRadians(180 - Math.toDegrees(lastscorePose.getHeading())));
            intake2PoseAlignment = new Pose(-intake2PoseAlignment.getX(), intake2PoseAlignment.getY(), Math.toRadians(180 - Math.toDegrees(intake2PoseAlignment.getHeading())));
            intake1Pose = new Pose(-intake1Pose.getX(), intake1Pose.getY(), Math.toRadians(180 - Math.toDegrees(intake1Pose.getHeading())));
            intake2Pose = new Pose(-intake2Pose.getX(), intake2Pose.getY(), Math.toRadians(180 - Math.toDegrees(intake2Pose.getHeading())));
            intake2PoseControlPoint = new Pose(-intake2PoseControlPoint.getX(), intake2PoseControlPoint.getY(), Math.toRadians(180 - Math.toDegrees(intake2PoseControlPoint.getHeading())));
            intake3PoseAlignment = new Pose(-intake3PoseAlignment.getX(), intake3PoseAlignment.getY(), Math.toRadians(180 - Math.toDegrees(intake3PoseAlignment.getHeading())));
            intake3Pose = new Pose(-intake3Pose.getX(), intake3Pose.getY(), Math.toRadians(180 - Math.toDegrees(intake3Pose.getHeading())));
            leave = new Pose(-leave.getX(), leave.getY(), Math.toRadians(180 - Math.toDegrees(leave.getHeading())));
            intake1openGateControlPoint= new Pose(-intake1openGateControlPoint.getX(), intake1openGateControlPoint.getY(), Math.toRadians(180-Math.toDegrees(intake1openGateControlPoint.getHeading())));
            openGate= new Pose(-openGate.getX(), openGate.getY(), Math.toRadians(180-Math.toDegrees(openGate.getHeading())));
            intake2openGateControlPoint = new Pose(-intake2openGateControlPoint.getX(), intake2openGateControlPoint.getY(), Math.toRadians(180-Math.toDegrees(intake2openGateControlPoint.getHeading())));

        }
        follower.setStartingPose(startPose);

        PathChain starttoscore = follower.pathBuilder().addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        PathChain scoretointake1 = follower.pathBuilder().addPath(new BezierLine(scorePose, intake1Pose))
                .addParametricCallback(0.1, () -> follower.setMaxPower(0.5))
                .setTangentHeadingInterpolation()
                .build();
        PathChain intake1toopenGate = follower.pathBuilder().addPath(new BezierCurve(intake1Pose, intake1openGateControlPoint, openGate))
                .addParametricCallback(0.05, () -> follower.setMaxPower(1))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), intake1openGateControlPoint.getHeading(), openGate.getHeading())
                .build();

        PathChain openGatetoScore1 = follower.pathBuilder().addPath(new BezierLine(openGate, scorePose))
                .addParametricCallback(0.3, () -> follower.setMaxPower(0.5))
                .setLinearHeadingInterpolation(openGate.getHeading(), scorePose.getHeading())
                .build();

        PathChain scoretoIntake2PoseAlignment = follower.pathBuilder().addPath(new BezierCurve(intake2PoseAlignment, intake2PoseControlPoint, intake2Pose))
                .addParametricCallback(0.3, () -> follower.setMaxPower(1))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), intake2PoseAlignment.getHeading(), intake2PoseControlPoint.getHeading())
                .build();
        PathChain intake2PosetoOpenGate = follower.pathBuilder().addPath(new BezierCurve(intake2Pose, intake2openGateControlPoint, openGate))
                .addParametricCallback(0.1, () -> follower.setMaxPower(1))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), intake2openGateControlPoint.getHeading(), openGate.getHeading())
                .build();
        PathChain opengateoscore2 = follower.pathBuilder().addPath(new BezierCurve(intake2Pose, intake2PoseControlPoint, lastscorePose))
                .addParametricCallback(0.05, () -> follower.setMaxPower(1))
                .setLinearHeadingInterpolation(openGate.getHeading(), scorePose.getHeading())
                .build();
        PathChain scorePosetoLeave = follower.pathBuilder().addPath(new BezierLine(lastscorePose, leave))
                .setLinearHeadingInterpolation(leave.getHeading(), scorePose.getHeading())
                .build();


        StateMachine kickerMachine1 = new StateMachineBuilder()
                .state(KickerStates1.IDLE)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerUpPos))
                .transition(() -> kick)
                .state(KickerStates1.Up1)
                .onEnter(() -> {
                    kickerServo1.setPosition(Teleop.kickerDownPos);
                    kick = false;
                })
                .transitionTimed(0.4)
                .state(KickerStates1.Down1)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerUpPos))
                .transitionTimed(0.4)
                .state(KickerStates1.Up2)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.4)
                .state(KickerStates1.Down2)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerUpPos))
                .transitionTimed(0.4)
                .state(KickerStates1.Up3)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.4)
                .state(KickerStates1.Down3)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerUpPos))
                .transitionTimed(0.4)
                .state(KickerStates1.Up4)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.4)
                .state(KickerStates1.Down4)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerUpPos))
                .transitionTimed(0.4, KickerStates1.IDLE)
                .state(KickerStates1.Up5)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.4)
                .state(KickerStates1.Down5)
                .onEnter(() -> kickerServo1.setPosition(Teleop.kickerUpPos))
                .transitionTimed(0.4, KickerStates1.IDLE)


                .build();

        StateMachine kickerMachine2 = new StateMachineBuilder()
                .state(KickerStates2.IDLE)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transition(() -> kick)
                .state(KickerStates2.Up1)
                .onEnter(() -> {
                    kickerServo2.setPosition(Teleop.kickerUpPos2);
                    kick = false;
                })
                .transitionTimed(0.4)
                .state(KickerStates2.Down1)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.4)
                .state(KickerStates2.Up2)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerUpPos2))
                .transitionTimed(0.4)
                .state(KickerStates2.Down2)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.4)
                .state(KickerStates2.Up3)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerUpPos2))
                .transitionTimed(0.4)
                .state(KickerStates2.Down3)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.4)
                .state(KickerStates2.Up4)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerUpPos2))
                .transitionTimed(0.4)
                .state(KickerStates2.Down4)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.4, KickerStates2.IDLE)
                .state(KickerStates2.Up5)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerUpPos2))
                .transitionTimed(0.4)
                .state(KickerStates2.Down5)
                .onEnter(() -> kickerServo2.setPosition(Teleop.kickerDownPos))
                .transitionTimed(0.4, KickerStates2.IDLE)
                .build();


        StateMachine autoMachine = new StateMachineBuilder()
                .state(AutoStates.AutoWait)
                .transitionTimed(1.5)

                .state(AutoStates.MoveToShootPreload)
                .onEnter(() -> follower.followPath(starttoscore))
                .transition(() -> !follower.isBusy())
                .transitionTimed(2)

                .state(AutoStates.ShootPreload)
                .onEnter(() -> kick = true)
                .transitionTimed(3.2)

                .state(AutoStates.Intake1stSpike)
                .onEnter(() -> follower.followPath(scoretointake1))
                .transition(() -> !follower.isBusy())
                .transitionTimed(2.5)

                .state(AutoStates.OpenGate)
                .onEnter(() -> follower.followPath(intake1toopenGate))
                .transition(() -> !follower.isBusy())
                .transitionTimed(4.5)

                .state(AutoStates.OpenGate1Wait)
                .transitionTimed(1.5)


                .state(AutoStates.MovetoShootspike1)
                .onEnter(() -> follower.followPath(openGatetoScore1))
                .transition(() -> !follower.isBusy())
                .transitionTimed(2)

                .state(AutoStates.ShootSpike1)
                .onEnter(() -> kick = true)
                .transitionTimed(3)

                .state(AutoStates.MoveToSpike2)
                .onEnter(() -> follower.followPath(scoretoIntake2PoseAlignment))
                .transition(() -> !follower.isBusy())
                .transitionTimed(2.5)


                .state(AutoStates.MovetoShootspike2)
                .onEnter(() -> follower.followPath(opengateoscore2))
                .transition(() -> !follower.isBusy())
                .transitionTimed(2.5)

                .state(AutoStates.ShootSpike2)
                .onEnter(()-> kick = true)
                .transitionTimed(3)

                .state(AutoStates.Leave)
                .onEnter(() -> follower.followPath(scorePosetoLeave))
                .transition(() -> !follower.isBusy())
                .transitionTimed(2.5)

                .state(AutoStates.End)
                .onEnter(() -> end = true)
                .build();

        waitForStart();
        kickerMachine1.start();
        kickerMachine2.start();
        autoMachine.start();
        intake.setPower(1);

        transferMotor.setPower(1);
        transferBootWheels.setPower(1);

        while (opModeIsActive() && !end) {
            shooter.setTargetVelocity(shooterVelocity);
            follower.update();
            kickerMachine1.update();
            kickerMachine2.update();
            autoMachine.update();
            // Feedback to Driver Hub for debugging
            telemetry.addData("Auto state", autoMachine.getStateEnum());
            shooter.update();
            telemetry.addData("Kicker state", kickerMachine1.getStateEnum());
            telemetry.addData("shooter Velo", shooter.getCurrentVelocity());

            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
