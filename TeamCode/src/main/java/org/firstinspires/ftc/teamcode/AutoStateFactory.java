package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous
@Disabled
public class AutoStateFactory extends LinearOpMode {
    Servo kickerServo;
    DcMotor shooter;
    DcMotor intake;
    DcMotor transferMotor;
    private Follower follower;
    private Pose startPose = new Pose(19.412, 120.821, Math.toRadians(180)); // Start Pose of our robot.
    private Pose scorePose = new Pose(59, 134, Math.toRadians(180));
    private Pose intake2PoseAlignment = new Pose(53, 82, Math.toRadians(180));
    private Pose intake1Pose = new Pose(10, 78, Math.toRadians(180));
    private Pose intake2Pose = new Pose(1, 50, Math.toRadians(180));
    private Pose intake2PoseControlPoint = new Pose(56,50, Math.toRadians(180));
    private Pose leave = new Pose(60,109, Math.toRadians(180));
    enum KickerStates{
        IDLE,
        Up1, Down1,
        Up2, Down2,
        Up3, Down3,

        Up4, Down4
    }

    //Byte Batallion

    enum AutoStates{
        AutoWait,
        MoveToShootPreload,
        ShootPreload,
        Intake1stSpike,
        MoveToShootSpike1,
        ShootSpike1,
        Intake2ndSpike,
        MoveToShootSpike2,
        ShootSpike2,
        Leave
    }

    boolean kick = false;
    boolean end = false;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        kickerServo = hardwareMap.get(Servo.class, "kicker");

        boolean isAllianceBlue = true;

        while (opModeInInit()){
            if (gamepad1.a){
                //red
                isAllianceBlue = false;
            }
            if (gamepad1.b){
                //blue
                isAllianceBlue=true;
            }
            telemetry.addData("Alliance is blue", isAllianceBlue);
            telemetry.update();
        }
        if (!isAllianceBlue) {
            startPose = new Pose(-startPose.getX(), startPose.getY(), Math.toRadians(180-Math.toDegrees(startPose.getHeading())));
            scorePose = new Pose(-scorePose.getX(), scorePose.getY(), Math.toRadians(180-Math.toDegrees(scorePose.getHeading())));
            intake2PoseAlignment = new Pose(-intake2PoseAlignment.getX(), intake2PoseAlignment.getY(), Math.toRadians(180-Math.toDegrees(intake2PoseAlignment.getHeading())));
            intake1Pose = new Pose(-intake1Pose.getX(), intake1Pose.getY(), Math.toRadians(180-Math.toDegrees(intake1Pose.getHeading())));
            intake2Pose = new Pose(-intake2Pose.getX(), intake2Pose.getY(), Math.toRadians(180-Math.toDegrees(intake2Pose.getHeading())));
            intake2PoseControlPoint = new Pose(-intake2PoseControlPoint.getX(), intake2PoseControlPoint.getY(), Math.toRadians(180-Math.toDegrees(intake2PoseControlPoint.getHeading())));
        }
        follower.setStartingPose(startPose);

        PathChain starttoscore = follower.pathBuilder().addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        PathChain scoretointake1 = follower.pathBuilder().addPath(new BezierLine(scorePose, intake1Pose))
                .setTangentHeadingInterpolation()
                .build();
        PathChain intake1toscore = follower.pathBuilder().addPath(new BezierLine(intake1Pose, scorePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading())
                .build();

        PathChain scoretoIntake2 = follower.pathBuilder().addPath(new BezierCurve(intake2PoseAlignment, intake2PoseControlPoint, intake2Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), intake2PoseAlignment.getHeading(), intake2PoseControlPoint.getHeading())
                .build();
        PathChain intake2toscore = follower.pathBuilder().addPath(new BezierLine(intake2Pose, scorePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading())
                .build();
        PathChain scorePosetoLeave = follower.pathBuilder().addPath(new BezierLine(scorePose, leave))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading())
                .build();



        StateMachine kickerMachine = new StateMachineBuilder()
                .state(KickerStates.IDLE)
                .onEnter(()->kickerServo.setPosition(1))
                .transition(()->kick)
                .state(KickerStates.Up1)
                .onEnter(()->{
                    kickerServo.setPosition(0.8);
                    kick=false;
                })
                .transitionTimed(0.4)
                .state(KickerStates.Down1)
                .onEnter(()->kickerServo.setPosition(1))
                .transitionTimed(0.4)
                .state(KickerStates.Up2)
                .onEnter(()->kickerServo.setPosition(0.8))
                .transitionTimed(0.4)
                .state(KickerStates.Down2)
                .onEnter(()->kickerServo.setPosition(1))
                .transitionTimed(0.4)
                .state(KickerStates.Up3)
                .onEnter(()->kickerServo.setPosition(0.8))
                .transitionTimed(0.4)
                .state(KickerStates.Down3)
                .onEnter(()->kickerServo.setPosition(1))
                .transitionTimed(0.4)
                .state(KickerStates.Up4)
                .onEnter(()->kickerServo.setPosition(0.8))
                .transitionTimed(0.4)
                .state(KickerStates.Down4)
                .onEnter(()->kickerServo.setPosition(1))
                .transitionTimed(0.4, KickerStates.IDLE)
                .build();

        StateMachine autoMachine = new StateMachineBuilder()
                .state(AutoStates.AutoWait)
                .transitionTimed(2.5)

                .state(AutoStates.MoveToShootPreload)
                .onEnter(()->follower.followPath(starttoscore))
                .transition(()->!follower.isBusy())
                .transitionTimed(2)

                .state(AutoStates.ShootPreload)
                .onEnter(()->kick = true)
                .transitionTimed(3.2)

                .state(AutoStates.Intake1stSpike)
                .onEnter(()->follower.followPath(scoretointake1))
                .transition(()->!follower.isBusy())
                .transitionTimed(2.5)

                .state(AutoStates.MoveToShootSpike1)
                .onEnter(()->follower.followPath(intake1toscore))
                .transition(()->!follower.isBusy())
                .transitionTimed(2.5)

                .state(AutoStates.ShootSpike1)
                .onEnter(()->kick = true)
                .transitionTimed(3)

                .state(AutoStates.Intake2ndSpike)
                .onEnter(()->follower.followPath(scoretoIntake2))
                .transition(()->!follower.isBusy())
                .transitionTimed(2.5)

                .state(AutoStates.MoveToShootSpike2)
                .onEnter(()->follower.followPath(intake2toscore))
                .transitionTimed(2.5)
                .transition(()->!follower.isBusy())

                .state(AutoStates.ShootSpike2)
                .onEnter(()->kick = true)
                .transitionTimed(3)

                .state(AutoStates.Leave).onEnter(()->end=true)
                .build();

        waitForStart();
        kickerMachine.start();
        autoMachine.start();
        shooter.setPower(0.6);
        intake.setPower(1);
        transferMotor.setPower(1);
        while (opModeIsActive() && !end){
            follower.update();
            kickerMachine.update();
            autoMachine.update();
            // Feedback to Driver Hub for debugging
            telemetry.addData("Auto state", autoMachine.getStateEnum());
            telemetry.addData("Kicker state", kickerMachine.getStateEnum());

            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}