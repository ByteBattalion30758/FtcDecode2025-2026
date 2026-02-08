package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp
public class ShooterTesting extends LinearOpMode {
    Shooter1 shooter;

    public static double targetVelocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooter = new Shooter1(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            shooter.setTargetVelocity(targetVelocity);
            shooter.update();
            telemetry.addData("Real Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.update();
        }
    }
}
