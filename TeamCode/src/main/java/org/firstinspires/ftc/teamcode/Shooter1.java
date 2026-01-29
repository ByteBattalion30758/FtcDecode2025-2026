package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Configurable


public class Shooter1 {

    private DcMotorEx shooter;

    VoltageSensor voltageSensor;


    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;

    // --- Flywheel PIDF coefficients ---
    public static double kP = 0.000037;

    public static double kS = 0.06; // Static feedforward
    public static double kV = 0.000385; // Velocity feedforward

    public static boolean enablePIDF = true;

    public Shooter1(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        //shooter.setDirection(DcMotorEx.Direction.REVERSE);

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
