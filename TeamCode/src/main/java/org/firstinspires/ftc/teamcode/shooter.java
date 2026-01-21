package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class shooter extends LinearOpMode {
    public static double Ki = 0;
    public static double Kp = 0;

     public static double Kd = 0;
    public static double Kf = 0;
    public static double KS = 0;

    public static double closeShooting = 1300;
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public Servo kickerServo1, kickerServo2;
    public DcMotorEx shooter, intake, transferMotor, transferBootWheels;

    private double Setpoint = 0.0;
    private double LastError = 0.0;
    private double CumulativeError = 0.0;
    private ElapsedTime Timer = new ElapsedTime();

    private static final double INTEGRAL_MAX_OUTPUT = 0.3;

    public void setshooter(double power) {shooter.setPower(power);}


    public void runOpMode() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");



    }
    private double calculatePIDF(double currentVelo, double setpoint, double lastError, double cumulativeError, ElapsedTime timer) {

        // --- 1. Calculate Time Difference (dt) ---
        double deltaTime = timer.seconds();
        timer.reset();

        // --- 2. Calculate Error Terms ---
        double error = setpoint - currentVelo;

        // --- 3. Proportional Term (P) ---
        double pTerm = Kp * error;

        // --- 4. Integral Term (I) ---
        cumulativeError += error * deltaTime;

        // Anti-windup: Clamp the cumulative error to limit the I-term's power contribution.
        double maxITerm = INTEGRAL_MAX_OUTPUT / Ki;
        if (Ki != 0) {
            cumulativeError = Math.max(-maxITerm, Math.min(maxITerm, cumulativeError));
        } else {
            cumulativeError = 0.0;
        }

        double iTerm = Ki * cumulativeError;

        // --- 5. Derivative Term (D) ---
        double dTerm = Kd * ((error - lastError) / deltaTime);

        // --- 6. Feed-forward Term (F) ---
        double fTerm = Kf * setpoint;

        // --- 7. Calculate Total Output Power ---
        double outputPower = pTerm + iTerm + dTerm + fTerm;

        // --- 8. Clamp Output and Update State (This is tricky with primitive types) ---
        // We return the output, but the state variables (lastError, cumulativeError)
        // need to be updated in the calling scope. This is a limitation of this approach.
        // For simplicity, we will update the class members directly below.

        // Clamp the final power to the valid motor range [-1.0, 1.0]
        double clampedOutput = Math.max(-1.0, Math.min(1.0, outputPower));

        return clampedOutput;
    }

    /**
     * Sets the target velocity for both motors.
     * @param targetVelocity The desired velocity in encoder ticks/sec.
     */
    public void setShooterVelocity(double targetVelocity) {
        Setpoint = targetVelocity;
    }

    /**
     * Stops the shooter motors and resets the PIDF state.
     */
    public void stopShooter() {
        setShooterVelocity(0.0);

        // Reset ALL PIDF state variables for a clean restart
        CumulativeError = 0.0;
        LastError = 0.0;

        Timer.reset();
    }

    /**
     * The core function: runs the PIDF calculations and applies the power.
     * This MUST be called every cycle in the OpMode loop().
     */
    /**
     * The core function: runs the PIDF-S calculations for both motors and applies the power.
     * This MUST be called every cycle in the OpMode loop().
     */
    public void update() {

        // --- 1. Get Current Velocities ---
        double Velo = shooter.getVelocity();

        // ==========================================================
        //                       LEFT MOTOR PIDF-S
        // ==========================================================

        double leftDeltaTime = Timer.seconds();
        Timer.reset();

        double Error = Setpoint - Velo;

        // P-Term (Feedback)
        double PTerm = Kp * Error;

        // I-Term (Feedback with anti-windup)
        CumulativeError += Error * leftDeltaTime;
        double MaxITerm = INTEGRAL_MAX_OUTPUT / Ki;
        if (Ki != 0) {
            CumulativeError = Math.max(MaxITerm, Math.min(MaxITerm, CumulativeError));
        } else {
            CumulativeError = 0.0;
        }
        double ITerm = Ki * CumulativeError;

        // D-Term (Feedback)
        double DTerm = Kd * ((Error - LastError) / leftDeltaTime);

        // F-Term (Feedforward)
        double FTerm = Kf * Setpoint;

        // S-Term (Static Friction/Stiction)
        double STerm = 0.0;
        if (Setpoint != 0.0) {
            // Applies a constant bias in the direction of motion to overcome stiction
            STerm = Math.signum(Setpoint) * KS;
        }

        // Total Output Power = Feedback (P+I+D) + Feedforward (F) + Stiction (S)
        double Power = PTerm + ITerm + DTerm + FTerm + STerm;

        // Clamp and Update State
        Power = Math.max(-1.0, Math.min(1.0, Power));
        LastError = Error;


        // ==========================================================
        //                        APPLY POWER
        // ==========================================================

        shooter.setPower(Power);

        // Update Telemetry Variables

    }

    // --- Telemetry Helpers ---
    public double getTargetVelocity() {
        return Setpoint; // Since both setpoints should be the same
    }

    public double getCurrentVelocity() {
        return (shooter.getVelocity());
    }
}