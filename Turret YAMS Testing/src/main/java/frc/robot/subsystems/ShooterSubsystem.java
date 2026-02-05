package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase{
    private final SparkFlex leaderMotor = new SparkFlex(Constants.Shooter.leaderMotor, MotorType.kBrushless);
    private final SparkFlex followerMotor = new SparkFlex(Constants.Shooter.followerMotor, MotorType.kBrushless);

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withFollowers(Pair.of(followerMotor, true))
    .withControlMode(ControlMode.CLOSED_LOOP)
    // Feedback Constants (PID Constants) TODO : Tune PID Values (Go to Constants)
    .withClosedLoopController(0.00936, 0, 0)
    .withSimClosedLoopController(0.00936, 0, 0)
    // Feedforward Constants TODO : Tune Feedforward Values (Go to Constants)
    .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
    .withSimFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
    .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
    //Gearing TODO: Ensure our Gearing matches these values
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)//TODO : Change if nessicary (COAST/BRAKE)
    .withStatorCurrentLimit(Amps.of(40));

    private final SmartMotorController smc = new SparkWrapper(leaderMotor, DCMotor.getNeoVortex(2), smcConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
    .withDiameter(Inches.of(4))//TODO : Tune Values to match our robot's build
    .withMass(Pounds.of(1))
    .withUpperSoftLimit(RPM.of(6000))
    .withLowerSoftLimit(RPM.of(0))
    .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    private final FlyWheel shooter = new FlyWheel(shooterConfig);

    public ShooterSubsystem() {}

    public AngularVelocity getSpeed() {
        return shooter.getSpeed();
    }

    public Command setSpeed(AngularVelocity speed) {
        return shooter.setSpeed(speed);
    }

    public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
        return shooter.setSpeed(speedSupplier);
    }

    public Command stop() {
        return shooter.set(0);
    }

    public Command spinUp() {
        return setSpeed(RPM.of(5500));
    }

    public Command sysId() {
        return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/LeaderVelocity", leaderMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/FollowerVelocity", followerMotor.getEncoder().getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
    }

    private Distance wheelRadius() {
        return Inches.of(4).div(2);
    }

    public LinearVelocity getTangentialVelocity() {
        // Calculate tangential velocity at the edge of the wheel and convert to
        // LinearVelocity

        return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
            * wheelRadius().in(Meters));
    }
}
