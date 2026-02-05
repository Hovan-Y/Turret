package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class PivotSubsystem extends SubsystemBase{
    private SparkMax pivotMotor = new SparkMax(Constants.Pivot.PivotMotorID, MotorType.kBrushless);

    private SmartMotorControllerConfig PivotSMCConfig = new SmartMotorControllerConfig() //TODO : Configure to whatever motor used
    .withControlMode(ControlMode.CLOSED_LOOP)
    //Feedback Constants (PID) TODO : Tune these values and move them to Constants
    .withClosedLoopController(25, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
    .withSimClosedLoopController(25, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
    //Feedforward Constants TODO : Tune these values and move them to Constants
    .withFeedforward(new SimpleMotorFeedforward(0, 10, 0))
    .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
    //Gearing TODO : Change Values to match our robots
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0)))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withSoftLimit(Degrees.of(0), Degrees.of(150))
    .withStatorCurrentLimit(Amps.of(10))
    .withClosedLoopRampRate(Seconds.of(0.1))
    .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorController pivotController = new SparkWrapper(pivotMotor, DCMotor.getNeo550(1), PivotSMCConfig);

    private final ArmConfig pivotConfig = new ArmConfig(pivotController)//TODO : Ensure Hard Limit matches our Robot
    .withSoftLimits(Degrees.of(0), Degrees.of(150))
    .withHardLimit(Degrees.of(0), Degrees.of(155))
    .withStartingPosition(Degrees.of(0))//Should match our Stow Position
    .withLength(Constants.Pivot.kLength) //TODO : Tune Values to match our robot's build (Go to Constants)
    .withMass(Constants.Pivot.kMass) 
    .withTelemetry("Pivot", TelemetryVerbosity.HIGH);

    private Arm pivot = new Arm(pivotConfig);

    public PivotSubsystem() {}

    public Command setAngle(Angle angle) {
        return pivot.setAngle(angle).withName("Pivot.SetAngle");
    }

    public Command setAngleDynamic(Supplier<Angle> angleSupplier) {
        return pivot.setAngle(angleSupplier).withName("Pivot.SetAngleDynamic");
    }

    public Command deployIntake() {//TODO : Change the Magnitude to match our robot
        return pivot.setAngle(Constants.Pivot.deployAngle);
    }

    public Command stowIntake() {
        return pivot.setAngle(Constants.Pivot.stowAngle);
    }

    @Override
    public void periodic() {
        pivot.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }
}