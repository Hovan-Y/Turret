package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {
  private SparkMax rollerMotor = new SparkMax(Constants.MotorID.Intake, MotorType.kBrushless);
  private SparkMax pivotMotor = new SparkMax(Constants.MotorID.Pivot, MotorType.kBrushless);


  private SmartMotorControllerConfig rollerSMCConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withClosedLoopController(0, 0, 0)
      .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // Direct drive, adjust if geared
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController rollerSMC = new SparkWrapper(rollerMotor, DCMotor.getNeo550(1), rollerSMCConfig);

  private final FlyWheelConfig rollerConfig = new FlyWheelConfig(rollerSMC)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Intake", TelemetryVerbosity.HIGH);

  private FlyWheel roller = new FlyWheel(rollerConfig);

  // 5:1, 5:1, 60/18 reduction
  private SmartMotorControllerConfig pivotSMCConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(25, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
      .withFeedforward(new ArmFeedforward(0, 10, 0, 0))
      .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0)))
      // .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 /
      // 18.0, 42)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withSoftLimit(Degrees.of(0), Degrees.of(150))
      .withStatorCurrentLimit(Amps.of(10))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));



  private SmartMotorController pivotSMC = new SparkWrapper(pivotMotor, DCMotor.getNEO(1),
     pivotSMCConfig);

  private final ArmConfig pivotConfig = new ArmConfig(pivotSMC)
      .withSoftLimits(Degrees.of(0), Degrees.of(150))
      .withHardLimit(Degrees.of(0), Degrees.of(155))
      .withStartingPosition(Degrees.of(0))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(2)) // Reis says: 2 pounds, not a lot
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

  private Arm pivot = new Arm(pivotConfig);

  public IntakeSubsystem() {}

  public Command stop() {
    return roller.set(0);
  }

  public Command setPivotAngle(Angle angle) {
    return pivot.setAngle(angle).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotMotor.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
  }

  @Override
  public void periodic() {
    roller.updateTelemetry();
    pivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    roller.simIterate();
    pivot.simIterate();
  }
}