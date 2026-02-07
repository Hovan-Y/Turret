package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;    
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class TurretSubsystem extends SubsystemBase{
    private final double MAX_ONE_DIR_FOV = 135; // degrees
    public final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375);

    private SparkMax spark = new SparkMax(Constants.TurretConstants.kMotorID, MotorType.kBrushless);

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    // Feedback Constants (PID Constants)
    .withClosedLoopController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, DegreesPerSecond.of(TurretConstants.MAX_VELOCITY), DegreesPerSecondPerSecond.of(TurretConstants.MAX_ACCELERATION))
    .withSimClosedLoopController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, DegreesPerSecond.of(TurretConstants.MAX_VELOCITY), DegreesPerSecondPerSecond.of(TurretConstants.MAX_ACCELERATION))
    // Feedforward Constants
    .withFeedforward(new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA))
    .withSimFeedforward(new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA))
    // Telemetry name and verbosity level
    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
    // Gearing from the motor rotor to final shaft.
    // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
    // You could also use .withGearing(12) which does the same thing.
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(10.75)))
    // Motor properties to prevent over currenting.
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25));

    private SmartMotorController smc = new SparkWrapper(spark, DCMotor.getNeo550(1), smcConfig);

    private final PivotConfig turretConfig = new PivotConfig(smc)
      .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV + 5))
      .withStartingPosition(Degrees.of(0))
      .withMOI(KilogramSquareMeters.of(0.1))
      .withTelemetry("Turret", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(
          new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(turretTranslation));

    private Pivot turret = new Pivot(turretConfig);

    // ShuffleboardTab tab = Shuffleboard.getTab("Turret");
    // GenericEntry pEntry = tab.add("SET P", Constants.TurretConstants.kP).getEntry();
    // GenericEntry dEntry = tab.add("SET D", Constants.TurretConstants.kD).getEntry();
    // GenericEntry iEntry = tab.add("SET I", Constants.TurretConstants.kI).getEntry();

    // GenericEntry kEntry = tab.add("SET S", Constants.TurretConstants.kS).getEntry();
    // GenericEntry vEntry = tab.add("SET V", Constants.TurretConstants.kV).getEntry();
    // GenericEntry aEntry = tab.add("SET A", Constants.TurretConstants.kA).getEntry();

    // GenericEntry maxVelocityEntry = tab.add("SET MAX VELOCITY", Constants.TurretConstants.MAX_VELOCITY).getEntry();
    // GenericEntry maxAccelerationEntry = tab.add("SET MAX ACCELERATION", Constants.TurretConstants.MAX_ACCELERATION).getEntry();
    public TurretSubsystem(){}

    public Command setAngle(Angle angle) {
        System.out.println("Turret");
        return turret.setAngle(angle);
    }

    public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
        return turret.setAngle(turretAngleSupplier);
    }

    public Command center() {
        return turret.setAngle(Degrees.of(0));
    }

    public Angle getRobotAdjustedAngle() {
        // Returns the turret angle in the robot's coordinate frame
        // since the turret is mounted backwards, we need to add 180 degrees
        return turret.getAngle().plus(Degrees.of(180));
    }

    public Angle getRawAngle() {
        return turret.getAngle();
    }

    public Command set(double dutyCycle) {
        return turret.set(dutyCycle);
    }

    public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();
    
    Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
        new Pose3d(
            turretTranslation,
            new Rotation3d(0, 0, turret.getAngle().in(Radians)))
    });

    // smcConfig
    // .withClosedLoopController(
    //     pEntry.getDouble(Constants.TurretConstants.kP),
    //     iEntry.getDouble(Constants.TurretConstants.kP),
    //     dEntry.getDouble(Constants.TurretConstants.kP),
    //     DegreesPerSecond.of(maxAccelerationEntry.getDouble(Constants.TurretConstants.MAX_ACCELERATION)),
    //     DegreesPerSecondPerSecond.of(maxVelocityEntry.getDouble(Constants.TurretConstants.MAX_VELOCITY))
    // )
    // .withSimClosedLoopController(
    //     pEntry.getDouble(Constants.TurretConstants.kP),
    //     iEntry.getDouble(Constants.TurretConstants.kP),
    //     dEntry.getDouble(Constants.TurretConstants.kP),
    //     DegreesPerSecond.of(maxAccelerationEntry.getDouble(Constants.TurretConstants.MAX_ACCELERATION)),
    //     DegreesPerSecondPerSecond.of(maxVelocityEntry.getDouble(Constants.TurretConstants.MAX_VELOCITY))
    // )
    // .withFeedforward(
    //     new SimpleMotorFeedforward(
    //         kEntry.getDouble(Constants.TurretConstants.kS), 
    //         vEntry.getDouble(Constants.TurretConstants.kV),
    //         aEntry.getDouble(Constants.TurretConstants.kA)
    // ))
    // .withSimFeedforward(
    //     new SimpleMotorFeedforward(
    //         kEntry.getDouble(Constants.TurretConstants.kS), 
    //         vEntry.getDouble(Constants.TurretConstants.kV),
    //         aEntry.getDouble(Constants.TurretConstants.kA)
    // ));
  }
}