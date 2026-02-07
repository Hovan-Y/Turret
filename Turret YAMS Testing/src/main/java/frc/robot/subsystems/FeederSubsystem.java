package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
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

public class FeederSubsystem extends SubsystemBase{
    private SparkMax feederMotor = new SparkMax(Constants.MotorID.Feeder, MotorType.kBrushless);
    
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) // 4:1 gear reduction TODO: Figure Gear Ratio
    .withMotorInverted(true)
    .withIdleMode(MotorMode.COAST) //TODO : Change Value if Nessicary (Coast/Brake)
    .withStatorCurrentLimit(Amps.of(20));

    private SmartMotorController smc = new SparkWrapper(feederMotor, DCMotor.getNeo550(1), smcConfig);

    private final FlyWheelConfig feederConfig = new FlyWheelConfig(smc)
    .withDiameter(Inches.of(4))// TODO : Tune to our robot's built
    .withMass(Pounds.of(0.5))
    .withUpperSoftLimit(RPM.of(6000))
    .withLowerSoftLimit(RPM.of(-6000))
    .withTelemetry("Feeder", TelemetryVerbosity.HIGH);

    private FlyWheel feeder = new FlyWheel(feederConfig);

    public Command setSpeed(AngularVelocity speed) {
        return feeder.setSpeed(speed);
    }

    public Command setSpeedDynamic(Supplier<AngularVelocity> speed) {
        return feeder.setSpeed(speed);
    }

    public Command stop() {
        return feeder.set(0).withName("Feeder.Stop");
    }

    public Command feed() {
        return setSpeed(Constants.Feeder.FEED_SPEED).finallyDo(() -> stop()).withName("Feeder.Feed");
    }

    public Command backFeed() {
        return setSpeed(Constants.Feeder.BACK_SPEED).finallyDo(() -> stop()).withName("Feeder.Backfeed");
    }

    @Override
    public void periodic() {
        feeder.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        feeder.simIterate();
    }
}
