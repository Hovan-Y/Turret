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

public class HopperSubsystem extends SubsystemBase{
    private SparkMax hopperMotor = new SparkMax(Constants.MotorID.Hopper, MotorType.kBrushless);

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(5))) //5:1 gear Ratio
    .withMotorInverted(true)//TODO : Change if Nessicary (Feeding backward)
    .withIdleMode(MotorMode.BRAKE) //TODO : Change if Nessicary (COAST/BRAKE)
    .withStatorCurrentLimit(Amps.of(40));

    private SmartMotorController smc = new SparkWrapper(hopperMotor, DCMotor.getNEO(1), smcConfig);

    private final FlyWheelConfig hopperConfig = new FlyWheelConfig(smc)
    .withDiameter(Inches.of(10))//TODO : Tune these Values to match our robot's build
    .withMass(Pounds.of(0.5))
    .withUpperSoftLimit(RPM.of(6000))
    .withLowerSoftLimit(RPM.of(-6000))
    .withTelemetry("Hopper", TelemetryVerbosity.HIGH);

    private FlyWheel hopper = new FlyWheel(hopperConfig);

    public HopperSubsystem() {}

    public Command setSpeed(double speed) {
        return hopper.setSpeed(RPM.of(speed)).withName("Hopper.feedRPM");
    }

    public Command setSpeedDynamic(Supplier<AngularVelocity> speed) {
        return hopper.setSpeed(speed).withName("Hopper.feedDynamic");
    }

    public Command stop() {
        return hopper.set(0).withName("Hopper.Stop");
    }

    public Command feed() {
        return setSpeed(Constants.Hopper.speed).finallyDo(() -> stop()).withName("Hopper.Feed");
    }

    public Command backFeed() {
        return setSpeed(-Constants.Hopper.speed).finallyDo(() -> stop()).withName("Hopper.BackFeed");
    }

    @Override
    public void periodic() {
        hopper.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hopper.simIterate();
    }
}
