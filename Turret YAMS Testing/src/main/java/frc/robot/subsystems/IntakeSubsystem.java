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

public class IntakeSubsystem extends SubsystemBase{
    private SparkMax intakeMotor = new SparkMax(Constants.Intake.IntakeMotorID, MotorType.kBrushless);

    private SmartMotorControllerConfig intakeSMCConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) //TODO : Configure to our robots gearing
    .withMotorInverted(true)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40));

    private SmartMotorController smc = new SparkWrapper(intakeMotor, DCMotor.getNeo550(1), intakeSMCConfig);

    private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smc)//Configure to our intakes measurements
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(0.5))
    .withUpperSoftLimit(RPM.of(6000))
    .withLowerSoftLimit(RPM.of(-6000))
    .withTelemetry("Intake", TelemetryVerbosity.HIGH);

    private FlyWheel intake = new FlyWheel(intakeConfig);

    
    public IntakeSubsystem(){}

    public Command setDuty(double cycle){
        return intake.set(cycle).withName("Intake.setDuty");
    }

    public Command setSpeed(double speed) {
        return intake.setSpeed(RPM.of(speed)).withName("intake.setSpeed");
    }

    public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier){
        return intake.setSpeed(speedSupplier);
    }

    public Command stop() {
        return intake.set(0);
    }

    public Command intake() {
        return setSpeed(Constants.Intake.INTAKE_SPEED).finallyDo(() -> stop());
    }
    
    public Command eject() {
        return setSpeed(-Constants.Intake.INTAKE_SPEED).finallyDo(() -> stop());
    }

    @Override
    public void periodic() {
        intake.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        intake.simIterate();
    }
}
