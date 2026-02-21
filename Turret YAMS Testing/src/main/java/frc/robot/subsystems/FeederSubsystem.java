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
    //Creating feeder Motor
    private SparkMax feederMotor = new SparkMax(Constants.MotorID.Feeder, MotorType.kBrushless);
    
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this) //Creating the Smart Motor Controller Config (All of the Following are Optional, and there are a lot more configs that can be added)
    //Control Mode (TODO : Check what each does)
    .withControlMode(ControlMode.OPEN_LOOP)
    //Setting Telemetry
    .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
    //Setting Gearing
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) // 4:1 gear reduction TODO: Figure Gear Ratio
    //Invert Motor (Might not be Nessicary)
    .withMotorInverted(true)
    //Setting Idle Mode (TODO : Change Value if Nessicary (Coast/Brake))
    .withIdleMode(MotorMode.COAST) 
    //Setting Stal Stator Current Limit (TODO : Check what this actually does)
    .withStatorCurrentLimit(Amps.of(20));

    /*
     *Spark Wrapper 
     * @param SparkBase, Requires a Spark Controller (Spark Max or Spark Flex)
     * @param DCMotor, Requires the (Brushless) Motor used (just use DCMotor.getYOURMOTORHERE()), 
     * @param SmartMotorControllerConfig, Requires the SMCConfig Created
     * @return SmartMotor Controller
     */
    private SmartMotorController smc = new SparkWrapper(feederMotor, DCMotor.getNeo550(1), smcConfig);

    /*
     * Creating Flywheel Config
     * @param Smart Motor Controller
     * @return Flywheel Config 
     */
    //Creating FlyWheel Config
    private final FlyWheelConfig feederConfig = new FlyWheelConfig(smc) //All of the Following are optional
    //Set Diameter of flywheel (Requires a value of kind Distance)
    .withDiameter(Inches.of(4))// TODO : Tune to our robot's built
    //Set Mass of flywheel (Requires a value of kind Mass 
    .withMass(Pounds.of(0.5))// TODO : Tune to our robot's built
    //Set Soft Limit of the flywheel (Requires a value of kind AngularVelocity)
    .withUpperSoftLimit(RPM.of(6000)) //Going Clockwise
    .withLowerSoftLimit(RPM.of(-6000))//Going Counter Clockwise
    //Setting Telemetry
    .withTelemetry("Feeder", TelemetryVerbosity.HIGH);

    /*
     * Creating Flywheel
     * @param FlyWheelConfig
     * @return flywheel
     */
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
