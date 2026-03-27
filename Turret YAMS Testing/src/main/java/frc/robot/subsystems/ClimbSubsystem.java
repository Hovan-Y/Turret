package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimbSubsystem extends SubsystemBase{
    //Creating TalonFX Motor Controller
    private TalonFX climbMotor = new TalonFX(3);

    //Creating Smart Motor Controller Config
    private SmartMotorControllerConfig SMCConfig = new SmartMotorControllerConfig()
    .withClosedLoopController(0, 0, 0)
    .withIdleMode(MotorMode.BRAKE);

    //Creating Smart Motor Controller
    private SmartMotorController smc = new TalonFXWrapper(climbMotor, DCMotor.getFalcon500(1), SMCConfig);

    private final PivotConfig climbConfig = new PivotConfig(smc)
    .withHardLimit(Degree.of(1), Degree.of(10));

    private Pivot climb = new Pivot(climbConfig);

    //TODO : ADD MORE COMMANDS

    public Command CLIMB(){
        return climb.setAngle(Degree.of(10));
    }
}
