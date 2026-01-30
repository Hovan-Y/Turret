package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase{
    private static final double FEEDER_SPEED = 1.0;

    private SparkMax feederMotor = new SparkMax(Constants.FeederConstants.FeederMotorID, null);
    
}
