package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Spin;

public class MotorTesting extends SubsystemBase{
    public SparkMax testMotor = new SparkMax(1, MotorType.kBrushless);

    public void setSpeed(double speed) {
        testMotor.set(speed);
    }

    public void setIdle() {
        testMotor.set(0);
    }

    public Command SpinCommand() {
        return new Spin(this, 50);
    }
}
