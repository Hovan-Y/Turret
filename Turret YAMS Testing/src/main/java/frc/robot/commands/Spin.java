package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorTesting;

public class Spin extends Command{
    public MotorTesting testSubsystem;
    public double speed;

    public Spin(MotorTesting testSubsystem, double speed) {
        this.testSubsystem = testSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        testSubsystem.setIdle();
    }

    @Override
    public void execute() {
        testSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        testSubsystem.setIdle();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}