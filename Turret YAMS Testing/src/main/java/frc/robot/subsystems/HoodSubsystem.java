package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase{
    private Servo servo = new Servo(0);
    
    public void setAngle(double angle){
        servo.setAngle(angle);
    }

    //TODO : Add / Remove as Nessicary (No hood remove all, yes Hood add more)
}
