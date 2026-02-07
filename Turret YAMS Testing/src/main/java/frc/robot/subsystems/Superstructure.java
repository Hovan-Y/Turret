package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Superstructure extends SubsystemBase {
    public final TurretSubsystem turret;

    private static final Angle TURRET_TOLERANCE = Degrees.of(1);

    private final Trigger isTurretOnTarget;
    private final Trigger isReadyToShoot;

    private Angle targetTurretAngle = Degrees.of(0);

    private Translation3d aimPoint = Constants.AimPoints.RED_HUB.value;

    public Superstructure(TurretSubsystem turret){
        this.turret = turret;

        this.isTurretOnTarget = new Trigger(
        () -> Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees)) < TURRET_TOLERANCE
            .in(Degrees));

        this.isReadyToShoot = isTurretOnTarget;
    }

    public Command stopAllCommand() {
        return Commands.parallel(
            turret.set(0).asProxy()
        ).withName("Superstructure.stopAll");
    }

    public Command setTurretForward() {
        System.out.println("Superstructure");
        return turret.setAngle(Degrees.of(0)).withName("Superstructure.setTurretForward");
    }

    public Command setTurretLeft() {
        return turret.setAngle(Degrees.of(45)).withName("Superstructure.setTurretLeft");
    }

    public Command setTurretRight() {
        return turret.setAngle(Degrees.of(-45)).withName("Superstructure.setTurretRight");
    }

    public Angle getTurretAngle() {
        return turret.getRawAngle();
    }

    public Angle getTargetTurretAngle() {
        return targetTurretAngle;
    }

}
