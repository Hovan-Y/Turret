package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Superstructure extends SubsystemBase {
    public final FeederSubsystem feeder;
    public final HopperSubsystem hopper;
    public final IntakeSubsystem intake;
    public final ShooterSubsystem shooter;
    public final TurretSubsystem turret;

    // Tolerance for "at setpoint" checks
    private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
    private static final Angle TURRET_TOLERANCE = Degrees.of(1);

    // Triggers for readiness checks
    private final Trigger isShooterAtSpeed;
    private final Trigger isTurretOnTarget;
    private final Trigger isReadyToShoot;

    private AngularVelocity targetShooterSpeed = RPM.of(0);
    private Angle targetTurretAngle = Degrees.of(0);

    private Translation3d aimPoint = Constants.AimPoints.RED_HUB.value;

    public Superstructure(FeederSubsystem feeder, HopperSubsystem hopper, IntakeSubsystem intake, ShooterSubsystem shooter, TurretSubsystem turret) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.turret = turret;
        this.intake = intake;
        this.hopper = hopper;

        // Create triggers for checking if mechanisms are at their targets
    this.isShooterAtSpeed = new Trigger(
        () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM));

    this.isTurretOnTarget = new Trigger(
        () -> Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees)) < TURRET_TOLERANCE
            .in(Degrees));

    this.isReadyToShoot = isShooterAtSpeed.and(isTurretOnTarget);
    }


    public Command stopAllCommand() {
        return Commands.parallel(
            feeder.stopCommand().asProxy(),
            hopper.stopCommand().asProxy(),
            intake.stop().asProxy(),
            shooter.stop().asProxy(),
            turret.set(0)
        ).withName("Superstructure.stopAll");
    }
    /*
     * Stops Shooter and Turret movement
     */
    public Command stopShootingCommand() {
        return Commands.parallel(
            shooter.stop().asProxy(),
            turret.set(0).asProxy()
        ).withName("Superstructure.StopShooting");
    }

    public Command stopFeedingCommand() {
        return Commands.parallel(
            feeder.stopCommand().asProxy(),
            hopper.stopCommand().asProxy()
        );
    }

    public Command stopIntakeCommand() {
        return intake.stop();
    }

    
}
