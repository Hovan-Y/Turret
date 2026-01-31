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
    public final PivotSubsystem pivot;
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

    private Translation3d aimPoint = Constants.AimPoints.getAllianceHubPosition();

    public Superstructure(FeederSubsystem feeder, HopperSubsystem hopper, IntakeSubsystem intake, PivotSubsystem pivot, ShooterSubsystem shooter, TurretSubsystem turret) {
        this.feeder = feeder;
        this.intake = intake;
        this.hopper = hopper;
        this.pivot = pivot;
        this.shooter = shooter;
        this.turret = turret;

        // Create triggers for checking if mechanisms are at their targets
        this.isShooterAtSpeed = new Trigger(
            () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM));

        this.isTurretOnTarget = new Trigger(
            () -> Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees)) < TURRET_TOLERANCE
                .in(Degrees));

        this.isReadyToShoot = isShooterAtSpeed.and(isTurretOnTarget);
    }

    public Command setTurretForward() {
        return turret.setAngle(Degrees.of(0)).withName("Superstructure.setTurretForward");
    }

    public Command setTurretLeft() {
        return turret.setAngle(Degrees.of(45)).withName("Superstructure.setTurretLeft");
    }

    public Command setTurretRight() {
        return turret.setAngle(Degrees.of(-45)).withName("Superstructure.setTurretRight");
    }

    public Command startIntake() {
        return intake.intake().withName("Superstructure.startIntake");
    }

    public Command stopIntake() {
        return intake.stop().withName("Superstructure.stopIntake");
    }

    public Command stopAllCommand() {
        return Commands.parallel(
            feeder.stop().asProxy(),
            hopper.stop().asProxy(),
            intake.stop().asProxy(),
            shooter.stop().asProxy(),
            turret.stop().asProxy()
        ).withName("Superstructure.stopAll");
    }
    /*
     * Stops Shooter and Turret movement
     */
    public Command stopShootingCommand() {
        return Commands.parallel(
            shooter.stop().asProxy(),
            turret.stop().asProxy()
        ).withName("Superstructure.stopShooting");
    }

    public Command stopFeedingCommand() {
        return Commands.parallel(
            feeder.stop().asProxy(),
            hopper.stop().asProxy()
        ).withName("Superstructure.stopFeeding");
    }

    public Command feedAllCommand() {
        return Commands.parallel(
            feeder.feed().asProxy(),
            hopper.feed().asProxy()
        ).withName("Superstructure.feedAll");
    }

    public Command backfeedAllCommand() {
        return Commands.parallel(
            feeder.backFeed().asProxy(),
            hopper.backFeed().asProxy(),
            intake.eject().asProxy()
        ).withName("Superstructure.backFeedAll");
    }

    public Command stopAndStowIntake() {
        return Commands.sequence(
            intake.stop(),
            pivot.stowIntake()
        ).withName("Superstructure.stopAndStowIntake");
    }

    public Command deployAndStartIntake() {
        return Commands.sequence(
            pivot.deployIntake(),
            intake.intake()
        ).withName("SuperStructure.deployAndStartIntake");
    }


    //TODO : ADD MORE COMMANDS
}
