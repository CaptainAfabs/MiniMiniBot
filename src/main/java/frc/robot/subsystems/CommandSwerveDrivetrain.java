package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * CTRE Phoenix6 swerve subsystem wrapped as a WPILib Command-based Subsystem.
 *
 * Generated style follows the Tuner X 2025 swerve project template; the
 * actual hardware values live in {@link frc.robot.generated.TunerConstants}.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final Rotation2dSupplier m_blueAlliancePerspective = () -> edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0);
    private final Rotation2dSupplier m_redAlliancePerspective = () -> edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180);
    private boolean m_hasAppliedOperatorPerspective = false;

    @FunctionalInterface
    private interface Rotation2dSupplier {
        edu.wpi.first.math.geometry.Rotation2d get();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
                                   SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        // On enable, latch the alliance-perspective heading so field-centric
        // drives the right way on both alliances.
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                setOperatorPerspectiveForward(
                        alliance == Alliance.Red
                                ? m_redAlliancePerspective.get()
                                : m_blueAlliancePerspective.get());
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
