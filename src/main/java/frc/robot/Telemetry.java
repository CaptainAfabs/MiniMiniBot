package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Telemetry {
    private final double MaxSpeed;

    private final NetworkTable driveStateTable = NetworkTableInstance.getDefault().getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable
            .getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final DoubleArrayPublisher driveModuleStates = driveStateTable
            .getDoubleArrayTopic("ModuleStates").publish();

    public Telemetry(double maxSpeed) {
        this.MaxSpeed = maxSpeed;
    }

    public void telemeterize(SwerveDriveState state) {
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);

        double[] flattened = new double[state.ModuleStates.length * 2];
        for (int i = 0; i < state.ModuleStates.length; i++) {
            SwerveModuleState s = state.ModuleStates[i];
            flattened[i * 2] = s.angle.getRadians();
            flattened[i * 2 + 1] = s.speedMetersPerSecond / MaxSpeed;
        }
        driveModuleStates.set(flattened);
    }
}
