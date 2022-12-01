package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

public class OdometryHandler {
    private DifferentialDriveOdometry odometry;
    private Pose2d currentPose;
    private Supplier<Double> leftDistanceSupplier, rightDistanceSupplier;
    private Supplier<Double> yawSupplier;

    public OdometryHandler(Supplier<Double> leftDistanceSupplier, Supplier<Double> rightDistanceSupplier,
            Supplier<Double> yawSupplier) {
        this.leftDistanceSupplier = leftDistanceSupplier;
        this.rightDistanceSupplier = rightDistanceSupplier;
        this.yawSupplier = yawSupplier;
    }

    public void update() {
        Rotation2d gyroAngle = Rotation2d.fromDegrees(-yawSupplier.get());
        currentPose = odometry.update(gyroAngle, leftDistanceSupplier.get(), rightDistanceSupplier.get());
    }

    public void reset(Pose2d pose) {
        currentPose = new Pose2d(
            pose.getTranslation().getX(),
            pose.getTranslation().getY(),
            new Rotation2d());
        odometry = new DifferentialDriveOdometry(new Rotation2d(0), currentPose);
    }

    public Pose2d getCurrentPosition() {
        return currentPose;
    }

    public double getYaw() {
        return yawSupplier.get();
    }
}
