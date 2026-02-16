// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// BSD license

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    private final List<VisionFieldPoseEstimate> estimates = new LinkedList<>();
    private boolean useVision = true;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < io.length; i++) {
            disconnectedAlerts[i] =
                    new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
        }
    }

    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    private VisionFieldPoseEstimate fuseEstimates(
            VisionFieldPoseEstimate a, VisionFieldPoseEstimate b) {

        if (b.getTimestampSeconds() < a.getTimestampSeconds()) {
            VisionFieldPoseEstimate tmp = a;
            a = b;
            b = tmp;
        }

        Pose2d poseA = a.getVisionRobotPoseMeters();
        Pose2d poseB = b.getVisionRobotPoseMeters();

        var varianceA =
                a.getVisionMeasurementStdDevs().elementTimes(a.getVisionMeasurementStdDevs());
        var varianceB =
                b.getVisionMeasurementStdDevs().elementTimes(b.getVisionMeasurementStdDevs());

        Rotation2d fusedHeading = poseB.getRotation();
        if (varianceA.get(2, 0) < VisionConstants.kLargeVariance
                && varianceB.get(2, 0) < VisionConstants.kLargeVariance) {
            fusedHeading =
                    new Rotation2d(
                            poseA.getRotation().getCos() / varianceA.get(2, 0)
                                    + poseB.getRotation().getCos() / varianceB.get(2, 0),
                            poseA.getRotation().getSin() / varianceA.get(2, 0)
                                    + poseB.getRotation().getSin() / varianceB.get(2, 0));
        }

        double weightAx = 1.0 / varianceA.get(0, 0);
        double weightAy = 1.0 / varianceA.get(1, 0);
        double weightBx = 1.0 / varianceB.get(0, 0);
        double weightBy = 1.0 / varianceB.get(1, 0);

        Pose2d fusedPose =
                new Pose2d(
                        new Translation2d(
                                (poseA.getX() * weightAx + poseB.getX() * weightBx)
                                        / (weightAx + weightBx),
                                (poseA.getY() * weightAy + poseB.getY() * weightBy)
                                        / (weightAy + weightBy)),
                        fusedHeading);

        Matrix<N3, N1> fusedStdDev =
                VecBuilder.fill(
                        Math.sqrt(1.0 / (weightAx + weightBx)),
                        Math.sqrt(1.0 / (weightAy + weightBy)),
                        Math.sqrt(
                                1.0
                                        / (1.0 / varianceA.get(2, 0)
                                                + 1.0 / varianceB.get(2, 0))));

        return new VisionFieldPoseEstimate(
                fusedPose,
                b.getTimestampSeconds(),
                fusedStdDev,
                a.getNumTags() + b.getNumTags());
    }

    @Override
    public void periodic() {
        estimates.clear();

        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + i, inputs[i]);
        }

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            for (int tagId : inputs[cameraIndex].tagIds) {
                aprilTagLayout.getTagPose(tagId).ifPresent(tagPoses::add);
            }

            for (var observation : inputs[cameraIndex].poseObservations) {
                boolean rejectPose =
                        observation.tagCount() == 0
                                || (observation.tagCount() == 1
                                        && observation.ambiguity() > maxAmbiguity)
                                || Math.abs(observation.pose().getZ()) > maxZError
                                || observation.pose().getX() < 0.0
                                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                                || observation.pose().getY() < 0.0
                                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                    continue;
                }

                robotPosesAccepted.add(observation.pose());

                double stdDevFactor =
                        Math.pow(observation.averageTagDistance(), 2.0)
                                / observation.tagCount();

                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;

                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }

                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                estimates.add(
                        new VisionFieldPoseEstimate(
                                observation.pose().toPose2d(),
                                observation.timestamp(),
                                VecBuilder.fill(
                                        linearStdDev, linearStdDev, angularStdDev),
                                observation.tagCount()));
            }

            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/TagPoses",
                    tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[0]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[0]));

        if (useVision && !estimates.isEmpty()) {
            VisionFieldPoseEstimate fused = estimates.get(0);
            for (int i = 1; i < estimates.size(); i++) {
                fused = fuseEstimates(fused, estimates.get(i));
            }

            consumer.accept(
                    fused.getVisionRobotPoseMeters(),
                    fused.getTimestampSeconds(),
                    fused.getVisionMeasurementStdDevs());
        }
    }

    private double calculateCameraStdDev(VisionFieldPoseEstimate estimate) {
        final double baseCoeff = 0.125;

        int tagCount = estimate.getNumTags();
        if (tagCount <= 0) {
            return Double.POSITIVE_INFINITY;
        }

        double baseStdDev = estimate.getVisionMeasurementStdDevs().get(0, 0);
        return baseCoeff * baseStdDev / tagCount;
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
