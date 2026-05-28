package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record MegatagPoseEstimate(
        Pose2d fieldToRobot,
        double timestampSeconds,
        double latency,
        double avgTagArea,
        double quality,
        int[] fiducialIds)
        implements StructSerializable {

    public MegatagPoseEstimate {
        if (fieldToRobot == null) {
            fieldToRobot = new Pose2d();
        }
        if (fiducialIds == null) {
            fiducialIds = new int[0];
        }
    }

    public static final Struct<MegatagPoseEstimate> struct =
            new MegatagPoseEstimateStruct();

    private static class MegatagPoseEstimateStruct
            implements Struct<MegatagPoseEstimate> {

        @Override
        public Class<MegatagPoseEstimate> getTypeClass() {
            return MegatagPoseEstimate.class;
        }

        @Override
        public String getTypeName() {
            return "MegatagPoseEstimate";
        }

        @Override
        public int getSize() {
            // Pose2d + 4 doubles
            return Pose2d.struct.getSize() + 4 * Double.BYTES;
        }

        @Override
        public String getSchema() {
            return "Pose2d fieldToRobot;"
                 + "double timestampSeconds;"
                 + "double latency;"
                 + "double avgTagArea;"
                 + "double quality";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] { Pose2d.struct };
        }

        @Override
        public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
            Pose2d.struct.pack(bb, value.fieldToRobot());
            bb.putDouble(value.timestampSeconds());
            bb.putDouble(value.latency());
            bb.putDouble(value.avgTagArea());
            bb.putDouble(value.quality());
        }

        @Override
        public MegatagPoseEstimate unpack(ByteBuffer bb) {
            Pose2d pose = Pose2d.struct.unpack(bb);
            double timestamp = bb.getDouble();
            double latency = bb.getDouble();
            double avgTagArea = bb.getDouble();
            double quality = bb.getDouble();

            return new MegatagPoseEstimate(
                    pose,
                    timestamp,
                    latency,
                    avgTagArea,
                    quality,
                    new int[0]);
        }
    }
}
