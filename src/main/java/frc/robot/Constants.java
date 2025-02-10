package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
//Only constants in life are change, death, and taxes
public final class Constants {
    public static final class AprilTags {
        public static final double[] coralBlueTags = { 17, 18, 19, 20, 21, 22 };
        public static final double[] coralRedTags =  { 6, 7, 8, 9, 10, 11 };

        // X is horizontal distance, Y is distance out from coral aprilTag -- half of bot width (bumpers included)
        public static final Translation2d coralOffset = new Translation2d(0, 1);
        //Slow for testing
        public static final PathConstraints constraints = new PathConstraints(3, 3, 5, 12.5664);
    }
    public static final double stickDeadband = 0.1;
   
    public static final class PoseEstimator{
        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> VisionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }
}
