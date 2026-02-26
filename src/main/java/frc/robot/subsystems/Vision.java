/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    /**
     * Vision subsystem for the robot - used to estimate the robot's pose based on
     * April Tags using PhotonVision.
     * Check out clickup knowledge base for more information on photonvision
     * debugging.
     * 
     * @author Max Clemetson, Siddhartha Hiremath
     * @since 2025-11
     * 
     */

    private final PhotonCamera shooterCamera;
    private final PhotonCamera intakeCamera;
    private final PhotonPoseEstimator shooterPhotonEstimator;
    private final PhotonPoseEstimator intakePhotonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private Swerve swerve;

    public Vision(Swerve swerve) {
        /**
         * Vision constructor for the robot.
         * 
         * @args Swerve swerve
         * @author Max Clemetson, Siddhartha Hiremath
         * @since 2025-11
         * @return void
         */
        this.swerve = swerve;

        shooterCamera = new PhotonCamera(Constants.VisionConstants.kAprilShooterCameraName);
        intakeCamera = new PhotonCamera(Constants.VisionConstants.kAprilIntakeCameraName);

        shooterPhotonEstimator = new PhotonPoseEstimator(Constants.VisionConstants.kTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.kShooterCamPosition);
        intakePhotonEstimator = new PhotonPoseEstimator(Constants.VisionConstants.kTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.kIntakeCamPosition);

        shooterPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        intakePhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void periodic() {
        // Process shooter camera
        for (var change : shooterCamera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = shooterPhotonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets(), shooterPhotonEstimator);

            visionEst.ifPresent(est -> {
                var estStdDevs = getEstimationStdDevs();
                swerve.addVisionMeasurement(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds,
                        estStdDevs);
            });
        }

        // Process intake camera
        for (var change : intakeCamera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = intakePhotonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets(), intakePhotonEstimator);

            visionEst.ifPresent(est -> {
                var estStdDevs = getEstimationStdDevs();
                swerve.addVisionMeasurement(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds,
                        estStdDevs);
            });
        }
    }

    /**
     * Calculates new standard deviations. This algorithm is a heuristic that
     * creates
     * dynamic standard deviations based on number of tags, estimation strategy, and
     * distance from the tags.
     *
     * @param estimatedPose   The estimated pose to guess standard deviations for.
     * @param targets         All targets in this camera frame
     * @param photonEstimator The PhotonPoseEstimator associated with this camera
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator photonEstimator) {

        if (estimatedPose.isEmpty()) {
            curStdDevs = VisionConstants.kSingleTagStdDevs;
            return;
        }

        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            curStdDevs = VisionConstants.kSingleTagStdDevs;
            return;
        }

        avgDist /= numTags;
        if (numTags > 1)
            estStdDevs = VisionConstants.kMultiTagStdDevs;
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        curStdDevs = estStdDevs;
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        /**
         * See docs from {@link #updateEstimationStdDevs()}
         * 
         * @args None
         * @author Max Clemetson. Siddhartha Hiremath
         * @since 2025-11
         * @return Matrix<N3, N1> curStdDevs
         */
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}