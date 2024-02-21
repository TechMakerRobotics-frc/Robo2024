package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.LimelightHelper;
import frc.robot.util.RollingAverage;

public class LimelightSubsystem extends SubsystemBase {
    private static LimelightSubsystem limelightFront;

    private RollingAverage txAverage, tyAverage, taAverage, xAverage;

    private String limelightName = "limelight";

    public LimelightSubsystem() {
        txAverage = new RollingAverage();
        tyAverage = new RollingAverage();
        taAverage = new RollingAverage();
        xAverage = new RollingAverage(4,getBotpose().getX());
        setPipeline(2);
    }

    public static LimelightSubsystem getInstance() {
        if (limelightFront == null) {
            limelightFront = new LimelightSubsystem();
        }
        return limelightFront;
    }
    public void startLimelight() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            setPipeline(LimelightConstants.kRedSpeakerPipeline);
        } else {
            setPipeline(LimelightConstants.kBlueSpeakerPipeline);
        }
    }
    @Override
    public void periodic() {
        updateRollingAverages();

    }

    public void startAveragingX(){
        xAverage = new RollingAverage(4,getBotpose().getX());
    }

    public double getAveragePoseX() {
        return xAverage.getAverage();
    }

    public Translation2d getBotXY() {
        double[] result;
        if(DriverStation.getAlliance().get() == Alliance.Red){
            result = LimelightHelper.getBotPose_wpiRed(limelightName);
        }
        else{
            result = LimelightHelper.getBotPose_wpiBlue(limelightName);
        }
        
        if (result.length > 0.0) {
            return new Translation2d(result[0], result[1]);
        }
        return new Translation2d(0, 0);
    }

    public Pose2d getBotpose() {
        double[] result;
        if(DriverStation.getAlliance().get() == Alliance.Red){
            result = LimelightHelper.getBotPose_wpiRed(limelightName);
        }
        else{
            result = LimelightHelper.getBotPose_wpiBlue(limelightName);
        }

        if (result.length > 0.0) {
            return new Pose2d(new Translation2d(result[0], result[1]), new Rotation2d(Math.toRadians(result[5])));
        }
        return new Pose2d();
    }

    // Tv is whether the limelight has a valid target
    public boolean getTv() {
        return LimelightHelper.getTV(limelightName);
    }

    // Tx is the Horizontal Offset From Crosshair To Target
    public double getTx() {
        return LimelightHelper.getTX(limelightName);
    }

    // Ty is the Vertical Offset From Crosshair To Target
    public double getTy() {
        return LimelightHelper.getTY(limelightName);
    }

    public double getTa() {
        return LimelightHelper.getTA(limelightName);
    }

    public double getTargetID(){
        return LimelightHelper.getFiducialID(limelightName);
    }

    public double getTxAverage() {
        return txAverage.getAverage();
    }

    public double getTyAverage() {
        return tyAverage.getAverage();
    }

    public double getTaAverage() {
        return taAverage.getAverage();
    }

    // Class ID of primary neural detector result or neural classifier result
    public double getNeuralClassID() {
        return LimelightHelper.getNeuralClassID(limelightName);
    }

    public double getDistance() {
        if (!hasTarget()) {
            return 0;
        } else {
            // a1 = LL panning angle
            // a2 = additional angle to target
            // tan(a1 + a2) = h/d
            // d = h/tan(a1+a2)
            double a2 = getTy();
            double a1 = LimelightConstants.kLimelightPanningAngle;
            double h1 = LimelightConstants.kLimelightHeight;
            double h2 = 1.4511; // Place holder Height of target

            double angleToGoal = (a1 + a2);
            double angleToGoalRadian = Math.toRadians(angleToGoal);
          
            return (h2 - h1) / Math.tan(angleToGoalRadian);
        }
    }

    public int getTagsSeen() {
        return LimelightHelper.getNumberOfAprilTagsSeen(limelightName);
    }


    public boolean hasTarget() {
        return getTv();
    }

    public void updateRollingAverages() {
        if (hasTarget()) {
            txAverage.add(getTx());
            tyAverage.add(getTy());
            taAverage.add(getTa());
        }
    }

    public void resetRollingAverages(){
        txAverage.clear();
        tyAverage.clear();
        taAverage.clear();
    }

    public void setPipeline(int pipelineNum) {
        LimelightHelper.setPipelineIndex(limelightName, pipelineNum);
    }

    public int getPipeline(){
        return (int)LimelightHelper.getCurrentPipelineIndex(limelightName);
    }

    public String getJSONDump() {
        return LimelightHelper.getJSONDump(limelightName);
    }

    public void checkForAprilTagUpdates(SwerveDrivePoseEstimator odometry) {
        int tagsSeen = LimelightHelper.getNumberOfAprilTagsSeen(limelightName);
        if (tagsSeen > 1 && this.getBotpose().relativeTo(odometry.getEstimatedPosition()).getTranslation().getNorm() < 0.5) {
            odometry.addVisionMeasurement(this.getBotpose(), Timer.getFPGATimestamp());
        }
    }

    public void forceAprilTagLocalization(SwerveDrivePoseEstimator odometry){
        if(getTv()){
            odometry.addVisionMeasurement(this.getBotpose(), Timer.getFPGATimestamp());
        }
    }

    // public Translation2d getCurrentAprilTag() { // gets the april tag the limelight is currently seeing
    //     return LimelightHelper.getAprilTagCoordinates((int) getTargetID()); // this isn't the closest, it's just the one we're seeing
    // }

    public Translation2d getAprilTagCoordinates(int tagNumber){
        return LimelightHelper.getAprilTagCoordinates(tagNumber);
    }

    public int getClosestColumn(Translation2d pose, boolean isCube){
        return LimelightHelper.getClosestColumn(pose, isCube);
    }    

    public boolean atSpeaker() {
        if (hasTarget()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                setPipeline(LimelightConstants.kRedSpeakerPipeline);
                return getTargetID() == LimelightConstants.kIDSpeakerRed;
            } else {
                setPipeline(LimelightConstants.kBlueSpeakerPipeline);
                return getTargetID() == LimelightConstants.kIDSpeakerBlue;
            }
        }
        return false;
    }


}