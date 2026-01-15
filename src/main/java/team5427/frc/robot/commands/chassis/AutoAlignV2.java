package team5427.frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import team5427.frc.robot.Main;
import team5427.frc.robot.Constants.SwerveConstants;


public class AutoAlignV2 extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Pose2d targetPose;
    private final PIDController distanceController;
    private final PIDController rotationalController;
    
    private static final double MAX_SPEED = 3.0;
    private static final double MAX_ROTATION_SPEED = Math.toRadians(180);
    
    public AutoAlignV2(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetPose = SwerveConstants.kTargetPose;

        distanceController = new PIDController(SwerveConstants.kPDistance, 0.0, SwerveConstants.kPDistance);
        rotationalController = new PIDController(SwerveConstants.kPRotation, 0.0, SwerveConstants.kPRotation);

        rotationalController.enableContinuousInput(-Math.PI, Math.PI);
        
        distanceController.setTolerance(SwerveConstants.kDistanceTolerance);
        rotationalController.setTolerance(SwerveConstants.kRotationTolerance);
        
        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void initialize() {
        distanceController.reset();
        rotationalController.reset();
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = swerveSubsystem.getPose();

        double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double angleToTarget = Math.atan2(
            targetPose.getY() - currentPose.getY(),
            targetPose.getX() - currentPose.getX()
        );

        double headingError = angleToTarget - currentPose.getRotation().getRadians();
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

        double distanceOutput = -distanceController.calculate(distanceError, 0);
        double rotationalOutput = -rotationalController.calculate(headingError, 0);

        distanceOutput = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, distanceOutput));
        rotationalOutput = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationalOutput));

        double fieldX = distanceOutput * Math.cos(angleToTarget);
        double fieldY = distanceOutput * Math.sin(angleToTarget);
        
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(fieldX, fieldY, rotationalOutput);
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds, 
            currentPose.getRotation()
        );
        
        swerveSubsystem.setInputSpeeds(robotSpeeds);
    }
    
    @Override
    public boolean isFinished() {
        return distanceController.atSetpoint() && rotationalController.atSetpoint();
    }
}