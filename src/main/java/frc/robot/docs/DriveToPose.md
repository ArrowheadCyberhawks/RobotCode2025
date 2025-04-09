# DriveToPose
### **Purpose**
It moves the robot to a **target pose (`Pose2d`)** using:

- A **ProfiledPIDController** (`driveController`) for translational movement.
- A **PIDController** (`headingController`) for rotation.

---

### **Variables**
#### **Subsystem and Pose Targeting**
```java
private final SwerveSubsystem swerveSubsystem;
private final Supplier<Pose2d> targetPose;
```
- **`swerveSubsystem`** → The swerve drive system being controlled.
- **`targetPose`** → The desired position and orientation the robot should move toward.

#### **PID Controllers**
```java
  private final ProfiledPIDController driveController =  new ProfiledPIDController(Constants.PID.Auto.kPTranslation, Constants.PID.Auto.kITranslation, Constants.PID.Auto.kDTranslation, new Constraints(3, 1));
  private final PIDController headingController = new PIDController(Constants.PID.Auto.kThetaPIDConstants.kP, Constants.PID.Auto.kThetaPIDConstants.kI, Constants.PID.Auto.kThetaPIDConstants.kD);


```
- **`driveController`**:
  - Controls the **distance** to the target using a **trapezoidal motion profile**.
  - Uses **Auto PID constants** for gains (these can be tuned in the Constants class).
  - Has constraints: **max speed = 3 m/s, max acceleration = 1 m/s²**.

- **`headingController`**:
  - Controls the **rotation** of the robot to match the target pose.

#### **Tracking Variables**
```java
private double driveErrorAbs;
private Translation2d lastSetpointTranslation;
```
- **`driveErrorAbs`** → Tracks the absolute error in distance to the target.
- **`lastSetpointTranslation`** → Stores the last commanded translation setpoint.

#### **Optional Overrides**
```java
private Optional<DoubleSupplier> yOverride = Optional.empty();
private Optional<Supplier<Translation2d>> translationOverride = Optional.empty();
```
- **`yOverride`** → Allows an external command to override the robot’s Y-axis velocity.
- **`translationOverride`** → Allows an external command to override the robot’s movement direction.

---

### **Constructors**
The class provides multiple constructors to allow flexibility in how the command is used.

#### **Basic Constructor**
```java
public DriveToPose(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetPose) {
    this.swerveSubsystem = swerveSubsystem;
    this.targetPose = targetPose;
    addRequirements(swerveSubsystem);
    setName("DriveToPose");
}
```
- Stores the swerve subsystem and the target pose.
- Calls `addRequirements(swerveSubsystem)`, ensuring only one command controls the drivetrain at a time.

#### **Constructors with Overrides**
- These constructors add optional **Y-axis movement control** or **custom translations**.
```java
public DriveToPose(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetPose, DoubleSupplier yOverride) {
    this(swerveSubsystem, targetPose);
    this.yOverride = Optional.of(yOverride);
}

public DriveToPose(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetPose,
    Supplier<Translation2d> translationOverride) {
    this(swerveSubsystem, targetPose);
    this.translationOverride = Optional.of(translationOverride);
}

public DriveToPose(SwerveSubsystem swerveSubsystem, Pose2d targetPose) {
    this(swerveSubsystem, () -> targetPose);
}
```
- These versions allow extra flexibility in how the command moves the robot.

---

### **`initialize()` Method**
This method runs **once** when the command starts.

```java
@Override
public void initialize() {
    Pose2d currentPose = swerveSubsystem.getPose();
```
- **Gets the current position (`currentPose`) of the robot.**

```java
    driveController.reset(
        currentPose.getTranslation().getDistance(targetPose.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(swerveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond,
                swerveSubsystem.getRobotRelativeSpeeds().vyMetersPerSecond)
                .rotateBy(
                    targetPose.get().getTranslation()
                        .minus(swerveSubsystem.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
```
- **Resets the drive controller**:
  - Initializes it with the **current distance to the target**.
  - Uses the robot's current velocity to set a reasonable starting velocity.
  - Ensures a **smooth start** to the movement.

```java
    headingController.reset();
    lastSetpointTranslation = currentPose.getTranslation();
}
```
- **Resets the heading controller** to ensure the robot correctly aligns with the target.
- **Stores the initial setpoint translation** for tracking.

---

### **`execute()` Method**
- This method is called repeatedly while the command is scheduled.

#### **Get Current and Target Poses**
```java
Pose2d currentPose = swerveSubsystem.getPose();
Pose2d targetPose = this.targetPose.get();
```
- Reads **current and target positions**.

#### **Calculate Scaling Factor**
```java
double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
double ffScaler = MathUtil.clamp((currentDistance - 0.2) / (0.8 - 0.2), 0.0, 1.0);
driveErrorAbs = currentDistance;
```
- **`ffScaler`** scales the movement **force**:
  - Limits speed when close to the target.
  - Ensures **smooth stopping**.

#### **Calculate Drive Velocity**
```java
double driveVelocity = driveController.getSetpoint().velocity * ffScaler
    + driveController.calculate(driveErrorAbs, 0.0);
```
- Uses the **PID controller** to calculate the forward movement speed.

#### **Calculate Heading Velocity**
```java
double headingVelocity = headingController.calculate(
    currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
```
- Uses the **PID controller** to calculate the rotational speed.

#### **Determine Final Velocity**
```java
Translation2d velocity;
if (translationOverride.isPresent() && translationOverride.get().get().getNorm() > 0.5) {
    velocity = translationOverride.get().get();
} else {
    velocity = new Pose2d(
        new Translation2d(),
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
        .transformBy(GeomUtil.translationToTransform(driveVelocity, 0.0))
        .getTranslation();
}
```
- If a **custom translation override** is provided, use it.
- Otherwise, compute the movement direction based on the target.

#### **Send Commands to Swerve Drive**
```java
swerveSubsystem.driveFieldOriented(new ChassisSpeeds(velocity.getX(),
    yOverride.isPresent() ? yOverride.get().getAsDouble() : velocity.getY(), headingVelocity));
```
Inputs all of the next velocities for the swerve drive into the driving function.

#### **Logging**
```java
Logger.recordOutput("DriveToPose/MeasuredDistance", currentDistance);
Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
Logger.recordOutput("DriveToPose/MeasuredHeading", currentPose.getRotation().getDegrees());
Logger.recordOutput("DriveToPose/SetpointHeading", targetPose.getRotation().getDegrees());
Logger.recordOutput("DriveToPose/TargetPose", targetPose);
```
Data for debugging.

---

### `atGoal()`
```java
public boolean atGoal() {
    return driveController.atGoal() && headingController.atSetpoint();
}
```
Returns **`true`** when the robot reaches the target position and rotation.