package org.firstinspires.ftc.teamcode.api.hardware

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.teamcode.api.DeviceNames.HUB
import org.firstinspires.ftc.teamcode.api.DeviceNames.HUB2
import org.firstinspires.ftc.teamcode.api.DeviceNames.IMU
import org.firstinspires.ftc.teamcode.api.DeviceNames.INTAKE_L
import org.firstinspires.ftc.teamcode.api.DeviceNames.INTAKE_R
import org.firstinspires.ftc.teamcode.api.DeviceNames.MOTOR_BL
import org.firstinspires.ftc.teamcode.api.DeviceNames.MOTOR_BR
import org.firstinspires.ftc.teamcode.api.DeviceNames.MOTOR_FL
import org.firstinspires.ftc.teamcode.api.DeviceNames.MOTOR_FR
import org.firstinspires.ftc.teamcode.api.DeviceNames.ODOM_H
import org.firstinspires.ftc.teamcode.api.DeviceNames.ODOM_L
import org.firstinspires.ftc.teamcode.api.DeviceNames.ODOM_R
import org.firstinspires.ftc.teamcode.api.DriveConstants.BASE_CONSTRAINTS
import org.firstinspires.ftc.teamcode.api.DriveConstants.HEADING_PID
import org.firstinspires.ftc.teamcode.api.DriveConstants.MOTOR_VELOCITY_F
import org.firstinspires.ftc.teamcode.api.DriveConstants.MOTOR_VELO_PID
import org.firstinspires.ftc.teamcode.api.DriveConstants.PID_DISABLED
import org.firstinspires.ftc.teamcode.api.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.api.DriveConstants.TRANSLATIONAL_PID
import org.firstinspires.ftc.teamcode.api.DriveConstants.WHEEL_BASE
import org.firstinspires.ftc.teamcode.api.DriveConstants.encoderTicksToInches
import org.firstinspires.ftc.teamcode.api.DriveConstants.kA
import org.firstinspires.ftc.teamcode.api.DriveConstants.kStatic
import org.firstinspires.ftc.teamcode.api.DriveConstants.kV
import org.firstinspires.ftc.teamcode.api.OdometryConstants.ODOM_H_POS
import org.firstinspires.ftc.teamcode.api.OdometryConstants.ODOM_L_POS
import org.firstinspires.ftc.teamcode.api.OdometryConstants.ODOM_R_POS
import org.firstinspires.ftc.teamcode.api.OdometryConstants.odomTicksToInches
import org.firstinspires.ftc.teamcode.api.ProgramConstants.DEBUG
import org.firstinspires.ftc.teamcode.api.framework.*
import org.openftc.revextensions2.ExpansionHubEx
import kotlin.math.PI

class Drivetrain(hardwareMap: HardwareMap, telemetry: Telemetry?) :
        MecanumDrive(kV, kA, kStatic, WHEEL_BASE, TRACK_WIDTH), Updatable {

    private enum class Mode { TELEOP, IDLE, TURN, FOLLOW_TRAJECTORY }

    private var mode = Mode.IDLE
    override val busy: Boolean
        get() = (mode != Mode.IDLE)

    override fun update() {
        updatePoseEstimate()
        when (mode) {
            Mode.TELEOP -> noop() // Do nothing and allow something external to move the robot
            Mode.IDLE -> setDriveSignal(DriveSignal()) // Do nothing and don't move
            Mode.TURN -> updateTurn() // Continue turning
            Mode.FOLLOW_TRAJECTORY -> updateTrajectory() // Continue following trajectory
        }
    }

    fun initTeleop() {
        mode = Mode.TELEOP
        //autoTelem.unregister() // We don't need the auto telemetry in TeleOp
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Hardware
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private val hub = hardwareMap[ExpansionHubEx::class.java, HUB]
    private val hub2 = hardwareMap[ExpansionHubEx::class.java, HUB2]

    private val imu = hardwareMap[BNO055IMU::class.java, IMU].apply {
        val params = BNO055IMU.Parameters()
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS
        initialize(params)
        remapAxes(AxesOrder.XYZ, AxesSigns.NPN)
    }

    private val leftFront = (hardwareMap.dcMotor[MOTOR_FL] as DcMotorEx).init(reverse = true)
    private val leftRear = (hardwareMap.dcMotor[MOTOR_BL] as DcMotorEx).init(reverse = true)
    private val rightRear = (hardwareMap.dcMotor[MOTOR_BR] as DcMotorEx).init()
    private val rightFront = (hardwareMap.dcMotor[MOTOR_FR] as DcMotorEx).init()
    private val motors = listOf(leftFront, leftRear, rightRear, rightFront)

    init {
        LynxModuleUtil.ensureMinFWVersion(hardwareMap)
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Localization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private val odomMotors = listOf(
            hardwareMap.dcMotor[ODOM_L], hardwareMap.dcMotor[ODOM_R],
            hardwareMap.dcMotor[ODOM_H]
    )

    init {
        localizer = object : ThreeTrackingWheelLocalizer(listOf(
                ODOM_L_POS, ODOM_R_POS, ODOM_H_POS
        )) {
            override fun getWheelPositions(): List<Double> {
                val bulkData = hub2.bulkInputData
                return odomMotors.map { bulkData.getMotorCurrentPosition(it) }.map { odomTicksToInches(it) }
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Telemetry
    ////////////////////////////////////////////////////////////////////////////////////////////////

    init { // Register telemetry relevent for both TeleOp and Auto
        TelemetrySource {

            put("heading", poseEstimate.heading)

            if (DEBUG) { // This data isn't useful when debug is turned off
                put("imu1", imu.angularOrientation.firstAngle)
                put("imu2", imu.angularOrientation.secondAngle)
                put("imu3", imu.angularOrientation.thirdAngle)
            }

            // Wheel velocities
            if (DEBUG) { // Adds an extra bulk read so let's avoid it by default
                val velocs = getWheelVelocities()
                put("flVelo", velocs[0])
                put("blVelo", velocs[1])
                put("brVelo", velocs[2])
                put("frVelo", velocs[3])
            }

            // Odometry
            if (DEBUG) { // Adds extra bulk reads so let's avoid it by default
                put("odomLeftPos", odomMotors[0].currentPosition)
                put("odomRightPos", odomMotors[1].currentPosition)
                put("odomHorizPos", odomMotors[2].currentPosition)
            }
        }.register(telemetry)
    }

    val autoTelem = TelemetrySource {
        put("mode", mode)

        put("x", poseEstimate.x)
        put("y", poseEstimate.y)

        val err = when (mode) {
            Mode.IDLE, Mode.TELEOP -> Pose2d()
            Mode.TURN -> Pose2d(0.0, 0.0, turnController.lastError)
            Mode.FOLLOW_TRAJECTORY -> follower.lastError
        }
        put("xErr", err.x)
        put("yErr", err.y)
        put("headingErr", err.heading)

        // Draw the robot on the field
        val field = fieldOverlay()
        field.setFill("black")
        field.fillCircle(poseEstimate.x, poseEstimate.y, 10.0)
        field.setFill("blue")
        field.fillCircle(poseEstimate.x, poseEstimate.y, 2.0)
        field.setStroke("blue")
        field.setStrokeWidth(1)
        val vec = poseEstimate.headingVec() * 10.0
        field.strokeLine(poseEstimate.x, poseEstimate.y,
                poseEstimate.x + vec.x, poseEstimate.y + vec.y)
        // TODO: Draw path in FOLLOW_TRAJECTORY
    }.register(telemetry)

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Trajectory
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private val constraints = MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH, WHEEL_BASE)
    private val follower = HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID)

    /**
     * Create a trajectory builder for this drivetrain.
     * This should be used to generate trajectories for followTrajectory
     *
     * @see TrajectoryBuilder
     * @see followTrajectory
     * @return A new TrajectoryBuilder
     */
    fun trajectoryBuilder() = TrajectoryBuilder(startPose = poseEstimate, constraints = constraints)

    /**
     * Follow a given trajectory
     *
     * @param traj The trajectory to follow
     *
     * @see drive
     * @see trajectoryBuilder
     */
    fun followTrajectory(traj: Trajectory) {
        if (mode != Mode.IDLE) throw ConcurrentOperationException()
        follower.followTrajectory(traj)
        mode = Mode.FOLLOW_TRAJECTORY
    }

    /**
     * A utility function to make trajectories easier to set up
     * @see followTrajectory
     */
    inline fun drive(setup: TrajectoryBuilder.() -> Unit) {
        val bld = trajectoryBuilder()
        bld.setup()
        followTrajectory(bld.build())
    }

    /**
     * Stop the currently-running task, stop the motors, and idle the robot
     */
    fun stop() {
        mode = Mode.IDLE
        setMotorPowers(0.0, 0.0, 0.0, 0.0)
    }

    private fun updateTrajectory() { // Called by the update loop when mode = FOLLOW_TRAJECTORY
        setDriveSignal(follower.update(poseEstimate))
        if (!follower.isFollowing()) {
            mode = Mode.IDLE
            setDriveSignal(DriveSignal())
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Convenience functions
    ////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Drive in a spline
     *
     * Angle mode:
     * - If heading is null and tan is false, robot stays @ constant heading
     * - If tan is true or reverse is true, robot is tangential to path
     * - If heading != null and tan is false, robot will rotate as it moves
     *
     * @param x Target X position. Current if null
     * @param y Target Y position. Current if null
     * @param heading The heading that the robot should end up facing. Current if null
     * @param approach The angle to approach the destination point from
     * @param tan Force tangential heading
     * @see drive
     */
    fun driveTo(x: Double? = null, y: Double? = null, heading: Double? = null,
                approach: Double? = null, tan: Boolean = false, linear: Boolean = false) = drive {
        driveTo(x, y, heading, approach, tan, linear)
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Turning
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private val clock = NanoClock.system()
    private val turnController = PIDFController(HEADING_PID).apply {
        setInputBounds(0.0, 2 * PI)
    }
    private var turnStart = 0.0
    private var turnProfile: MotionProfile? = null

    /**
     * Turn from the current heading by `angle`
     *
     * @param angle Angle to turn by
     * @param maxVel Maximum speed to turn at
     * @see turn
     */
    fun relTurn(angle: Double, maxVel: Double = constraints.maxAngVel) =
            turn(poseEstimate.heading + angle, maxVel)

    /**
     * Turn the robot from the current heading to `newHeading`
     *
     * @param newHeading Target heading
     * @param maxVel Maximum speed to turn at
     * @see relTurn
     */
    fun turn(newHeading: Double, maxVel: Double = constraints.maxAngVel) {
        if (mode != Mode.IDLE) throw ConcurrentOperationException()
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                start = MotionState(poseEstimate.heading, 0.0, 0.0, 0.0),
                goal = MotionState(newHeading, 0.0, 0.0, 0.0),
                maxVel = maxVel,
                maxAccel = constraints.maxAngAccel,
                maxJerk = constraints.maxAngJerk
        )
        turnStart = clock.seconds()
        mode = Mode.TURN
    }

    private fun updateTurn() { // Called by the update loop when mode = TURN
        val t = clock.seconds() - turnStart

        val targetState = turnProfile!![t]

        turnController.targetPosition = targetState.x

        val targetOmega = targetState.v
        val targetAlpha = targetState.a
        val correction = turnController.update(poseEstimate.heading, targetOmega)

        setDriveSignal(DriveSignal(
                Pose2d(0.0, 0.0, targetOmega + correction),
                Pose2d(0.0, 0.0, targetAlpha)
        ))

        if (t >= turnProfile!!.duration()) {
            mode = Mode.IDLE
            setDriveSignal(DriveSignal())
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Providing data to roadrunner
    ////////////////////////////////////////////////////////////////////////////////////////////////

    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()

    override fun getWheelPositions(): List<Double> {
        val bulkData = hub.bulkInputData!!
        return motors.map {
            bulkData.getMotorCurrentPosition(it)
        }.map { encoderTicksToInches(it) }
    }

    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        leftFront.power = frontLeft
        leftRear.power = rearLeft
        rightRear.power = rearRight
        rightFront.power = frontRight
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Tuning
    // Used in DriveVelocityPIDTuner
    ////////////////////////////////////////////////////////////////////////////////////////////////

    init { // Import existing tuning settings if configured
        if (MOTOR_VELO_PID != PID_DISABLED)
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID)
    }

    private var lastWheelPositions: List<Double>? = null
    private var lastTimestamp: Double = 0.0

    fun getPIDCoefficients(runMode: DcMotor.RunMode): PIDCoefficients {
        val coeff = leftFront.getPIDFCoefficients(runMode)
        return PIDCoefficients(coeff.p, coeff.i, coeff.d)
    }

    fun setPIDCoefficients(runMode: DcMotor.RunMode, coeff: PIDCoefficients) {
        motors.forEach {
            it.setPIDFCoefficients(runMode, PIDFCoefficients(
                    coeff.kP, coeff.kI, coeff.kD, MOTOR_VELOCITY_F
            ))
        }
    }

    fun getWheelVelocities(): List<Double> {
        val positions = getWheelPositions()
        val currentTimestamp = clock.seconds()

        val velocities = MutableList(positions.size) { 0.0 }
        if (lastWheelPositions != null) {
            val dt = currentTimestamp - lastTimestamp
            for (i in velocities.indices)
                velocities[i] = (positions[i] - lastWheelPositions!![i]) / dt
        } else {
            for (i in velocities.indices)
                velocities[i] = 0.0
        }

        lastTimestamp = currentTimestamp
        lastWheelPositions = positions
        return velocities
    }
}