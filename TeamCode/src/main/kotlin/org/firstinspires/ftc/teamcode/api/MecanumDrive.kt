package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.api.Constants.BASE_CONSTRAINTS
import org.firstinspires.ftc.teamcode.api.Constants.HEADING_PID
import org.firstinspires.ftc.teamcode.api.Constants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.api.Constants.TRANSLATIONAL_PID
import org.firstinspires.ftc.teamcode.api.Constants.WHEEL_BASE
import org.firstinspires.ftc.teamcode.api.Constants.encoderTicksToInches
import org.firstinspires.ftc.teamcode.api.Constants.kA
import org.firstinspires.ftc.teamcode.api.Constants.kStatic
import org.firstinspires.ftc.teamcode.api.Constants.kV
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import kotlin.math.PI

class MecanumDrive(hardwareMap: HardwareMap) : MecanumDrive(kV, kA, kStatic, WHEEL_BASE, TRACK_WIDTH) {

    enum class Mode {
        IDLE, TURN, FOLLOW_TRAJECTORY
    }

    val busy: Boolean
        get() = (mode != Mode.IDLE)
    private var mode = Mode.IDLE

    private val turnClock = NanoClock.system()
    private val turnController = PIDFController(HEADING_PID).apply {
        setInputBounds(0.0, 2 * PI)
    }
    private var turnStart = 0.0
    private var turnProfile: MotionProfile? = null

    private val constraints = MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH)
    private val follower = HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID)


    private val hub = hardwareMap[ExpansionHubEx::class.java, "hub"]
    private val imu = hardwareMap[BNO055IMU::class.java, "imu"].apply {
        val params = BNO055IMU.Parameters()
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS
        initialize(params)
    }

    private val leftFront = hardwareMap[ExpansionHubMotor::class.java, "motorFrontLeft"].init()
    private val leftRear = hardwareMap[ExpansionHubMotor::class.java, "motorBackLeft"].init()
    private val rightRear = hardwareMap[ExpansionHubMotor::class.java, "motorBackRight"].init(reverse = true)
    private val rightFront = hardwareMap[ExpansionHubMotor::class.java, "motorFrontRight"].init(reverse = true)
    private val motors = listOf(leftFront, leftRear, rightRear, rightFront)

    init {
        LynxModuleUtil.ensureMinFWVersion(hardwareMap)
    }

    fun getPIDCoefficients(runMode: DcMotor.RunMode): PIDCoefficients {
        val coeff = leftFront.getPIDFCoefficients(runMode)
        return PIDCoefficients(coeff.p, coeff.i, coeff.d)
    }

    fun setPIDCoefficients(runMode: DcMotor.RunMode, coeff: PIDCoefficients) {
        motors.forEach {
            it.setPIDFCoefficients(runMode, PIDFCoefficients(
                    coeff.kP, coeff.kI, coeff.kD, 1.0
            ))
        }
    }

    fun trajectoryBuilder() = TrajectoryBuilder(poseEstimate, constraints)

    fun followTrajectory(traj: Trajectory) {
        follower.followTrajectory(traj)
        mode = Mode.FOLLOW_TRAJECTORY
    }

    fun followTrajectorySync(traj: Trajectory) {
        followTrajectory(traj)
        waitForIdle()
    }

    fun turn(angle: Double) {
        val heading = poseEstimate.heading
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                start = MotionState(heading, 0.0, 0.0, 0.0),
                goal = MotionState(heading + angle, 0.0, 0.0, 0.0),
                maxVel = constraints.maxAngVel,
                maxAccel = constraints.maxAngAccel,
                maxJerk = constraints.maxAngJerk
        )
        turnStart = turnClock.seconds()
        mode = Mode.TURN
    }

    fun turnSync(angle: Double) {
        turn(angle)
        waitForIdle()
    }

    private fun getLastError() = when (mode) {
        Mode.IDLE -> Pose2d()
        Mode.TURN -> Pose2d(0.0, 0.0, turnController.lastError)
        Mode.FOLLOW_TRAJECTORY -> follower.lastError
    }

    fun update() {
        updatePoseEstimate()
        when (mode) {
            Mode.IDLE -> setDriveSignal(DriveSignal()) // Do nothing
            Mode.TURN -> {
                val t = turnClock.seconds() - turnStart
                val targetState = turnProfile!!.get(t)
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
            Mode.FOLLOW_TRAJECTORY -> {
                setDriveSignal(follower.update(poseEstimate))
                if (!follower.isFollowing()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }
        }
    }

    private fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && busy) update()
    }

    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()

    override fun getWheelPositions(): List<Double> {
        val bulkData = hub.bulkInputData ?: return listOf(0.0, 0.0, 0.0, 0.0)
        return motors.map { encoderTicksToInches(bulkData.getMotorCurrentPosition(it)) }
    }

    override fun setMotorPowers(vFL: Double, vRL: Double, vRR: Double, vFR: Double) {
        leftFront.power = vFL
        leftRear.power = vRL
        rightRear.power = vRR
        leftFront.power = vFR
    }
}