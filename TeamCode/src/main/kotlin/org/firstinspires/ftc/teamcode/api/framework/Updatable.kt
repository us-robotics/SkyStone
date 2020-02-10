package org.firstinspires.ftc.teamcode.api.framework

import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.api.DriveConstants.SERVO_MAX_SWEEP_TIME
import java.util.*

/**
 * Standardizes components in a way that they can be manipulated by classes like HardwareComposition
 *
 * @see HardwareComposition
 */
interface Updatable {
    val busy: Boolean

    fun update()
}


class ConcurrentOperationException : RuntimeException("Cannot run these tasks concurrently")

/**
 * An Updatable that composes other Updatables to let them work together
 */
open class UpdatableComposition : Updatable {
    val components = mutableListOf<Updatable>()

    override val busy: Boolean
        get() = (components.sumBy { if (it.busy) 1 else 0 } > 0)

    override fun update() = components.forEach { it.update() }

    /**
     * Registers an updatable with `this`
     */
    protected fun <T : Updatable> T.register(): T {
        components.add(this)
        return this
    }
}

/**
 * A special UpdatableComposition with some extra utility functions
 */
open class HardwareComposition(val opMode: OpMode) : UpdatableComposition() {

    /**
     * Registers a special temporary Sequence that allows multiple steps to happen in order
     * in the context of an Updatable
     */
    fun sequence(exec: Sequence.() -> Unit) {
        val seq = Sequence(this).register()
        seq.exec()
    }

    /**
     * Wait for some arbitrary boolean condition to become true
     */
    fun waitFor(condition: HardwareComposition.() -> Boolean) {
        // TODO: Check if this doesn't crash
        while (
                !Thread.currentThread().isInterrupted && !condition()
        ) update()
    }

    /**
     * Waits for component to stop being busy. Only this or components that are registered with
     * this are allowed.
     */
    fun waitFor(component: Updatable) = when {
        component == this || component in components -> waitFor { !component.busy }
        else -> throw IllegalArgumentException("Component is not part of this HardwareComposition")
    }

    /**
     * Waits for this composition to stop being busy
     */
    fun waitForIdle() = waitFor(this)
}

/**
 * An updatable that is busy for a certain amount of time after go() is called
 * Only really useful in Sequence
 */
class TimeoutUpdateable(val seconds: Double) : Updatable {
    val clock = NanoClock.system()

    private var startTime: Double? = null

    fun go() {
        startTime = clock.seconds()
    }

    override val busy: Boolean
        get() = (startTime == null) || (clock.seconds() - startTime!!) <= seconds

    override fun update() = noop
}

/**
 * An Updatable that allows multiple other updatable steps to be run in order
 */
private typealias Step = Pair<Updatable, () -> Unit>

class Sequence(private val compose: UpdatableComposition) : Updatable {
    private val queue: Queue<Step> = LinkedList()
    private var currentStep: Step? = null

    override val busy: Boolean
        get() = true // Always busy, because the sequence removes itself when done

    fun <T : Updatable> with(component: T, setup: T.() -> Unit): Sequence {
        queue.add(component to { component.setup() })
        return this
    }

    fun sleep(seconds: Double): Sequence {
        val timeout = TimeoutUpdateable(seconds)
        with(timeout) { timeout.go() }
        return this
    }

    override fun update() {
        if (currentStep == null && queue.isNotEmpty()) {
            currentStep = queue.element()
            currentStep!!.second() // Execute the setup function
        } else if (currentStep != null) {
            currentStep!!.first.update()
            if (!currentStep!!.first.busy) currentStep = null
        } else compose.components.remove(this)
    }

}

/**
 * Updatable + Updatable = a new UpdatableComposition
 *
 * Useful in Sequence like so:
 * ```
 * sequence {
 *  with(lift) { targetPosition = TOP }
 *  with(lift + depositor) {
 *      lift.targetPosition = BOTTOM
 *      depositor.out = true
 *  }
 *  with(...) { ... }
 * }
 * ```
 */
operator fun Updatable.plus(other: Updatable): Updatable = when (this) {
    is UpdatableComposition -> {
        this.components.add(other)
        this
    }
    else -> {
        val comp = UpdatableComposition()
        comp.components.add(this)
        comp.components.add(other)
        comp
    }
}

/**
 * A wrapper around Servo so that it can be detected as busy
 */
// TODO: Different servos have different sweep times
class UpdatableServo(private val s: Servo) : Updatable, Servo by s {
    val clock = NanoClock.system()
    private var startTime = 0.0

    override val busy: Boolean
        get() = (clock.seconds() - startTime) <= SERVO_MAX_SWEEP_TIME

    override fun update() = noop

    override fun setPosition(position: Double) {
        s.position = position
        startTime = clock.seconds()
    }
}

/**
 * Converts a Servo to an UpdatableServo
 */
fun Servo.toUpdatable() = UpdatableServo(this)

/**
 * A wrapper around DcMotorEx so that it can be detected as busy
 */
class UpdatableMotor(private val m: DcMotorEx, private val useEncoder: Boolean) :
        Updatable, DcMotorEx by m {

    override fun setTargetPosition(position: Int) {
        m.targetPosition = position
        m.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    override fun setPower(power: Double) {
        m.mode = if (useEncoder) DcMotor.RunMode.RUN_USING_ENCODER else DcMotor.RunMode.RUN_WITHOUT_ENCODER
        m.power = power
    }

    override val busy: Boolean
        get() = (mode == DcMotor.RunMode.RUN_TO_POSITION) && this.isBusy

    override fun update() = noop
}

fun DcMotor.toUpdatable(useEncoder: Boolean = true) = (this as DcMotorEx).toUpdatable(useEncoder)

fun DcMotorEx.toUpdatable(useEncoder: Boolean = true) = UpdatableMotor(this, useEncoder)