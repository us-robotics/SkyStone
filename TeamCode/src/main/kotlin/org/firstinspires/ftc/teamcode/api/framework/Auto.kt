package org.firstinspires.ftc.teamcode.api.framework

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.api.Robot
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad.Mode.DEBOUNCE
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad.Mode.TOGGLE
import kotlin.math.abs

interface Auto {
    fun init(robot: Robot)

    fun run(robot: Robot)
}

abstract class MirrorableAuto(val mirror: Boolean) : Auto {
    // Returns this but negative if mirror is true
    // Used for red vs blue auto
    protected val Double.m get() = if (mirror) -this else this
    protected val Int.m get() = this.toDouble().m
}

abstract class Configurator : LinearOpMode() {
    // Default config variables
    val delay = number("Delay Before Start", delta = 0.25)
    val mirror = toggle("Red Side")

    private val gamepad = ToggleableGamepad(gamepad1,
            rbMode = DEBOUNCE, lbMode = DEBOUNCE, aMode = TOGGLE)

    abstract fun build(): Auto

    override fun runOpMode() {
        val robot = Robot(this)
        AutoTransition.setup(this, "TeleOp")

        // Configure the autonomous
        var idx = 0
        var editing = false
        val varList = variables.toList()
        while (!isStopRequested) {
            val (name, opt) = varList[idx]
            telemetry.addData("Option", name)
            telemetry.addData("Value", opt.value)

            if (!editing) when {
                gamepad.leftBumper.value -> idx--
                gamepad.rightBumper.value -> idx++
            }
            else when {
                gamepad.leftBumper.value -> opt.dec()
                gamepad.rightBumper.value -> opt.inc()
            }

            if (editing)
                telemetry.addData("State", "PRESS LB/RB TO ADJUST THE VALUE OR [A] TO CONFIRM")
            else
                telemetry.addData("State", "PRESS [A] TO EDIT THIS OPTION OR LB/RB TO SELECT ANOTHER")

            editing = gamepad.a.value
            telemetry.update()
        }
        val auto = build()

        // Init the auto and wait for start
        auto.init(robot)
        waitForStart(robot) || return

        // Delay Handling
        if (delay.value != 0.0)
            sleep(abs(delay.value * 1000).toLong())

        // Run the auto
        auto.run(robot)
        robot.stop() // Ensure the robot stops moving

        // When auto is done, init TeleOp
    }

    interface Configurable<T> {
        var value: T
        fun inc()
        fun dec()
    }

    private val variables = mutableMapOf<String, Configurable<*>>()

    protected fun toggle(name: String, default: Boolean = false): Configurable<Boolean> {
        val configurable = object : Configurable<Boolean> {
            override var value: Boolean = default
            override fun inc() {
                value = !value
            }

            override fun dec() {
                value = !value
            }
        }
        variables[name] = configurable
        return configurable
    }

    protected fun number(name: String, default: Double = 0.0, min: Double = Double.MIN_VALUE,
                         max: Double = Double.MAX_VALUE, delta: Double = 1.0): Configurable<Double> {
        val configurable = object : Configurable<Double> {
            override var value: Double = default

            override fun inc() {
                value += delta
                if (value > max) value = min
            }

            override fun dec() {
                value -= delta
                if (value < min) value = max
            }
        }
        variables[name] = configurable
        return configurable
    }
}