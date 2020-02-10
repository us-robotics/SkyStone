package org.firstinspires.ftc.teamcode.api.framework

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl

object AutoTransition: Thread() {
    init { this.start() } // Start listening

    private var opMode: OpMode? = null
    private var transitionTo: String = OpModeManager.DEFAULT_OP_MODE_NAME
    private var manager: OpModeManagerImpl? = null

    override fun run() {
        try {
            while (true) {
                synchronized(this) {
                    if (opMode != null && manager?.activeOpMode != opMode) {
                        sleep(1000) // Prevent strangeness
                        manager?.initActiveOpMode(transitionTo)

                        // Reset this
                        opMode = null
                        manager = null
                        transitionTo = OpModeManager.DEFAULT_OP_MODE_NAME
                    }
                }
                sleep(100) // Don't run too quickly
            }
        } catch (e: InterruptedException) {
            Log.e("RCActivity", "AutoTransition exiting; thread interrupted")
        }
    }

    fun setup(opMode: OpMode, name: String = "TeleOp") {
        synchronized(this) {
            manager = opMode.internalOpModeServices as OpModeManagerImpl
            AutoTransition.opMode = opMode
            transitionTo = name
        }
    }
}