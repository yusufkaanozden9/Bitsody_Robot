import time
import sys
import numpy as np
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

import config

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

G1_NUM_MOTOR = 29

Kp = [
    60, 60, 60, 100, 40, 40,
    60, 60, 60, 100, 40, 40,
    60, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40
]

Kd = [
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1
]


class G1JointIndex:
    """Motor index mapping for G1 (29 DOF)"""
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5

    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11

    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28


# ─────────────────────────────────────────────────────────────────────────────
# Gait Generator
# ─────────────────────────────────────────────────────────────────────────────

class GaitGenerator:
    """
    Generates bipedal walking trajectories using parametric sine functions.
    Phases: Double Support → Swing Right → Double Support → Swing Left
    """

    def __init__(
        self,
        gait_period: float = 1.2,
        stride_length: float = 0.2,
        stance_width: float = 0.15,
        swing_height: float = 0.15,
    ):
        self.gait_period = gait_period  # seconds for full cycle
        self.stride_length = stride_length  # meters forward per step
        self.stance_width = stance_width  # meters lateral spread
        self.swing_height = swing_height  # meters foot clearance

        # Phase durations (symmetric gait)
        self.double_support_dur = gait_period * 0.25
        self.swing_dur = gait_period * 0.25

    def get_gait_phase(self, time: float) -> dict:
        """
        Returns gait phase info at given time.

        Returns dict with:
          - phase: 'double_support' / 'swing_left' / 'swing_right'
          - phase_time: time within current phase (0 to phase_duration)
          - phase_ratio: normalized phase progress (0.0 to 1.0)
          - gait_cycle_time: time within full cycle (0 to gait_period)
        """
        cycle_time = time % self.gait_period

        phase_1_end = self.double_support_dur
        phase_2_end = phase_1_end + self.swing_dur
        phase_3_end = phase_2_end + self.double_support_dur

        if cycle_time < phase_1_end:
            phase = "double_support"
            phase_time = cycle_time
            phase_dur = self.double_support_dur
        elif cycle_time < phase_2_end:
            phase = "swing_right"
            phase_time = cycle_time - phase_1_end
            phase_dur = self.swing_dur
        elif cycle_time < phase_3_end:
            phase = "double_support"
            phase_time = cycle_time - phase_2_end
            phase_dur = self.double_support_dur
        else:
            phase = "swing_left"
            phase_time = cycle_time - phase_3_end
            phase_dur = self.swing_dur

        phase_ratio = np.clip(phase_time / phase_dur, 0.0, 1.0)

        return {
            "phase": phase,
            "phase_time": phase_time,
            "phase_ratio": phase_ratio,
            "gait_cycle_time": cycle_time,
        }

    def get_leg_targets(self, time: float) -> dict:
        """
        Returns target joint angles for both legs.

        Returns dict:
          left: { "hip_pitch": float, "knee": float, "ankle_pitch": float }
          right: { "hip_pitch": float, "knee": float, "ankle_pitch": float }
        """
        phase_info = self.get_gait_phase(time)
        phase = phase_info["phase"]
        ratio = phase_info["phase_ratio"]

        targets = {"left": {}, "right": {}}

        if phase == "swing_right":
            # Right leg swings forward, left leg is stance
            targets["right"]["hip_pitch"] = self._swing_trajectory(ratio)
            targets["right"]["knee"] = self._knee_swing(ratio)
            targets["right"]["ankle_pitch"] = -0.05 + 0.05 * np.sin(np.pi * ratio)

            targets["left"]["hip_pitch"] = -0.05  # slight lean back for balance
            targets["left"]["knee"] = 0.05
            targets["left"]["ankle_pitch"] = -0.05

        elif phase == "swing_left":
            # Left leg swings forward, right leg is stance
            targets["left"]["hip_pitch"] = self._swing_trajectory(ratio)
            targets["left"]["knee"] = self._knee_swing(ratio)
            targets["left"]["ankle_pitch"] = -0.05 + 0.05 * np.sin(np.pi * ratio)

            targets["right"]["hip_pitch"] = -0.05
            targets["right"]["knee"] = 0.05
            targets["right"]["ankle_pitch"] = -0.05

        else:  # double_support
            # Both legs support
            targets["left"]["hip_pitch"] = 0.0
            targets["left"]["knee"] = 0.1
            targets["left"]["ankle_pitch"] = -0.05

            targets["right"]["hip_pitch"] = 0.0
            targets["right"]["knee"] = 0.1
            targets["right"]["ankle_pitch"] = -0.05

        return targets

    def _swing_trajectory(self, ratio: float) -> float:
        """Hip pitch trajectory during swing (0 to peak to 0)"""
        return self.stride_length * 0.5 * np.sin(np.pi * ratio)

    def _knee_swing(self, ratio: float) -> float:
        """Knee bend during swing (peak at middle)"""
        return self.swing_height * np.sin(np.pi * ratio)


# ─────────────────────────────────────────────────────────────────────────────
# Balance Controller
# ─────────────────────────────────────────────────────────────────────────────

class BalanceController:
    """
    Simple CoM balance controller using IMU feedback.
    Adjusts waist and hip angles to keep robot upright.
    """

    def __init__(self, kp_roll: float = 20.0, kp_pitch: float = 15.0):
        self.kp_roll = kp_roll      # waist roll gain
        self.kp_pitch = kp_pitch    # waist pitch gain
        self.last_imu_roll = 0.0
        self.last_imu_pitch = 0.0

    def get_balance_correction(self, low_state) -> dict:
        """
        Returns balance correction angles.

        Returns dict with waist corrections:
          - waist_roll: correction angle
          - waist_pitch: correction angle
        """
        if low_state is None:
            return {"waist_roll": 0.0, "waist_pitch": 0.0, "hip_roll": 0.0}

        # IMU orientation in radians
        rpy = low_state.imu_state.rpy
        imu_roll = float(rpy[0])   # rotation around x-axis
        imu_pitch = float(rpy[1])  # rotation around y-axis

        # PD correction (P only for simplicity)
        waist_roll_corr = -self.kp_roll * imu_roll
        waist_pitch_corr = -self.kp_pitch * imu_pitch
        hip_roll_corr = -self.kp_roll * imu_roll * 0.5  # half on hips

        return {
            "waist_roll": np.clip(waist_roll_corr, -0.3, 0.3),
            "waist_pitch": np.clip(waist_pitch_corr, -0.1, 0.1),
            "hip_roll": np.clip(hip_roll_corr, -0.1, 0.1),
        }


# ─────────────────────────────────────────────────────────────────────────────
# Walk Controller
# ─────────────────────────────────────────────────────────────────────────────

class WalkController:
    """
    Main walking controller. Orchestrates gait generation + balance control.
    Similar architecture to g1_player.Custom class.
    """

    def __init__(
        self,
        gait_period: float = 1.2,
        stride_length: float = 0.2,
        warmup_dur: float = 5.0,
    ):
        self.gait_gen = GaitGenerator(
            gait_period=gait_period,
            stride_length=stride_length,
        )
        self.balance_ctrl = BalanceController()

        self.time_ = 0.0
        self.control_dt_ = 0.002  # 2ms
        self.warmup_dur_ = warmup_dur
        self.mode_pr_ = 0  # PR mode
        self.mode_machine_ = 0

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.update_mode_machine_ = False
        self.crc = CRC()

        self.counter_ = 0
        self.phase_log_interval = int(0.5 / self.control_dt_)  # log every 0.5s

    def Init(self):
        """Initialize DDS publishers/subscribers"""
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        """Start control thread"""
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="walk_control"
        )

        while not self.update_mode_machine_:
            time.sleep(0.1)

        print("Robot ready. Walking starts in 5 seconds...")
        print(f"  ► Warmup: 0-{self.warmup_dur_:.0f}s (zero posture)")
        print(f"  ► Walking: {self.warmup_dur_:.0f}s+ (continuous gait)")

        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        """Receive robot state feedback"""
        self.low_state = msg

        if not self.update_mode_machine_:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True

        self.counter_ += 1
        if self.counter_ % self.phase_log_interval == 0:
            self.counter_ = 0
            rpy = self.low_state.imu_state.rpy
            elapsed = self.time_ - self.warmup_dur_
            if elapsed > 0:
                phase_info = self.gait_gen.get_gait_phase(elapsed)
                print(
                    f"  [Walk] Phase: {phase_info['phase']:15s} "
                    f"Ratio: {phase_info['phase_ratio']:.2f} "
                    f"IMU(rpy): ({rpy[0]:6.3f}, {rpy[1]:6.3f}, {rpy[2]:6.3f})"
                )

    def LowCmdWrite(self):
        """Main control loop (runs at 2ms intervals)"""
        self.time_ += self.control_dt_

        self.low_cmd.mode_pr = self.mode_pr_
        self.low_cmd.mode_machine = self.mode_machine_

        if self.time_ < self.warmup_dur_:
            # Stage 1: Warmup - gradually go to zero posture
            self._warmup_stage()
        else:
            # Stage 2: Walking
            self._walking_stage()

        # Send command
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

    def _warmup_stage(self):
        """Gradually transition to zero posture"""
        ratio = np.clip(self.time_ / self.warmup_dur_, 0.0, 1.0)

        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 1
            self.low_cmd.motor_cmd[i].tau = 0.0
            # Gradually move from current position to zero
            self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 50
            self.low_cmd.motor_cmd[i].kd = Kd[i]

    def _walking_stage(self):
        """Generate walking motion"""
        elapsed = self.time_ - self.warmup_dur_

        # Get leg targets from gait generator
        leg_targets = self.gait_gen.get_leg_targets(elapsed)

        # Get balance corrections from IMU
        balance = self.balance_ctrl.get_balance_correction(self.low_state)

        # Fill motor commands
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 1
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = Kp[i]
            self.low_cmd.motor_cmd[i].kd = Kd[i]

            # Default: neutral position
            self.low_cmd.motor_cmd[i].q = 0.0

        # Left leg targets
        self.low_cmd.motor_cmd[G1JointIndex.LeftHipPitch].q = leg_targets["left"]["hip_pitch"]
        self.low_cmd.motor_cmd[G1JointIndex.LeftKnee].q = leg_targets["left"]["knee"]
        self.low_cmd.motor_cmd[G1JointIndex.LeftAnklePitch].q = leg_targets["left"]["ankle_pitch"]

        # Right leg targets
        self.low_cmd.motor_cmd[G1JointIndex.RightHipPitch].q = leg_targets["right"]["hip_pitch"]
        self.low_cmd.motor_cmd[G1JointIndex.RightKnee].q = leg_targets["right"]["knee"]
        self.low_cmd.motor_cmd[G1JointIndex.RightAnklePitch].q = leg_targets["right"]["ankle_pitch"]

        # Balance corrections
        self.low_cmd.motor_cmd[G1JointIndex.WaistRoll].q = balance["waist_roll"]
        self.low_cmd.motor_cmd[G1JointIndex.WaistPitch].q = balance["waist_pitch"]
        self.low_cmd.motor_cmd[G1JointIndex.LeftHipRoll].q = balance["hip_roll"]
        self.low_cmd.motor_cmd[G1JointIndex.RightHipRoll].q = -balance["hip_roll"]

        # Counter-swing for arms (opposite to leg motion)
        phase_info = self.gait_gen.get_gait_phase(elapsed)
        arm_swing = 0.3 * np.sin(2 * np.pi * elapsed / self.gait_gen.gait_period)

        if phase_info["phase"] == "swing_right":
            # Right leg swinging → left arm forward
            self.low_cmd.motor_cmd[G1JointIndex.LeftShoulderPitch].q = arm_swing
            self.low_cmd.motor_cmd[G1JointIndex.RightShoulderPitch].q = -arm_swing
        elif phase_info["phase"] == "swing_left":
            # Left leg swinging → right arm forward
            self.low_cmd.motor_cmd[G1JointIndex.LeftShoulderPitch].q = -arm_swing
            self.low_cmd.motor_cmd[G1JointIndex.RightShoulderPitch].q = arm_swing


# ─────────────────────────────────────────────────────────────────────────────
# Main Entry Point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print(f"\n{'='*60}")
    print("  G1 BIPEDAL WALKING CONTROLLER")
    print(f"{'='*60}\n")

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)

    walk_controller = WalkController(
        gait_period=1.2,      # 1.2 seconds per full cycle
        stride_length=0.2,    # 0.2m forward per step
        warmup_dur=5.0,       # 5 seconds warmup
    )
    walk_controller.Init()
    walk_controller.Start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutdown...")
