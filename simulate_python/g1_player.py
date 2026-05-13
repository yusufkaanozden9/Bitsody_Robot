import time
import sys
import json
import argparse
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import numpy as np

# ─── Sabitler ────────────────────────────────────────────────────────────────

SCRIPT_DIR = Path(__file__).resolve().parent
POSITIONS_DIR = SCRIPT_DIR / "positions"

G1_NUM_MOTOR = 29

Kp = [
    60, 60, 60, 100, 40, 40,  # legs
    60, 60, 60, 100, 40, 40,  # legs
    60, 40, 40,  # waist
    40, 40, 40, 40, 40, 40, 40,  # arms
    40, 40, 40, 40, 40, 40, 40  # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,  # legs
    1, 1, 1, 2, 1, 1,  # legs
    1, 1, 1,  # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1  # arms
]


# ─── Joint index tanımları ────────────────────────────────────────────────────

class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistA = 13
    WaistPitch = 14
    WaistB = 14
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


class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints


# ─── positions JSON ↔ motor index eşleştirmesi ───────────────────────────────
#
#  positions.json joint adı          → G1 motor index (29DOF, PR mode)
#
JOINT_NAME_TO_MOTOR_INDEX: dict[str, int] = {
    "left_hip_pitch_joint": G1JointIndex.LeftHipPitch,
    "left_hip_roll_joint": G1JointIndex.LeftHipRoll,
    "left_hip_yaw_joint": G1JointIndex.LeftHipYaw,
    "left_knee_joint": G1JointIndex.LeftKnee,
    "left_ankle_pitch_joint": G1JointIndex.LeftAnklePitch,
    "left_ankle_roll_joint": G1JointIndex.LeftAnkleRoll,
    "right_hip_pitch_joint": G1JointIndex.RightHipPitch,
    "right_hip_roll_joint": G1JointIndex.RightHipRoll,
    "right_hip_yaw_joint": G1JointIndex.RightHipYaw,
    "right_knee_joint": G1JointIndex.RightKnee,
    "right_ankle_pitch_joint": G1JointIndex.RightAnklePitch,
    "right_ankle_roll_joint": G1JointIndex.RightAnkleRoll,
    "waist_yaw_joint": G1JointIndex.WaistYaw,
    "waist_roll_joint": G1JointIndex.WaistRoll,
    "waist_pitch_joint": G1JointIndex.WaistPitch,
    "left_shoulder_pitch_joint": G1JointIndex.LeftShoulderPitch,
    "left_shoulder_roll_joint": G1JointIndex.LeftShoulderRoll,
    "left_shoulder_yaw_joint": G1JointIndex.LeftShoulderYaw,
    "left_elbow_joint": G1JointIndex.LeftElbow,
    "left_wrist_roll_joint": G1JointIndex.LeftWristRoll,
    "left_wrist_pitch_joint": G1JointIndex.LeftWristPitch,
    "left_wrist_yaw_joint": G1JointIndex.LeftWristYaw,
    "right_shoulder_pitch_joint": G1JointIndex.RightShoulderPitch,
    "right_shoulder_roll_joint": G1JointIndex.RightShoulderRoll,
    "right_shoulder_yaw_joint": G1JointIndex.RightShoulderYaw,
    "right_elbow_joint": G1JointIndex.RightElbow,
    "right_wrist_roll_joint": G1JointIndex.RightWristRoll,
    "right_wrist_pitch_joint": G1JointIndex.RightWristPitch,
    "right_wrist_yaw_joint": G1JointIndex.RightWristYaw,
}


# ─── Positions dosyasını yükle ────────────────────────────────────────────────

def load_frames(filename: str) -> list[dict[int, float]]:
    """
    positions/ klasöründeki JSON dosyasını okur.
    Her frame: motor_index → hedef açı (rad) dict'i döndürür.
    Bilinmeyen joint isimleri atlanır.
    """
    path = POSITIONS_DIR / filename
    if not path.exists():
        raise FileNotFoundError(f"Dosya bulunamadı: {path}")

    with open(path, "r") as f:
        data = json.load(f)

    if not isinstance(data, list):
        data = [{"positions": data}]

    frames: list[dict[int, float]] = []
    for entry in data:
        positions: dict = entry.get("positions", entry)
        frame: dict[int, float] = {}
        for joint_name, value in positions.items():
            idx = JOINT_NAME_TO_MOTOR_INDEX.get(joint_name)
            if idx is not None:
                frame[idx] = float(value)
        frames.append(frame)

    print(f"Yüklendi: {len(frames)} frame  ←  {path}")
    return frames


# ─── Kontrol sınıfı ──────────────────────────────────────────────────────────

class Custom:
    def __init__(self, frames: list[dict[int, float]], delay_ms: int = 100):
        self.frames_ = frames
        self.delay_s_ = delay_ms / 1000.0
        self.interp_interval_s_ = 0.01  # her 10 ms'te bir aradeğer üret
        self.interp_steps_per_frame_ = (
            max(1, int(np.ceil(self.delay_s_ / self.interp_interval_s_)))
            if self.delay_s_ > 0
            else 1
        )

        self.time_ = 0.0
        self.control_dt_ = 0.002  # 2 ms
        self.warmup_dur_ = 5.0  # ilk 3 saniye zero posture
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.update_mode_machine_ = False
        self.crc = CRC()

        # Playback durumu
        self._last_frame_idx = -1  # son uygulanan frame index'i (log için)
        self._playback_done = False

    def Init(self):
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while not self.update_mode_machine_:
            time.sleep(0.1)

        print("Mod bilgisi alındı. Kontrol başlatılıyor...")
        print(f"  ► İlk {self.warmup_dur_:.0f} saniye: zero posture (ısınma)")
        print(f"  ► Ardından {len(self.frames_)} frame oynatılacak "
              f"({self.delay_s_ * 1000:.0f} ms aralıkla)")

        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if not self.update_mode_machine_:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True

        self.counter_ += 1
        if self.counter_ % 500 == 0:
            self.counter_ = 0
            print(f"  IMU rpy: {self.low_state.imu_state.rpy}")

    def _get_interpolated_frame(self, elapsed: float) -> tuple[int, dict[int, float], dict[int, float], float]:
        if len(self.frames_) == 1 or self.delay_s_ <= 0:
            frame = self.frames_[0]
            return 0, frame, frame, 0.0

        frame_idx = min(int(elapsed / self.delay_s_), len(self.frames_) - 1)
        current_frame = self.frames_[frame_idx]

        if frame_idx >= len(self.frames_) - 1:
            return frame_idx, current_frame, current_frame, 0.0

        if self.delay_s_ < self.interp_interval_s_:
            return frame_idx, current_frame, current_frame, 0.0

        next_frame = self.frames_[frame_idx + 1]
        segment_elapsed = elapsed - (frame_idx * self.delay_s_)
        interp_tick = min(
            int(segment_elapsed / self.interp_interval_s_),
            self.interp_steps_per_frame_,
        )
        blend_ratio = min(
            (interp_tick * self.interp_interval_s_) / self.delay_s_,
            1.0,
        )
        return frame_idx, current_frame, next_frame, blend_ratio

    def LowCmdWrite(self):
        self.time_ += self.control_dt_

        self.low_cmd.mode_pr = Mode.PR
        self.low_cmd.mode_machine = self.mode_machine_

        if self.time_ < self.warmup_dur_:
            # ── Stage 1: zero posture ────────────────────────────────────────
            ratio = np.clip(self.time_ / self.warmup_dur_, 0.0, 1.0)
            for i in range(G1_NUM_MOTOR):
                self.low_cmd.motor_cmd[i].mode = 1
                self.low_cmd.motor_cmd[i].tau = 0.0
                self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q
                self.low_cmd.motor_cmd[i].dq = 0.0
                # self.low_cmd.motor_cmd[i].kp   = Kp[i]
                self.low_cmd.motor_cmd[i].kp = 50
                self.low_cmd.motor_cmd[i].kd = Kd[i]

        else:
            # ── Stage 2: positions oynat ─────────────────────────────────────
            elapsed = self.time_ - self.warmup_dur_
            frame_idx, current_frame, next_frame, blend_ratio = self._get_interpolated_frame(elapsed)

            if frame_idx != self._last_frame_idx:
                self._last_frame_idx = frame_idx
                if frame_idx < len(self.frames_) - 1:
                    print(f"  Frame [{frame_idx + 1}/{len(self.frames_)}] uygulanıyor...")
                else:
                    if not self._playback_done:
                        self._playback_done = True
                        print("  ✓ Tüm frameler oynatıldı. Son pozisyonda bekleniyor.")

            for i in range(G1_NUM_MOTOR):
                start_q = current_frame.get(i, 0.0)  # JSON'da yoksa 0.0
                end_q = next_frame.get(i, 0.0)
                target_q = start_q + ((end_q - start_q) * blend_ratio)
                self.low_cmd.motor_cmd[i].mode = 1
                self.low_cmd.motor_cmd[i].tau = 0.0
                self.low_cmd.motor_cmd[i].q = target_q
                self.low_cmd.motor_cmd[i].dq = 0.0
                # self.low_cmd.motor_cmd[i].kp = Kp[i]
                self.low_cmd.motor_cmd[i].kp = 40
                self.low_cmd.motor_cmd[i].kd = Kd[i]

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)


# ─── Giriş noktası ───────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="G1 pozisyon oynatıcı — positions/ klasöründeki JSON dosyasını oynatır"
    )
    parser.add_argument(
        "filename",
        help="positions/ klasöründeki JSON dosyasının adı (örn: positions.json)"
    )
    parser.add_argument(
        "--delay",
        type=int,
        default=100,
        metavar="MS",
        help="Frameler arası bekleme süresi (ms, varsayılan: 100)"
    )
    args = parser.parse_args()

    # Dosyayı yükle
    try:
        frames = load_frames(args.filename)
    except FileNotFoundError as e:
        print(f"HATA: {e}")
        sys.exit(1)

    print(f"\n{'=' * 50}")
    print(f"  Dosya  : {args.filename}")
    print(f"  Frame  : {len(frames)}")
    print(f"  Gecikme: {args.delay} ms")
    print(f"  DİKKAT : İlk 5 saniye boyunca hareket yok (ısınma)")
    print(f"{'=' * 50}\n")

    ChannelFactoryInitialize(1, "lo")

    custom = Custom(frames=frames, delay_ms=args.delay)
    custom.Init()
    custom.Start()

    while True:
        time.sleep(1)
