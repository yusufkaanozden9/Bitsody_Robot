import os
import time
import threading
import json
import tkinter as tk
from tkinter import ttk
from pathlib import Path


os.environ["MUJOCO_GL"] = "glfw"

import mujoco
import mujoco.viewer


SCRIPT_DIR = Path(__file__).resolve().parent
SCENE_PATH = SCRIPT_DIR / "g1_test" / "scene.xml"
POSITIONS_DIR = SCRIPT_DIR / "positions"

locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
mj_data = mujoco.MjData(mj_model)

HINGE_JOINTS = []
for joint_id in range(mj_model.njnt):
    if mj_model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
        continue

    qpos_adr = mj_model.jnt_qposadr[joint_id]
    joint_range = mj_model.jnt_range[joint_id]
    HINGE_JOINTS.append(
        {
            "id": joint_id,
            "name": mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
            "qpos_adr": qpos_adr,
            "min": float(joint_range[0]),
            "max": float(joint_range[1]),
        }
    )

selected_joint_index = 0
joint_step = 0.05



def sync_static_pose():
    mj_model.opt.gravity[:] = (0.0, 0.0, 0.0)
    mj_data.qvel[:] = 0.0
    mj_data.qacc[:] = 0.0
    mj_data.ctrl[:] = 0.0
    mj_data.qfrc_applied[:] = 0.0
    mj_data.xfrc_applied[:] = 0.0
    mujoco.mj_forward(mj_model, mj_data)


def move_selected_joint(direction: float):
    joint = HINGE_JOINTS[selected_joint_index]
    qpos_adr = joint["qpos_adr"]
    new_value = mj_data.qpos[qpos_adr] + direction * joint_step
    mj_data.qpos[qpos_adr] = max(joint["min"], min(joint["max"], new_value))
    sync_static_pose()


def select_joint(direction: int):
    global selected_joint_index
    selected_joint_index = (selected_joint_index + direction) % len(HINGE_JOINTS)


def change_joint_step(scale: float):
    global joint_step
    joint_step = min(0.5, max(0.005, joint_step * scale))


def reset_pose():
    mujoco.mj_resetData(mj_model, mj_data)
    sync_static_pose()


def zero_all_joints():
    """Tüm hinge joint'leri 0 radyana ayarlar (limitler dahilinde)."""
    for joint in HINGE_JOINTS:
        clamped = max(joint["min"], min(joint["max"], 0.0))
        mj_data.qpos[joint["qpos_adr"]] = clamped
    sync_static_pose()


def get_all_joint_positions() -> dict:
    return {
        joint["name"]: round(float(mj_data.qpos[joint["qpos_adr"]]), 6)
        for joint in HINGE_JOINTS
    }


def print_all_joint_positions():
    positions = get_all_joint_positions()
    print("\n=== Current Joint Positions ===")
    for name, value in positions.items():
        print(f"  {name}: {value:.6f} rad")
    print("================================\n")


def append_joint_positions(filepath: str) -> int:
    """Mevcut konumları positions/ klasöründeki JSON array dosyasına ekler."""
    POSITIONS_DIR.mkdir(exist_ok=True)
    path = POSITIONS_DIR / filepath
    if path.exists():
        with open(path, "r") as f:
            data = json.load(f)
        if not isinstance(data, list):
            data = [data]
    else:
        data = []

    entry = {
        "index": len(data),
        "positions": get_all_joint_positions(),
    }
    data.append(entry)

    with open(path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"[{len(data) - 1}] pozisyon eklendi → {path.resolve()}  (toplam: {len(data)} kayıt)")
    return len(data)


def load_joint_positions(filepath: str, entry_index: int = -1):
    """positions/ klasöründeki JSON array dosyasından pozisyonu yükler (-1 = son kayıt)."""
    path = POSITIONS_DIR / filepath
    if not path.exists():
        print(f"Dosya bulunamadı: {path.resolve()}")
        return 0

    with open(path, "r") as f:
        data = json.load(f)

    if not isinstance(data, list):
        data = [{"positions": data}]

    if not data:
        print("Dosya boş.")
        return 0

    idx = entry_index % len(data)
    entry = data[idx]
    positions = entry.get("positions", entry)  # eski format uyumluluğu

    name_to_joint = {j["name"]: j for j in HINGE_JOINTS}
    loaded = 0
    for name, value in positions.items():
        if name in name_to_joint:
            joint = name_to_joint[name]
            clamped = max(joint["min"], min(joint["max"], float(value)))
            mj_data.qpos[joint["qpos_adr"]] = clamped
            loaded += 1
    sync_static_pose()
    print(f"Yüklendi: kayıt [{idx}] / toplam {len(data)} — {path.resolve()}")
    return len(data)


def key_callback(keycode):
    pass  # keyboard control removed


# ─────────────────────────────────────────────
#  Tkinter GUI
# ─────────────────────────────────────────────

class JointControlGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("G1 Joint Controller")
        self.root.resizable(True, True)
        self._user_dragging = False
        self._slider_updating = False
        self._build_ui()
        self._refresh()

    # ── helpers ────────────────────────────────

    def _format_row(self, idx: int) -> str:
        joint = HINGE_JOINTS[idx]
        value = mj_data.qpos[joint["qpos_adr"]]
        marker = "▶" if idx == selected_joint_index else " "
        return f"{marker}{idx + 1:02d}. {joint['name']:<34} {value:+.3f} rad"

    def _sync_slider_to_joint(self):
        self._slider_updating = True
        joint = HINGE_JOINTS[selected_joint_index]
        self._slider_var.set(mj_data.qpos[joint["qpos_adr"]])
        self._slider_updating = False

    def _update_slider_range(self):
        joint = HINGE_JOINTS[selected_joint_index]
        self.slider.config(from_=joint["min"], to=joint["max"])
        self.sel_label.config(
            text=f"{joint['name']}   [{joint['min']:.3f} .. {joint['max']:.3f} rad]"
        )
        self._entry_var.set("")
        self._entry_status.config(text="")

    # ── build UI ───────────────────────────────

    def _build_ui(self):
        # ── Joint list ──────────────────────────
        lf = ttk.LabelFrame(self.root, text="Joints")
        lf.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        sb = ttk.Scrollbar(lf, orient=tk.VERTICAL)
        self.listbox = tk.Listbox(
            lf, font=("Courier", 10), yscrollcommand=sb.set,
            activestyle="none", height=16, selectbackground="#4a90d9"
        )
        sb.config(command=self.listbox.yview)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

        for i in range(len(HINGE_JOINTS)):
            self.listbox.insert(tk.END, self._format_row(i))
        self.listbox.selection_set(selected_joint_index)
        self.listbox.see(selected_joint_index)
        self.listbox.bind("<<ListboxSelect>>", self._on_list_select)

        # ── Selected joint control ───────────────
        cf = ttk.LabelFrame(self.root, text="Selected Joint")
        cf.pack(fill=tk.X, padx=6, pady=4)

        self.sel_label = ttk.Label(cf, text="", font=("TkDefaultFont", 10, "bold"))
        self.sel_label.pack(anchor=tk.W, padx=6, pady=(4, 0))

        self._slider_var = tk.DoubleVar()
        self.slider = tk.Scale(
            cf, variable=self._slider_var,
            from_=-3.14159, to=3.14159, resolution=0.001,
            orient=tk.HORIZONTAL, length=460,
            command=self._on_slider_move,
        )
        self.slider.pack(fill=tk.X, padx=6)
        self.slider.bind("<ButtonPress-1>",   lambda _: setattr(self, "_user_dragging", True))
        self.slider.bind("<ButtonRelease-1>", lambda _: setattr(self, "_user_dragging", False))

        # ── Manual value entry ──────────────────
        entry_row = ttk.Frame(cf)
        entry_row.pack(pady=(0, 4))
        ttk.Label(entry_row, text="Elle giriş (rad):").pack(side=tk.LEFT, padx=(6, 4))
        self._entry_var = tk.StringVar()
        self._entry = ttk.Entry(entry_row, textvariable=self._entry_var, width=12)
        self._entry.pack(side=tk.LEFT, padx=2)
        self._entry.bind("<Return>",    self._on_entry_apply)
        self._entry.bind("<KP_Enter>",  self._on_entry_apply)
        ttk.Button(entry_row, text="Uygula", command=self._on_entry_apply).pack(side=tk.LEFT, padx=4)
        self._entry_status = ttk.Label(entry_row, text="", foreground="gray")
        self._entry_status.pack(side=tk.LEFT, padx=4)

        br = ttk.Frame(cf)
        br.pack(pady=4)
        ttk.Button(br, text="◄ Önceki", width=14, command=lambda: self._gui_select(-1)).pack(side=tk.LEFT, padx=3)
        ttk.Button(br, text="−",         width=10, command=lambda: self._gui_move(-1)).pack(side=tk.LEFT, padx=3)
        ttk.Button(br, text="+",         width=10, command=lambda: self._gui_move(+1)).pack(side=tk.LEFT, padx=3)
        ttk.Button(br, text="Sonraki ►", width=14, command=lambda: self._gui_select(+1)).pack(side=tk.LEFT, padx=3)

        # ── Step size ───────────────────────────
        sf = ttk.LabelFrame(self.root, text="Step Size (rad)")
        sf.pack(fill=tk.X, padx=6, pady=4)

        sr = ttk.Frame(sf)
        sr.pack(pady=4)
        ttk.Button(sr, text="÷2", width=10, command=lambda: self._gui_change_step(0.5)).pack(side=tk.LEFT, padx=4)
        self.step_var = tk.DoubleVar(value=joint_step)
        ttk.Spinbox(
            sr, from_=0.005, to=0.5, increment=0.005,
            textvariable=self.step_var, width=9,
            command=self._on_step_changed, format="%.3f",
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(sr, text="×2", width=10, command=lambda: self._gui_change_step(2.0)).pack(side=tk.LEFT, padx=4)

        # ── Actions ─────────────────────────────
        af = ttk.LabelFrame(self.root, text="Kayıt")
        af.pack(fill=tk.X, padx=6, pady=4)

        # Dosya adı satırı
        file_row = ttk.Frame(af)
        file_row.pack(fill=tk.X, padx=6, pady=(6, 2))
        ttk.Label(file_row, text="Dosya adı:").pack(side=tk.LEFT)
        self._file_var = tk.StringVar(value="positions.json")
        ttk.Entry(file_row, textvariable=self._file_var, width=28).pack(side=tk.LEFT, padx=4)
        self._file_status = ttk.Label(file_row, text="", foreground="gray")
        self._file_status.pack(side=tk.LEFT, padx=4)

        # Index satırı (yükleme için)
        idx_row = ttk.Frame(af)
        idx_row.pack(fill=tk.X, padx=6, pady=(2, 2))
        ttk.Label(idx_row, text="Yüklenecek kayıt index (-1=son):").pack(side=tk.LEFT)
        self._load_index_var = tk.StringVar(value="-1")
        ttk.Entry(idx_row, textvariable=self._load_index_var, width=6).pack(side=tk.LEFT, padx=4)
        self._record_count_label = ttk.Label(idx_row, text="", foreground="gray")
        self._record_count_label.pack(side=tk.LEFT, padx=4)

        # Butonlar
        btn_row = ttk.Frame(af)
        btn_row.pack(pady=(2, 6))
        ttk.Button(btn_row, text="Sıfırla",    width=13, command=self._gui_reset).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Ekle",       width=13, command=self._gui_append).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Yükle",      width=13, command=self._gui_load).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Tümünü Gör", width=13, command=print_all_joint_positions).pack(side=tk.LEFT, padx=4)

        self._update_slider_range()
        self._sync_slider_to_joint()

    # ── event handlers ─────────────────────────

    def _on_entry_apply(self, _event=None):
        try:
            value = float(self._entry_var.get())
        except ValueError:
            self._entry_status.config(text="✗ Geçersiz değer", foreground="red")
            return
        joint = HINGE_JOINTS[selected_joint_index]
        clamped = max(joint["min"], min(joint["max"], value))
        with locker:
            mj_data.qpos[joint["qpos_adr"]] = clamped
            sync_static_pose()
        self._sync_slider_to_joint()
        if abs(clamped - value) > 1e-6:
            self._entry_status.config(
                text=f"⚠ Sınırlandırıldı → {clamped:.3f}", foreground="orange"
            )
        else:
            self._entry_status.config(text=f"✓ Uygulandı: {clamped:.3f} rad", foreground="green")
        self._entry_var.set(f"{clamped:.4f}")

    def _on_list_select(self, _event):
        global selected_joint_index
        sel = self.listbox.curselection()
        if not sel:
            return
        idx = int(sel[0])
        if idx == selected_joint_index:
            return
        with locker:
            selected_joint_index = idx
        self._update_slider_range()
        self._sync_slider_to_joint()

    def _on_slider_move(self, val):
        if self._slider_updating:
            return
        value = float(val)
        with locker:
            joint = HINGE_JOINTS[selected_joint_index]
            mj_data.qpos[joint["qpos_adr"]] = max(joint["min"], min(joint["max"], value))
            sync_static_pose()

    def _on_step_changed(self):
        global joint_step
        try:
            new_step = float(self.step_var.get())
            joint_step = min(0.5, max(0.005, new_step))
        except (ValueError, tk.TclError):
            pass

    def _gui_select(self, direction: int):
        global selected_joint_index
        with locker:
            selected_joint_index = (selected_joint_index + direction) % len(HINGE_JOINTS)
        self._update_slider_range()
        self._sync_slider_to_joint()

    def _gui_move(self, direction: int):
        with locker:
            joint = HINGE_JOINTS[selected_joint_index]
            qpos_adr = joint["qpos_adr"]
            new_val = mj_data.qpos[qpos_adr] + direction * joint_step
            mj_data.qpos[qpos_adr] = max(joint["min"], min(joint["max"], new_val))
            sync_static_pose()
        self._sync_slider_to_joint()

    def _gui_change_step(self, scale: float):
        global joint_step
        joint_step = min(0.5, max(0.005, joint_step * scale))
        self.step_var.set(round(joint_step, 3))

    def _gui_reset(self):
        with locker:
            reset_pose()
        self._sync_slider_to_joint()

    def _gui_zero_all(self):
        with locker:
            zero_all_joints()
        self._sync_slider_to_joint()

    def _gui_append(self):
        filepath = self._file_var.get().strip()
        if not filepath:
            self._file_status.config(text="✗ Dosya adı boş", foreground="red")
            return
        try:
            with locker:
                count = append_joint_positions(filepath)
            self._file_status.config(text=f"✓ Eklendi (toplam {count} kayıt)", foreground="green")
            self._record_count_label.config(text=f"toplam: {count}")
        except Exception as e:
            self._file_status.config(text=f"✗ Hata: {e}", foreground="red")

    def _gui_load(self):
        filepath = self._file_var.get().strip()
        if not filepath:
            self._file_status.config(text="✗ Dosya adı boş", foreground="red")
            return
        try:
            idx_str = self._load_index_var.get().strip()
            idx = int(idx_str) if idx_str else -1
        except ValueError:
            self._file_status.config(text="✗ Geçersiz index", foreground="red")
            return
        try:
            with locker:
                count = load_joint_positions(filepath, idx)
            if count:
                self._file_status.config(text=f"✓ Yüklendi (toplam {count} kayıt)", foreground="green")
                self._record_count_label.config(text=f"toplam: {count}")
            else:
                self._file_status.config(text="✗ Yüklenemedi", foreground="red")
        except Exception as e:
            self._file_status.config(text=f"✗ Hata: {e}", foreground="red")
        self._sync_slider_to_joint()

    # ── periodic refresh ───────────────────────

    def _refresh(self):
        # Update listbox rows while preserving scroll position
        yview_top = self.listbox.yview()[0]
        for i in range(len(HINGE_JOINTS)):
            new_text = self._format_row(i)
            self.listbox.delete(i)
            self.listbox.insert(i, new_text)
        self.listbox.selection_clear(0, tk.END)
        self.listbox.selection_set(selected_joint_index)
        self.listbox.yview_moveto(yview_top)

        # Sync slider only when user is not dragging
        if not self._user_dragging:
            self._sync_slider_to_joint()

        # Keep step spinbox in sync with keyboard changes
        self.step_var.set(round(joint_step, 3))

        self.root.after(100, self._refresh)


# ─────────────────────────────────────────────
#  Viewer + GUI entry point
# ─────────────────────────────────────────────

def run_viewer():
    sync_static_pose()
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data, key_callback=key_callback)
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

    # MuJoCo sync loop runs in a background thread
    def viewer_loop():
        while viewer.is_running():
            with locker:
                sync_static_pose()
                viewer.sync()
            time.sleep(1 / 60)

    vt = threading.Thread(target=viewer_loop, daemon=True)
    vt.start()

    print("MuJoCo viewer opened.")

    # Tkinter GUI runs in main thread
    root = tk.Tk()

    def on_close():
        try:
            viewer.close()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    JointControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    run_viewer()
