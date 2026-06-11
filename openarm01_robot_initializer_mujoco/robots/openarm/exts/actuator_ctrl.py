from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


class MujocoActuatorCtrl:
    """Resolves MJCF actuator names to ctrl indices and writes targets into
    data.ctrl[]. Used by ActuatorCtrlBridge to translate raw set_ctrl messages
    into in-process MuJoCo writes before mj_step.

    When the config entry carries per-joint MIT gains (kp/kd), setup() rewrites
    each position actuator's gain/bias so the servo torque law matches the real
    motors: tau = kp*(q_des - q) + kd*(dq_des - dq). The dq_des feedforward is
    folded into ctrl as q_des + (kd/kp)*dq_des, which is algebraically exact
    for MuJoCo's affine position actuator.
    """

    def __init__(self, model, data, params: dict | None = None) -> None:
        self._model = model
        self._data = data
        self._params = params or {}
        self._name_to_id: dict[str, int] = {}
        self._kd_over_kp: dict[str, float] = {}
        self._ready: bool = False

    def setup(self) -> bool:
        """Build the name → ctrl-id map. Indexes by both actuator name and the
        joint name the actuator drives — the joint-name alias is the cross-engine
        canonical key (matches Isaac dof_names), so components ship one payload
        for both engines.
        """
        try:
            import mujoco  # pylint: disable=E0401

            name_to_id: dict[str, int] = {}
            joint_aliases = 0
            for i in range(self._model.nu):
                actuator_name = (
                    mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                    or ""
                )
                if actuator_name:
                    name_to_id[actuator_name] = i
                if int(self._model.actuator_trntype[i]) != int(
                    mujoco.mjtTrn.mjTRN_JOINT
                ):
                    continue
                trnid = int(self._model.actuator_trnid[i, 0])
                joint_name = (
                    mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, trnid)
                    or ""
                )
                if joint_name and joint_name not in name_to_id:
                    name_to_id[joint_name] = i
                    joint_aliases += 1
            self._name_to_id = name_to_id
            self._apply_gains()
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup MujocoActuatorCtrl: {exc}")
            return False

        logger.info(
            f"MujocoActuatorCtrl ready — {self._model.nu} actuator(s) resolved"
            f" ({joint_aliases} joint-name aliases,"
            f" {len(self._kd_over_kp)} gain override(s))"
        )
        return True

    def _apply_gains(self) -> None:
        """Overwrite actuator gain/bias from config so the sim servo runs the
        real driver's MIT gains. Position actuator torque is
        gainprm[0]*ctrl + biasprm[1]*q + biasprm[2]*dq, so kp/kd map to
        (kp, -kp, -kd). Joints without configured gains keep the MJCF values.
        """
        joint_names = self._params.get("joint_names") or []
        kps = self._params.get("kp") or []
        kds = self._params.get("kd") or []
        if not (len(joint_names) == len(kps) == len(kds)) or not joint_names:
            return
        for name, kp, kd in zip(joint_names, kps, kds):
            ctrl_id = self._name_to_id.get(name)
            if ctrl_id is None:
                logger.warning(f"gain config: unknown joint '{name}' — skipped")
                continue
            self._model.actuator_gainprm[ctrl_id][0] = float(kp)
            self._model.actuator_biasprm[ctrl_id][1] = -float(kp)
            self._model.actuator_biasprm[ctrl_id][2] = -float(kd)
            self._kd_over_kp[name] = float(kd) / float(kp)

    def teardown(self) -> None:
        self._ready = False
        self._name_to_id = {}
        self._kd_over_kp = {}

    def write_targets(self, actuator_values: dict, velocity_values: dict | None = None) -> int:
        """Write each {name: value} pair into data.ctrl[ctrl_id[name]]. When a
        velocity setpoint is supplied for a gain-configured joint, it is folded
        into ctrl as q_des + (kd/kp)*dq_des. Returns the count of values
        applied. Unknown actuator names are logged and dropped so a single bad
        entry does not stop the rest of the batch."""
        if not self._ready:
            return 0
        if not isinstance(actuator_values, dict):
            logger.warning(
                f"actuator_values must be a dict, got {type(actuator_values).__name__}"
            )
            return 0
        velocities = velocity_values if isinstance(velocity_values, dict) else {}
        applied = 0
        for name, value in actuator_values.items():
            ctrl_id = self._name_to_id.get(name)
            if ctrl_id is None:
                logger.warning(f"unknown actuator '{name}' — dropped")
                continue
            try:
                ctrl = float(value)
                ratio = self._kd_over_kp.get(name)
                dq_des = velocities.get(name)
                if ratio is not None and dq_des is not None:
                    ctrl += ratio * float(dq_des)
                self._data.ctrl[ctrl_id] = ctrl
                applied += 1
            except Exception as exc:
                logger.warning(f"failed to write ctrl[{ctrl_id}] for '{name}': {exc}")
        return applied

    @property
    def is_ready(self) -> bool:
        return self._ready
