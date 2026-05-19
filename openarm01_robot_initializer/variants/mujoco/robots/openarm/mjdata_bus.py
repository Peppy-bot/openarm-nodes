#!/usr/bin/env python3
# pylint: disable=R0902,R0903,R0913,R0914,R0917,C0413
from __future__ import annotations

import json
import logging
import mmap
import os
import struct
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np

logger = logging.getLogger(__name__)

_MAGIC = b"PEPPYMJD"
_SCHEMA_VERSION = 1
_HEADER_SIZE = 64
_MAX_CONTACTS = 64
_CONTACT_SIZE = 64  # 4×u32 + 6×f64

# Header field offsets (cache-line aligned, see openarm01_nodes/CLAUDE.md
# §"Sim variant architecture — variant-as-driver" for the layout).
_OFF_MAGIC = 0
_OFF_SCHEMA = 8
_OFF_NQ = 12
_OFF_NV = 16
_OFF_NU = 20
_OFF_NBODY = 24
_OFF_NSENSORDATA = 28
_OFF_MAX_CONTACTS = 32
_OFF_STEP_COUNTER = 40
_OFF_SIM_TIME = 48


class MjDataBus:
    def __init__(self, bus_dir: Path, model) -> None:
        self._bus_dir = bus_dir
        self._bus_path = bus_dir / "mjdata.bin"
        self._meta_path = bus_dir / "mjdata.meta.json"
        self._model = model
        self._mm: Optional[mmap.mmap] = None
        self._fd: Optional[int] = None

        self._nq = int(model.nq)
        self._nv = int(model.nv)
        self._nu = int(model.nu)
        self._nbody = int(model.nbody)
        self._nsensordata = int(model.nsensordata)
        self._compute_offsets()

    def _compute_offsets(self) -> None:
        h = _HEADER_SIZE
        self._qpos_off = h
        self._qvel_off = self._qpos_off + 8 * self._nq
        self._ctrl_off = self._qvel_off + 8 * self._nv
        self._xpos_off = self._ctrl_off + 8 * self._nu
        self._xquat_off = self._xpos_off + 8 * self._nbody * 3
        self._sensordata_off = self._xquat_off + 8 * self._nbody * 4
        self._ncon_off = self._sensordata_off + 8 * self._nsensordata
        self._contacts_off = self._ncon_off + 8  # u32 ncon + 4-byte pad
        self._total_size = self._contacts_off + _CONTACT_SIZE * _MAX_CONTACTS

    def open(self) -> None:
        self._bus_dir.mkdir(parents=True, exist_ok=True)
        # Restrictive umask so the file gets 0600 even if filesystem default is permissive.
        old_umask = os.umask(0o077)
        try:
            self._fd = os.open(
                str(self._bus_path),
                os.O_RDWR | os.O_CREAT | os.O_TRUNC,
                0o600,
            )
        finally:
            os.umask(old_umask)
        os.ftruncate(self._fd, self._total_size)
        self._mm = mmap.mmap(
            self._fd, self._total_size, prot=mmap.PROT_READ | mmap.PROT_WRITE
        )
        self._write_header()
        self._write_meta()
        logger.info(
            f"mjdata bus opened: {self._bus_path} "
            f"({self._total_size} bytes, nq={self._nq} nv={self._nv} nu={self._nu})"
        )

    def _write_header(self) -> None:
        # Pack each scalar at its declared offset — easier to audit than one big
        # struct.pack with implicit padding rules across versions.
        m = self._mm
        m[_OFF_MAGIC:_OFF_MAGIC + 8] = _MAGIC
        struct.pack_into("<I", m, _OFF_SCHEMA, _SCHEMA_VERSION)
        struct.pack_into("<I", m, _OFF_NQ, self._nq)
        struct.pack_into("<I", m, _OFF_NV, self._nv)
        struct.pack_into("<I", m, _OFF_NU, self._nu)
        struct.pack_into("<I", m, _OFF_NBODY, self._nbody)
        struct.pack_into("<I", m, _OFF_NSENSORDATA, self._nsensordata)
        struct.pack_into("<I", m, _OFF_MAX_CONTACTS, _MAX_CONTACTS)
        struct.pack_into("<Q", m, _OFF_STEP_COUNTER, 0)
        struct.pack_into("<d", m, _OFF_SIM_TIME, 0.0)

    def _write_meta(self) -> None:
        model = self._model
        joints: dict[str, dict[str, int]] = {}
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
            if not name:
                continue
            joints[name] = {
                "qpos_addr": int(model.jnt_qposadr[i]),
                "qvel_addr": int(model.jnt_dofadr[i]),
            }

        actuators: dict[str, dict[str, int]] = {}
        for i in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ""
            if name:
                actuators[name] = {"ctrl_id": i}

        bodies: dict[str, dict[str, int]] = {}
        for i in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
            if name:
                bodies[name] = {"id": i}

        sensors: dict[str, dict[str, int]] = {}
        for i in range(model.nsensor):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i) or ""
            if name:
                sensors[name] = {
                    "id": i,
                    "adr": int(model.sensor_adr[i]),
                    "dim": int(model.sensor_dim[i]),
                }

        geoms: dict[str, dict[str, int]] = {}
        for i in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
            if name:
                geoms[name] = {
                    "id": i,
                    "body_id": int(model.geom_bodyid[i]),
                }

        meta = {
            "schema_version": _SCHEMA_VERSION,
            "bus_path": str(self._bus_path),
            "header_size": _HEADER_SIZE,
            "model_name": "openarm",
            "dimensions": {
                "nq": self._nq,
                "nv": self._nv,
                "nu": self._nu,
                "nbody": self._nbody,
                "nsensordata": self._nsensordata,
                "max_contacts": _MAX_CONTACTS,
            },
            "contact_struct": {
                "size": _CONTACT_SIZE,
                "fields": {
                    "body1_id": {"offset": 0,  "dtype": "u32"},
                    "body2_id": {"offset": 4,  "dtype": "u32"},
                    "geom1_id": {"offset": 8,  "dtype": "u32"},
                    "geom2_id": {"offset": 12, "dtype": "u32"},
                    "pos":      {"offset": 16, "dtype": "f64", "count": 3},
                    "force":    {"offset": 40, "dtype": "f64", "count": 3},
                },
            },
            "joints": joints,
            "actuators": actuators,
            "bodies": bodies,
            "sensors": sensors,
            "geoms": geoms,
        }
        old_umask = os.umask(0o077)
        try:
            with open(self._meta_path, "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2, sort_keys=False)
            os.chmod(self._meta_path, 0o600)
        finally:
            os.umask(old_umask)
        logger.info(f"mjdata meta written: {self._meta_path}")

    def copy_ctrl_to(self, data) -> None:
        """Pull client-written ctrl from mmap into data.ctrl[]. Call before mj_step."""
        if self._nu == 0:
            return
        ctrl_view = np.frombuffer(
            self._mm, dtype=np.float64,
            count=self._nu, offset=self._ctrl_off,
        )
        data.ctrl[:] = ctrl_view

    def copy_state_from(self, data, step: int) -> None:
        """Snapshot data → mmap; step_counter written LAST as the memory barrier
        clients use to detect torn reads."""
        m = self._mm

        if self._nq:
            m[self._qpos_off:self._qpos_off + 8 * self._nq] = data.qpos.tobytes()
        if self._nv:
            m[self._qvel_off:self._qvel_off + 8 * self._nv] = data.qvel.tobytes()
        # ctrl[] is client-owned, never written by the server side.
        m[self._xpos_off:self._xpos_off + 8 * self._nbody * 3] = data.xpos.tobytes()
        m[self._xquat_off:self._xquat_off + 8 * self._nbody * 4] = data.xquat.tobytes()
        if self._nsensordata:
            m[self._sensordata_off:self._sensordata_off + 8 * self._nsensordata] = \
                data.sensordata.tobytes()

        ncon = min(int(data.ncon), _MAX_CONTACTS)
        struct.pack_into("<II", m, self._ncon_off, ncon, 0)
        force_buf = np.zeros(6, dtype=np.float64)
        for i in range(ncon):
            c = data.contact[i]
            mujoco.mj_contactForce(self._model, data, i, force_buf)
            off = self._contacts_off + i * _CONTACT_SIZE
            g0 = int(c.geom[0])
            g1 = int(c.geom[1])
            struct.pack_into(
                "<IIIIdddddd", m, off,
                int(self._model.geom_bodyid[g0]),
                int(self._model.geom_bodyid[g1]),
                g0, g1,
                float(c.pos[0]), float(c.pos[1]), float(c.pos[2]),
                float(force_buf[0]), float(force_buf[1]), float(force_buf[2]),
            )

        struct.pack_into("<d", m, _OFF_SIM_TIME, float(data.time))
        # Memory barrier: step_counter written last; clients detect inconsistent
        # reads by checking step_counter before+after their snapshot.
        struct.pack_into("<Q", m, _OFF_STEP_COUNTER, step)

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
