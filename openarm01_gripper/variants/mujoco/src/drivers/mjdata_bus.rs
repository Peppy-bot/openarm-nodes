use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{Ordering, fence};

use memmap2::{MmapMut, MmapOptions};
use serde::Deserialize;

const MAGIC: &[u8; 8] = b"PEPPYMJD";
const SCHEMA_VERSION: u32 = 1;
const HEADER_SIZE: usize = 64;
const CONTACT_SIZE: usize = 64;

// Header field byte offsets — kept in sync with robot_initializer's
// mjdata_bus.py. See openarm01_nodes/CLAUDE.md §"Sim variant architecture".
const OFF_MAGIC: usize = 0;
const OFF_SCHEMA: usize = 8;
const OFF_NQ: usize = 12;
const OFF_NV: usize = 16;
const OFF_NU: usize = 20;
const OFF_NBODY: usize = 24;
const OFF_NSENSORDATA: usize = 28;
const OFF_MAX_CONTACTS: usize = 32;
const OFF_STEP_COUNTER: usize = 40;
const OFF_SIM_TIME: usize = 48;

#[derive(Debug)]
pub enum BusError {
    Open { path: PathBuf, source: std::io::Error },
    MetaRead { path: PathBuf, source: std::io::Error },
    MetaParse { path: PathBuf, source: serde_json::Error },
    BadMagic { found: [u8; 8] },
    SchemaMismatch { found: u32, expected: u32 },
    UnknownActuator(String),
    UnknownJoint(String),
    UnknownBody(String),
    Inconsistent,
}

impl std::fmt::Display for BusError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Open { path, source } => write!(f, "open {}: {}", path.display(), source),
            Self::MetaRead { path, source } => {
                write!(f, "read meta {}: {}", path.display(), source)
            }
            Self::MetaParse { path, source } => {
                write!(f, "parse meta {}: {}", path.display(), source)
            }
            Self::BadMagic { found } => {
                write!(f, "bad magic header: {found:?}")
            }
            Self::SchemaMismatch { found, expected } => {
                write!(f, "schema {found} != expected {expected}")
            }
            Self::UnknownActuator(n) => write!(f, "unknown actuator: {n}"),
            Self::UnknownJoint(n) => write!(f, "unknown joint: {n}"),
            Self::UnknownBody(n) => write!(f, "unknown body: {n}"),
            Self::Inconsistent => write!(f, "snapshot torn (step_counter changed mid-read)"),
        }
    }
}

impl std::error::Error for BusError {}

pub type Result<T> = std::result::Result<T, BusError>;

// ---------- meta.json types ----------

#[derive(Debug, Deserialize)]
pub struct Meta {
    pub schema_version: u32,
    pub header_size: usize,
    pub dimensions: Dimensions,
    pub joints: HashMap<String, JointAddr>,
    pub actuators: HashMap<String, ActuatorId>,
    pub bodies: HashMap<String, BodyId>,
    #[serde(default)]
    pub geoms: HashMap<String, GeomEntry>,
}

#[derive(Debug, Deserialize)]
pub struct Dimensions {
    pub nq: usize,
    pub nv: usize,
    pub nu: usize,
    pub nbody: usize,
    pub nsensordata: usize,
    pub max_contacts: usize,
}

#[derive(Debug, Deserialize, Clone, Copy)]
pub struct JointAddr {
    pub qpos_addr: usize,
    pub qvel_addr: usize,
}

#[derive(Debug, Deserialize, Clone, Copy)]
pub struct ActuatorId {
    pub ctrl_id: usize,
}

#[derive(Debug, Deserialize, Clone, Copy)]
pub struct BodyId {
    pub id: usize,
}

#[derive(Debug, Deserialize, Clone, Copy)]
pub struct GeomEntry {
    pub id: usize,
    pub body_id: usize,
}

// ---------- offsets derived from meta dimensions ----------

#[derive(Debug, Clone, Copy)]
struct Offsets {
    qpos: usize,
    qvel: usize,
    ctrl: usize,
    xpos: usize,
    xquat: usize,
    ncon: usize,
    contacts: usize,
    total_size: usize,
}

impl Offsets {
    fn from_dims(d: &Dimensions, header_size: usize) -> Self {
        let qpos = header_size;
        let qvel = qpos + 8 * d.nq;
        let ctrl = qvel + 8 * d.nv;
        let xpos = ctrl + 8 * d.nu;
        let xquat = xpos + 8 * d.nbody * 3;
        let sensordata = xquat + 8 * d.nbody * 4;
        let ncon = sensordata + 8 * d.nsensordata;
        let contacts = ncon + 8; // u32 ncon + 4-byte pad
        let total_size = contacts + CONTACT_SIZE * d.max_contacts;
        Self { qpos, qvel, ctrl, xpos, xquat, ncon, contacts, total_size }
    }
}

// ---------- snapshot returned to consumers ----------

#[derive(Debug, Clone)]
pub struct Snapshot {
    pub step: u64,
    pub sim_time: f64,
    pub qpos: Vec<f64>,
    pub xpos: [f64; 3],
    pub xquat: [f64; 4],
    pub contacts: Vec<ContactSnap>,
}

#[derive(Debug, Clone)]
pub struct ContactSnap {
    pub body1_id: u32,
    pub body2_id: u32,
    pub geom1_id: u32,
    pub geom2_id: u32,
    pub pos: [f64; 3],
    pub force: [f64; 3],
}

// ---------- the bus client ----------

pub struct MjDataBus {
    pub meta: Meta,
    mmap: Mutex<MmapMut>,
    offsets: Offsets,
    max_contacts: usize,
}

impl MjDataBus {
    pub fn open(bus_dir: &Path) -> Result<Self> {
        let meta = Self::read_meta(&bus_dir.join("mjdata.meta.json"))?;
        if meta.schema_version != SCHEMA_VERSION {
            return Err(BusError::SchemaMismatch {
                found: meta.schema_version,
                expected: SCHEMA_VERSION,
            });
        }
        let bin_path = bus_dir.join("mjdata.bin");
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(&bin_path)
            .map_err(|source| BusError::Open { path: bin_path.clone(), source })?;
        // Safety: we trust robot_initializer to size mjdata.bin per the meta;
        // mmap fails fast if the file shrinks.
        let mmap = unsafe {
            MmapOptions::new()
                .map_mut(&file)
                .map_err(|source| BusError::Open { path: bin_path.clone(), source })?
        };

        let header_magic: [u8; 8] = mmap[OFF_MAGIC..OFF_MAGIC + 8]
            .try_into()
            .expect("mmap region too small for magic");
        if &header_magic != MAGIC {
            return Err(BusError::BadMagic { found: header_magic });
        }
        let schema = u32::from_le_bytes(mmap[OFF_SCHEMA..OFF_SCHEMA + 4].try_into().unwrap());
        if schema != SCHEMA_VERSION {
            return Err(BusError::SchemaMismatch { found: schema, expected: SCHEMA_VERSION });
        }

        // Cross-check binary header vs meta.json. Catches drift if
        // robot_initializer rewrites one without the other (the bus is a
        // shared-memory contract — silent mismatch corrupts every snapshot).
        let read_u32 = |off: usize| -> usize {
            u32::from_le_bytes(mmap[off..off + 4].try_into().unwrap()) as usize
        };
        let header_dims = [
            ("nq",          read_u32(OFF_NQ),          meta.dimensions.nq),
            ("nv",          read_u32(OFF_NV),          meta.dimensions.nv),
            ("nu",          read_u32(OFF_NU),          meta.dimensions.nu),
            ("nbody",       read_u32(OFF_NBODY),       meta.dimensions.nbody),
            ("nsensordata", read_u32(OFF_NSENSORDATA), meta.dimensions.nsensordata),
            ("max_contacts",read_u32(OFF_MAX_CONTACTS),meta.dimensions.max_contacts),
        ];
        for (name, header, meta_val) in header_dims {
            if header != meta_val {
                tracing::error!(
                    "mjdata header/meta mismatch: {name} header={header} meta={meta_val}"
                );
                return Err(BusError::Inconsistent);
            }
        }

        let offsets = Offsets::from_dims(&meta.dimensions, meta.header_size);
        // Catch the case where meta.json declares more space than mjdata.bin
        // actually has — otherwise the first snapshot() panics deep inside a
        // worker task with an opaque OOB.
        if mmap.len() < offsets.total_size {
            return Err(BusError::Inconsistent);
        }
        let max_contacts = meta.dimensions.max_contacts;
        Ok(Self { meta, mmap: Mutex::new(mmap), offsets, max_contacts })
    }

    fn read_meta(path: &Path) -> Result<Meta> {
        let mut f = File::open(path).map_err(|source| BusError::MetaRead {
            path: path.to_path_buf(),
            source,
        })?;
        let mut buf = String::new();
        f.read_to_string(&mut buf)
            .map_err(|source| BusError::MetaRead { path: path.to_path_buf(), source })?;
        serde_json::from_str(&buf)
            .map_err(|source| BusError::MetaParse { path: path.to_path_buf(), source })
    }

    pub fn joint_qpos_addr(&self, name: &str) -> Result<usize> {
        self.meta.joints.get(name).map(|j| j.qpos_addr)
            .ok_or_else(|| BusError::UnknownJoint(name.to_string()))
    }

    pub fn actuator_ctrl_id(&self, name: &str) -> Result<usize> {
        self.meta.actuators.get(name).map(|a| a.ctrl_id)
            .ok_or_else(|| BusError::UnknownActuator(name.to_string()))
    }

    pub fn body_id(&self, name: &str) -> Result<usize> {
        self.meta.bodies.get(name).map(|b| b.id)
            .ok_or_else(|| BusError::UnknownBody(name.to_string()))
    }

    /// Read qpos[addr_a..addr_b] and body xpos/xquat in one consistent snapshot.
    /// Retries on torn reads (step_counter changes between pre/post check).
    /// Caller passes the qpos addresses and body id this gripper cares about.
    pub fn snapshot(
        &self,
        qpos_addrs: &[usize],
        ee_body_id: usize,
        finger_geom_ids: &[u32],
    ) -> Result<Snapshot> {
        const MAX_RETRIES: usize = 8;
        for _ in 0..MAX_RETRIES {
            let m = self.mmap.lock().unwrap();
            let step_pre = u64::from_le_bytes(
                m[OFF_STEP_COUNTER..OFF_STEP_COUNTER + 8].try_into().unwrap(),
            );
            // Seqlock read pattern: Acquire fences pin payload reads inside the
            // window bounded by step_pre/step_post. Without these, the compiler
            // or CPU is free to reorder payload reads outside the window
            // (happens to work on x86_64; not portable to ARM).
            fence(Ordering::Acquire);

            let sim_time = f64::from_le_bytes(
                m[OFF_SIM_TIME..OFF_SIM_TIME + 8].try_into().unwrap(),
            );

            let qpos: Vec<f64> = qpos_addrs
                .iter()
                .map(|&a| {
                    let off = self.offsets.qpos + 8 * a;
                    f64::from_le_bytes(m[off..off + 8].try_into().unwrap())
                })
                .collect();

            let xpos_off = self.offsets.xpos + 8 * ee_body_id * 3;
            let xpos: [f64; 3] = [
                f64::from_le_bytes(m[xpos_off..xpos_off + 8].try_into().unwrap()),
                f64::from_le_bytes(m[xpos_off + 8..xpos_off + 16].try_into().unwrap()),
                f64::from_le_bytes(m[xpos_off + 16..xpos_off + 24].try_into().unwrap()),
            ];

            let xquat_off = self.offsets.xquat + 8 * ee_body_id * 4;
            let xquat: [f64; 4] = [
                f64::from_le_bytes(m[xquat_off..xquat_off + 8].try_into().unwrap()),
                f64::from_le_bytes(m[xquat_off + 8..xquat_off + 16].try_into().unwrap()),
                f64::from_le_bytes(m[xquat_off + 16..xquat_off + 24].try_into().unwrap()),
                f64::from_le_bytes(m[xquat_off + 24..xquat_off + 32].try_into().unwrap()),
            ];

            let ncon = u32::from_le_bytes(
                m[self.offsets.ncon..self.offsets.ncon + 4].try_into().unwrap(),
            ) as usize;
            let ncon = ncon.min(self.max_contacts);

            let mut contacts = Vec::new();
            for i in 0..ncon {
                let off = self.offsets.contacts + i * CONTACT_SIZE;
                let g1 = u32::from_le_bytes(m[off + 8..off + 12].try_into().unwrap());
                let g2 = u32::from_le_bytes(m[off + 12..off + 16].try_into().unwrap());
                if !finger_geom_ids.contains(&g1) && !finger_geom_ids.contains(&g2) {
                    continue;
                }
                let body1 = u32::from_le_bytes(m[off..off + 4].try_into().unwrap());
                let body2 = u32::from_le_bytes(m[off + 4..off + 8].try_into().unwrap());
                let p0 = f64::from_le_bytes(m[off + 16..off + 24].try_into().unwrap());
                let p1 = f64::from_le_bytes(m[off + 24..off + 32].try_into().unwrap());
                let p2 = f64::from_le_bytes(m[off + 32..off + 40].try_into().unwrap());
                let f0 = f64::from_le_bytes(m[off + 40..off + 48].try_into().unwrap());
                let f1 = f64::from_le_bytes(m[off + 48..off + 56].try_into().unwrap());
                let f2 = f64::from_le_bytes(m[off + 56..off + 64].try_into().unwrap());
                contacts.push(ContactSnap {
                    body1_id: body1, body2_id: body2,
                    geom1_id: g1, geom2_id: g2,
                    pos: [p0, p1, p2], force: [f0, f1, f2],
                });
            }

            fence(Ordering::Acquire);
            let step_post = u64::from_le_bytes(
                m[OFF_STEP_COUNTER..OFF_STEP_COUNTER + 8].try_into().unwrap(),
            );
            drop(m);

            if step_pre == step_post {
                return Ok(Snapshot { step: step_pre, sim_time, qpos, xpos, xquat, contacts });
            }
        }
        Err(BusError::Inconsistent)
    }

    /// Write ctrl values for the given actuator ids. Holds the mmap lock for
    /// the duration of the write — the writes are bytes, not f64-aligned, so
    /// the lock prevents inter-finger torn writes.
    pub fn write_ctrl(&self, updates: &[(usize, f64)]) {
        let mut m = self.mmap.lock().unwrap();
        for &(ctrl_id, value) in updates {
            let off = self.offsets.ctrl + 8 * ctrl_id;
            m[off..off + 8].copy_from_slice(&value.to_le_bytes());
        }
        // Release fence ensures all ctrl writes above happen-before any later
        // observation by another thread/process via the same mmap region.
        fence(Ordering::Release);
    }
}
