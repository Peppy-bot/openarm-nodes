# OpenArm Isaac Sim 6.0.1 with Commander and Scence/Object Loaders — Quick Start

# 1. Clone the required repositories

Create a workspace:

```sh
mkdir -p ~/ws
cd ~/ws
```

Clone:

```sh
git clone https://github.com/Peppy-bot/contracts-hub.git
git clone https://github.com/Peppy-bot/nodes-hub.git
git clone https://github.com/Peppy-bot/openarm-nodes.git
git clone https://github.com/Peppy-bot/launchers-hub.git
```

For the current Isaac Sim 6.0.1 development branch:

```sh
cd ~/ws/openarm-nodes
git checkout feature/isaacsim-6.0.1-clean
```

Use the corresponding launcher configuration from `launchers-hub`.

---

# 2. Register the repositories with Peppy

```sh
peppy repo add ~/ws/contracts-hub
peppy repo add ~/ws/openarm-nodes
peppy repo add ~/ws/nodes-hub
peppy repo refresh
```

Check:

```sh
peppy repo list
```

For local development, make sure the local `openarm-nodes` and `nodes-hub` repositories are the versions being used and are not shadowed by duplicate remote repository registrations.

---

# 3. Configure Isaac WebRTC

Find IP address of the machine running Isaac Sim:

```sh
hostname -I
```

Choose LAN IP that your browser can reach.

Configure it for the Apptainer runtime:

```sh
systemctl --user set-environment \
  APPTAINERENV_PEPPY_ISAAC_PUBLIC_IP=<ISAAC_HOST_IP> \
  APPTAINERENV_PEPPY_ISAAC_SIGNAL_PORT=49100 \
  APPTAINERENV_PEPPY_ISAAC_STREAM_PORT=47998
```

Restart the Peppy service:

```sh
systemctl --user restart peppy.service
```

Verify:

```sh
systemctl --user show-environment | \
grep APPTAINERENV_PEPPY_ISAAC
```

Expected:

```text
APPTAINERENV_PEPPY_ISAAC_PUBLIC_IP=<ISAAC_HOST_IP>
APPTAINERENV_PEPPY_ISAAC_SIGNAL_PORT=49100
APPTAINERENV_PEPPY_ISAAC_STREAM_PORT=47998
```

Do not hard-code a machine-specific IP into the repository in case you are on wifi network with DHCP like me.

---

# 4. Launch the complete OpenArm Isaac stack

Go to the OpenArm launcher:

```sh
cd ~/ws/launchers-hub/openarm
```

Launch:

```sh
peppy stack launch \
  ./openarm_v2_teleop_isaac_browser.json5
```
# 5. Open Isaac Sim in the browser

Open:

```text
http://<ISAAC_HOST_IP>:8210
```

Use the same IP configured in:

```text
APPTAINERENV_PEPPY_ISAAC_PUBLIC_IP
```

when Isaac is advertising a LAN IP.

WebRTC uses:

```text
8210/TCP     browser viewer
49100/TCP    signalling
47998/UDP    media stream
```

---


# 8. Load an Isaac scene

Load the standard warehouse:

```sh
python3 commander.py scene-isaac \
  Isaac/Environments/Simple_Warehouse/warehouse.usd \
  --scale 1.0
```

Load the full warehouse:

```sh
python3 commander.py scene-isaac \
  Isaac/Environments/Simple_Warehouse/full_warehouse.usd \
  --scale 1.0
```

The scene is loaded live without restarting Isaac Sim.

Runtime environments are created beneath:

```text
/World/RuntimeScene
```

---

# 6. Open OpenArm Commander

Open a second browser tab:

```text
http://localhost:8765
```

Use this interface to move the simulated OpenArm arms and grippers.

---

# 9. Add a table

Spawn a table in front of the robot:

```sh
python3 commander.py spawn-isaac \
  table \
  Isaac/Props/Mounts/SeattleLabTable/table_instanceable.usd \
  0.75 0.0 0.2 \
  --scale 0.5 \
  --physics none
```
---

# 10. Add manipulation objects

## Cracker box

```sh
python3 commander.py spawn-isaac \
  cracker_box \
  Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd \
  0.70 0.0 0.90 \
  --scale 0.5 \
  --physics none
```

## Power drill

```sh
python3 commander.py spawn-isaac \
  drill \
  Isaac/Props/YCB/Axis_Aligned/035_power_drill.usd \
  0.70 0.20 0.90 \
  --scale 1.0 \
  --physics none
```

Runtime objects are created beneath:

```text
/World/RuntimeObjects/<name>
```

---

# 11. Move objects

Move the cracker box:

```sh
python3 commander.py move cracker_box 0.65 -0.10 0.90
```

Move the drill:

```sh
python3 commander.py move drill 0.70 0.15 0.90
```

This lets you assemble the workcell interactively while Isaac remains running.

---

# 12. Enable physics when required

Runtime objects support:

```text
none
static
dynamic
```

Use:

```text
--physics none
```

for environments, complex USDs, conveyors, articulated assets, or initial placement tests.

Use:

```text
--physics static
```

for fixed objects such as tables, shelves, walls, and fixtures.

Use:

```text
--physics dynamic
```

for objects that OpenArm should push, grasp, lift, move, or drop.

Example:

```sh
python3 commander.py remove cracker_box
```

Respawn it slightly above the support surface:

```sh
python3 commander.py spawn-isaac \
  cracker_box \
  Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd \
  0.70 0.0 0.95 \
  --scale 1.0 \
  --physics dynamic \
  --mass 0.3
```

Mass is specified in kilograms.

---

# 13. Control OpenArm

Use:

```text
http://localhost:8765
```

to control the robot while watching:

```text
http://<ISAAC_HOST_IP>:8210
```

# 16. Remove objects

```sh
python3 commander.py remove drill
python3 commander.py remove cracker_box
python3 commander.py remove table
```

---

# 17. Clear or replace the scene

Clear the current runtime scene:

```sh
python3 commander.py clear-scene
```

Load another scene:

```sh
python3 commander.py scene-isaac \
  Isaac/Environments/Simple_Warehouse/full_warehouse.usd \
  --scale 1.0
```

---



