# Running VLM Service on Orion (NSLS-II Cluster)

## Overview

Run the VLM service on Orion's A100 GPUs and tunnel it to your local
machine so the ROS workspace can reach it at `http://localhost:8765`.

## Prerequisites

- SSH access to `orion.nsls2.bnl.gov` (direct from local machine)
- Active Slurm job on a mars node (A100 80GB GPUs)

---

## Step 1: SSH Tunnel (local machine, Terminal 2)

This sets up port forwarding through orion to the compute node.
Run this AFTER you know which mars node you landed on (Step 2).

```bash
ssh -J your_user@orion.nsls2.bnl.gov -N -L 8765:localhost:8765 your_user@marsX
```

Replace `marsX` with the actual node (e.g., `mars1`, `mars8`).

This means: local:8765 → (jump through orion) → marsX:8765

---

## Step 2: Get a GPU allocation (Orion, Terminal 1)

```bash
ssh orion.nsls2.bnl.gov

# Debug partition (30 min max — for testing)
salloc --partition=debug --qos=debug --gres=gpu:1 --cpus-per-task=8 --mem=64G --time=00:30:00

# Normal partition (up to 12 hours — for real use)
salloc --partition=normal --qos=normal --gres=gpu:1 --cpus-per-task=8 --mem=64G --time=04:00:00
```

Note which node you land on (shown in prompt, e.g. `[abondada@mars1]`).
Now go back and run the tunnel command from Step 1 using that node name.

---

## Step 3: Setup environment (on the compute node, first time only)

```bash
# Load Python 3.12 (required — model code uses Python 3.10+ syntax)
module load python/3.12.9

# Clone repo (first time only)
cd ~
git clone https://github.com/bondada-a/erobs.git -b feat/vlm-detector-integration
cd erobs

## Update  - use python3.12 for every command instead of python3
rm -rf ~/erobs/vlm_service/.venv
python3.12 -m venv ~/erobs/vlm_service/.venv

# Create venv and install deps
python3 -m venv vlm_service/.venv
source vlm_service/.venv/bin/activate
pip install --upgrade pip
pip install -r vlm_service/requirements.txt
pip install torchvision

# Download model weights (first time only, ~32 GB, fast on cluster network)
huggingface-cli download allenai/MolmoAct-7B-D-0812
```

---

## Step 4: Launch the VLM service (on the compute node)

```bash
cd ~/erobs
module load python/3.12.9
source vlm_service/.venv/bin/activate

# Full BF16 — A100 80GB has plenty of room
./vlm_service/launch.sh molmoact
```

Wait for: `Backend molmoact ready.` and `Uvicorn running on http://0.0.0.0:8765`

---

## Step 5: Verify from local machine (Terminal 3)

```bash
curl -s http://localhost:8765/health
# Expected: {"status":"ok","backend":"molmoact","loaded":true}

curl -s http://localhost:8765/info | python3 -m json.tool
```

---

## Step 6: Use from ROS

The beamline config (`src/beambot/config/cms_beamline.yaml`) should have:

```yaml
vlm:
  service_url: "http://localhost:8765"
  timeout_seconds: 15
  enabled: true
```

Then `detect_sample_vlm` in the MCP server / agent works transparently.

---

## Subsequent runs (after first-time setup)

```bash
# Terminal 1: SSH to orion, get allocation
ssh orion.nsls2.bnl.gov
salloc --partition=normal --qos=normal --gres=gpu:1 --cpus-per-task=8 --mem=64G --time=04:00:00
# Note the node name (e.g. mars1)

# On the compute node:
module load python/3.12.9
cd ~/erobs && git pull
source vlm_service/.venv/bin/activate
./vlm_service/launch.sh molmoact

# Terminal 2: Tunnel (from local machine)
ssh -J your_user@orion.nsls2.bnl.gov -N -L 8765:localhost:8765 your_user@marsX

# Terminal 3: Verify
curl -s http://localhost:8765/health
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `Address already in use` on local port 8765 | `lsof -i :8765` then kill the process, or use a different port |
| SSH multiplexing conflict | `ssh -O exit orion.nsls2.bnl.gov` then retry |
| `Connection refused` through tunnel | Verify the VLM service is actually running on the compute node |
| Python 3.9 type errors (`int \| None`) | `module load python/3.12.9` before creating the venv |
| `torch_dtype is deprecated` warning | Harmless — ignore |
| Slow model download | Set `HF_TOKEN` for authenticated access: `export HF_TOKEN=hf_...` |
| Tunnel drops after inactivity | Add `ServerAliveInterval 60` to your `~/.ssh/config` for the orion host |

---

## Port forwarding diagram

```
┌──────────────┐         ┌──────────────────┐         ┌──────────────────┐
│ Local machine│  SSH -J  │ orion (login)    │  SSH    │ marsX (compute)  │
│              │─────────▶│ (jump host only) │────────▶│                  │
│ :8765 ◀──────────────── tunnel ────────────────────── :8765 (uvicorn)  │
└──────────────┘         └──────────────────┘         └──────────────────┘
```
