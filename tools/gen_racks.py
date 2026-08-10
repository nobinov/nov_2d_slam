#!/usr/bin/env python3
"""Regenerate the rack internals (uprights, beams, stocked items) in warehouse.sdf.

Racks are OPEN structures: no solid back panel. Each rack is double-sided --
its 0.9 m depth is two 0.45 m lanes, one served from each aisle, stocked
independently.

Dimensions follow the reference warehouse used by warehouse_stock_count:
  mission_files/mission_file_sim_warehouse_v4.json
    rack_structure_map.vertical_uprights -> 2.796 m pitch, z 0.0 .. 1.8
    rack_structure_map.horizontal_beams  -> z = 0.35, 0.90, 1.45
  .../laser_heading_estimation_params.yaml
    pillar_extraction.min_length/max_length 0.06/0.16 -> 0.09 m nominal upright
    wall_protrusion.min/max 0.05/0.35                 -> aisle-ward protrusion

IDEMPOTENT: every upright/beam/item element is stripped before being rebuilt,
so re-running never double-applies. Item placement is seeded, so the same SEED
always yields the same warehouse.

    python3 tools/gen_racks.py [--seed N]
"""
import argparse
import random
import re

RACK_Z      = 1.25       # rack link origin height in world
PITCH       = 2.796      # upright pitch (reference)
UPRIGHT     = 0.09       # upright cross-section (reference nominal)
UPRIGHT_H   = 1.8        # upright height (reference)
RACK_HALF_Y = 0.45       # half the rack depth
FACE_Y      = RACK_HALF_Y + UPRIGHT / 2          # flush outside -> protrudes 0.09 m
UPRIGHT_XS  = (-PITCH, 0.0, PITCH)               # 3 per face, centred in the 8 m block

BAY_CENTRES = (-PITCH / 2, +PITCH / 2)           # 2 bays between the 3 uprights
LEVELS_W    = (0.35, 0.90, 1.45)                 # world z of each beam (reference)
# The reference gives beam CENTRELINES only, not a cross-section, so these are
# ours to choose. 0.06 keeps the level-0 beam's top at 0.38 -- 3.1 cm below the
# 0.411 m scan plane. At 0.10 the top was 0.40, and the robot's 0.37 deg
# nose-down pitch was enough to drop forward-oblique rays onto the beam.
BEAM_H, BEAM_D = 0.06, 0.05
BEAM_LEN    = PITCH - UPRIGHT                    # clear span between uprights

SLOTS       = 3                                  # slot positions per bay per level
SLOT_PITCH  = PITCH / SLOTS
ITEM_W, ITEM_D, ITEM_H = 0.80, 0.38, 0.30
ITEM_Y      = 0.25                               # inboard of the beam, inside the 0.45 m lane
MAX_PER_BAY = 3                                  # at most 3 items per bay

# Only the bottom level is stocked. The 0.411 m scan plane cuts level 0 items
# (z 0.38..0.68); levels 1 and 2 (z 0.93..1.23, 1.48..1.78) are far above it, so
# stocking them adds geometry the laser can never see. Beams are still built at
# all three levels -- they are rack structure, not stock.
ITEM_LEVELS = (0,)

UPRIGHT_COL = ("0.55 0.25 0.05 1", "0.90 0.42 0.08 1")
BEAM_COL    = ("0.15 0.18 0.30 1", "0.25 0.32 0.55 1")
PALETTE = [
    ("0.38 0.26 0.13 1", "0.62 0.44 0.24 1"),    # cardboard
    ("0.30 0.30 0.32 1", "0.52 0.52 0.55 1"),    # shrink-wrapped
    ("0.30 0.12 0.12 1", "0.60 0.24 0.20 1"),    # red crate
    ("0.14 0.28 0.20 1", "0.26 0.52 0.36 1"),    # green crate
    ("0.34 0.30 0.10 1", "0.68 0.60 0.20 1"),    # sacks
]


def box(name, pose, size, col):
    amb, dif = col
    return (f'        <collision name="{name}_collision">\n'
            f'          <pose>{pose}</pose>\n'
            f'          <geometry><box><size>{size}</size></box></geometry>\n'
            f'        </collision>\n'
            f'        <visual name="{name}_visual">\n'
            f'          <pose>{pose}</pose>\n'
            f'          <geometry><box><size>{size}</size></box></geometry>\n'
            f'          <material>\n'
            f'            <ambient>{amb}</ambient>\n'
            f'            <diffuse>{dif}</diffuse>\n'
            f'          </material>\n'
            f'        </visual>\n')


def build(rng):
    """Uprights + beams + stocked items for one rack block, both sides."""
    parts, stocked = [], 0
    for side, sy in (("n", +1), ("s", -1)):
        for i, xl in enumerate(UPRIGHT_XS):
            parts.append(box(
                f"upright_{side}{i}",
                f"{xl:.4g} {sy * FACE_Y:.3f} {UPRIGHT_H / 2 - RACK_Z:.3f} 0 0 0",
                f"{UPRIGHT} {UPRIGHT} {UPRIGHT_H}", UPRIGHT_COL))

        for bi, bx in enumerate(BAY_CENTRES):
            for li, zw in enumerate(LEVELS_W):
                parts.append(box(
                    f"beam_{side}{bi}_l{li}",
                    f"{bx:.4g} {sy * (RACK_HALF_Y + BEAM_D / 2):.3f} {zw - RACK_Z:.3f} 0 0 0",
                    f"{BEAM_LEN:.4g} {BEAM_D} {BEAM_H}", BEAM_COL))

        for bi, bx in enumerate(BAY_CENTRES):
            cells = [(li, si) for li in ITEM_LEVELS for si in range(SLOTS)]
            for li, si in rng.sample(cells, rng.randint(0, MAX_PER_BAY)):
                x = bx + (si - (SLOTS - 1) / 2) * SLOT_PITCH
                z = LEVELS_W[li] + BEAM_H / 2 + ITEM_H / 2 - RACK_Z
                parts.append(box(
                    f"item_{side}{bi}_l{li}_s{si}",
                    f"{x:.4g} {sy * ITEM_Y:.3f} {z:.3f} 0 0 0",
                    f"{ITEM_W} {ITEM_D} {ITEM_H}", rng.choice(PALETTE)))
                stocked += 1
    return "".join(parts), stocked


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=20260810)
    ap.add_argument("--sdf", default="warehouse.sdf")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    src = open(args.sdf).read()
    stats = {"racks": 0, "items": 0}

    def repl(m):
        name, body = m.group(1), m.group(2)
        if not name.startswith("rack_"):
            return m.group(0)
        stats["racks"] += 1
        # strip the old solid block and any previously generated internals
        for pat in (r'\n\s*<collision name="collision">.*?</collision>',
                    r'\n\s*<visual name="visual">.*?</visual>',
                    r'\n\s*<collision name="(?:upright|beam|item)_[^"]*">.*?</collision>',
                    r'\n\s*<visual name="(?:upright|beam|item)_[^"]*">.*?</visual>'):
            body = re.sub(pat, "", body, flags=re.S)
        extra, n = build(rng)
        stats["items"] += n
        return f'<link name="{name}">{body.rstrip()}\n\n{extra}      </link>'

    src = re.sub(r'<link name="(rack_[^"]+)">(.*?)\n      </link>', repl, src, flags=re.S)
    open(args.sdf, "w").write(src)

    bays = stats["racks"] * 2 * len(BAY_CENTRES)
    print(f"seed        : {args.seed}")
    print(f"racks       : {stats['racks']}")
    print(f"uprights    : {stats['racks'] * 2 * len(UPRIGHT_XS)}")
    print(f"beams       : {bays * len(LEVELS_W)}")
    print(f"items       : {stats['items']} / {bays * MAX_PER_BAY} capacity ({bays} bays)")


if __name__ == "__main__":
    main()
