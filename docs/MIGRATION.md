# Migration from the old repos (done once, Nov 2026)

Before this change, the lab's robot models lived in two repositories that disagreed with each other:

| | old All_Robot | Robots_IsaacLab |
|---|---|---|
| ROS package | `robot_description` | none |
| mesh URIs | `package://robot_description/meshes/...` | `/home/xuezhi/Robots_IsaacSim/...` (absolute), `package://myagv_urdf/...`, `package://mycobot_description/...` |
| content | Go2, myAGV, myCobot 280 Pi, Nova5+2F85, composites, RViz launch files | Nova5, myCobot variants, Robotiq, Isaac USDs |

`scripts/migrate.py` merged them into `robots/<name>/`:

- Identical files (by sha256) were de-duplicated.
- On a name clash, the old All_Robot version was kept and the Robots_IsaacLab version went to `urdf/legacy/`.
- All USDs were kept under `usd/legacy/`.

`scripts/normalize_urdf.py` then rewrote every mesh path and filled in missing physics. See `migration_report.md` and `normalize_report.md` for every individual change.

Reproduce:

```bash
python3 scripts/migrate.py --all-robot <old All_Robot clone> --isaaclab <Robots_IsaacLab clone>
python3 scripts/build_assets.py
python3 scripts/validate.py -v
```

Old ROS command → new:

| old | new |
|---|---|
| `ros2 launch robot_description all_in_one.launch.py` | `ros2 launch cps_robot_description all_in_one.launch.py` |
| `ros2 launch robot_description go2.launch.py` | `ros2 launch cps_robot_description display.launch.py robot:=go2` |
| `.../nova5.launch.py` | `display.launch.py robot:=nova5_2f85` |
| `.../myagv.launch.py`, `mycomb.launch.py` | `display.launch.py robot:=myagv` / `robot:=mycomb` |

After the merge, archive `Robots_IsaacLab` on GitHub (Settings → Archive) and point its README here.
