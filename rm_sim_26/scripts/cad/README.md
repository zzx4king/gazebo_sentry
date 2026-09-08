# CAD Pipeline Scripts

One-off tooling used to build `models/sentry/` from the SolidWorks assembly. Kept for
reproducibility; not part of the ROS build.

These scripts need intermediate data that is **not** committed (a few hundred MB):

| File | Produced by |
|---|---|
| `root_asm.json` | Onshape assembly definition API, with `includeMateFeatures=true` |
| `densities.json` | `swx_dump` over every `.SLDPRT`, giving mass and volume per part |
| `meshes_slim/*.glb` | `repack_meshes.py` output |

## Pipeline

1. Upload the SolidWorks assembly to Onshape (zip filename must match the root assembly
   name, and non-ASCII names fail the server-side check).
2. Export each subassembly as glTF via the assembly API, then `repack_meshes.py` strips
   geometry the node tree never references. This cut the set from 41 MB to 24 MB.
3. `recolor.py` assigns materials: name-based overrides first (referee modules, camera,
   lidar, omni-wheel rollers), then CAD density classification for structural parts.
   Acrylic is identified by density plus name because its density falls inside the
   printed-plastic band.
4. `fix_inertia.py` prints corrected link inertias — it removes double-counted parts and
   applies purchased-part mass corrections.

`scripts/check_inertia.py` needs none of this; it parses `model.sdf` directly and reports
the effective inertia about each joint axis.

## Notes

- glTF `baseColorFactor` is linear, not sRGB. Writing 0.145 renders as roughly RGB 107.
  `recolor.py` converts with `((c + 0.055) / 1.055) ** 2.4`.
- Transparency needs `alphaMode='BLEND'` declared explicitly; an alpha below 1 alone is
  ignored by renderers.
- SolidWorks mass properties are reported about the center of mass. Verify with the
  parallel-axis lower bound: any axis where `I < m * d^2` rules out an origin reference.
