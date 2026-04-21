# Cooperative Cylindrical Patch Printable Shell

- File: `cooperative_cylindrical_patch_printable.obj`
- Units: millimetres
- Type: watertight printable shell

Source dimensions copied from the dual-arm cooperative polishing demo:
- inner radius: 120 mm
- axial span: 300 mm
- arc length: 300 mm
- angular span: 143.239 deg
- chord width: 227.756 mm

Printable shell parameters:
- wall thickness: 8 mm
- angular segments: 96
- axial segments: 32

Notes:
- The inner cylindrical face matches the demo curvature.
- The shell is thickened outward, capped on all four boundary loops, and is watertight.
- Re-run `node dq_vizlab_academic/tools/generate_printable_cylindrical_patch_obj.cjs` to regenerate.
