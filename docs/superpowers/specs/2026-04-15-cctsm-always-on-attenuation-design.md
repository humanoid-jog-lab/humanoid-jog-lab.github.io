# CCTSM Always-On VFI Attenuation

## Goal

Make the CCTSM attenuation active for every monitored VFI constraint, not only for near-boundary constraints.

## Scope

- Update the paper definition of `W_vfi` from threshold-activated attenuation to always-on attenuation.
- Keep controller-side VFI constraints unchanged.
- Update `cctsm.js` and the VFI-constraint collection helpers so the manipulability metric sees all monitored constraints.

## Design

- Replace the near-active set formulation with an always-on attenuation law using the existing distance scale `\bar d_i`.
- Use
  `omega_i = d_i / (d_i + \bar d_i)`
  and attenuation weight
  `1 - omega_i = \bar d_i / (d_i + \bar d_i)`.
- Interpret the existing `threshold` field in code as the attenuation scale for CCTSM.
- Keep control rows for waist corridor VFI thresholded as before, but expose all finite corridor distances to the CCTSM-side attenuation builder.

## Verification

- Unit tests for `computeVFIAttenuationMatrix`.
- Unit tests for manipulability VFI constraint collection.
- Rebuild the frontend.
- Rebuild `submit_v1.pdf`.
