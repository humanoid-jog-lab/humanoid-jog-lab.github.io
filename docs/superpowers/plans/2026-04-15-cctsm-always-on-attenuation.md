# Plan: CCTSM Always-On VFI Attenuation

1. Update the Section IV paper derivation and prose to remove near-active activation and define always-on attenuation.
2. Update `computeVFIAttenuationMatrix()` to attenuate every finite monitored constraint.
3. Update joint-limit and waist-corridor constraint collection so CCTSM receives all finite distances while controller rows stay thresholded.
4. Adjust affected unit tests to the new always-on semantics.
5. Run targeted tests, `npm run build`, and rebuild the paper PDF.
