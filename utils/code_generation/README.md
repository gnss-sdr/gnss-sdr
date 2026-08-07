<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2026 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

# Code Generation Utilities

The scripts in this folder regenerate source files that are checked into the
repository. They are not needed to build or run GNSS-SDR; they only need to be
re-run when their inputs change, and the regenerated files must be committed.

## generate_beidou_b1c_codes.py

Generates `src/core/system_parameters/Beidou_B1C_codes.h` and
`Beidou_B1C_codes.cc` (the precomputed BeiDou B1C primary and secondary
spreading codes) from the ICD Weil-code parameters declared in
`src/core/system_parameters/Beidou_B1C_prn.h`. Run it from any directory:

```
python3 utils/code_generation/generate_beidou_b1c_codes.py
```

The script overwrites both output files in place and prints the paths it wrote.
The generated files carry an "Auto-generated. Do not edit manually" notice; edit
`Beidou_B1C_prn.h` (or this script) instead and regenerate.
