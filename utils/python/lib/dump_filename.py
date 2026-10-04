"""
 dump_filename.py

 Helper to interpret a GNSS-SDR ``dump_filename`` configuration value the same
 way the C++ signal-processing blocks do, so the plotting utilities can locate
 the dump files from the exact string the user wrote in the configuration.

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2026  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

from pathlib import Path


def resolve_dump_prefix(file_prefix, input_path="."):
    """Split a GNSS-SDR ``dump_filename`` into ``(directory, basename)``.

    Mirrors the C++ dump-file logic shared by the tracking, telemetry,
    observables, PVT, and acquisition blocks: the directory part (everything
    before the last ``/``) is separated from the basename, and any extension
    on the basename is removed. The directory is resolved relative to
    ``input_path`` (an absolute ``file_prefix`` path overrides it). The
    block-specific suffixes (channel number, ``.dat``, ``.mat``, ...) are
    appended by the caller.

    Examples:
        resolve_dump_prefix("track_ch")            -> (Path("."), "track_ch")
        resolve_dump_prefix("./observables.dat")   -> (Path("."), "observables")
        resolve_dump_prefix("run1/acq", "data")    -> (Path("data/run1"), "acq")
    """
    prefix_path = Path(file_prefix)
    name = prefix_path.name
    # Remove a trailing extension, ignoring a possible leading dot (matches the
    # ``substr(1).find_last_of('.')`` logic used by the GNSS-SDR blocks).
    dot = name[1:].rfind(".")
    if dot != -1:
        name = name[: dot + 1]
    directory = Path(input_path) / prefix_path.parent
    return directory, name
