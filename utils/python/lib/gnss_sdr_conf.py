"""
 gnss_sdr_conf.py

 Helpers to read GNSS-SDR configuration files from the Python plotting
 utilities.

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2026  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional


# Keep this order in sync with src/core/receiver/gnss_block_factory.cc. GNSS-SDR
# assigns absolute channel IDs by walking the configured signal counts in this
# order.
SIGNAL_ORDER = (
    "1C",
    "2S",
    "L5",
    "1B",
    "5X",
    "E6",
    "1G",
    "2G",
    "B1",
    "B3",
    "7X",
    "J1",
    "J5",
)

# GNSS-SDR signal nomenclature: each signal code maps to the system character
# used in acquisition dump filenames and to its primary code length in chips.
SIGNAL_TYPES = {
    "1C": ("G", 1023),  # GPS L1 C/A
    "2S": ("G", 10230),  # GPS L2C
    "L5": ("G", 10230),  # GPS L5
    "1B": ("E", 4092),  # Galileo E1B
    "5X": ("E", 10230),  # Galileo E5a
    "7X": ("E", 10230),  # Galileo E5b
    "E6": ("E", 5115),  # Galileo E6
    "1G": ("R", 511),  # GLONASS L1
    "2G": ("R", 511),  # GLONASS L2
    "B1": ("C", 2046),  # BeiDou B1I
    "B3": ("C", 10230),  # BeiDou B3I
    "J1": ("J", 1023),  # QZSS L1 C/A
    "J5": ("J", 10230),  # QZSS L5
}

N_CHIPS = {(system, code): nc for code, (system, nc) in SIGNAL_TYPES.items()}


class ConfigError(ValueError):
    """Raised when a configuration file cannot provide required metadata."""


@dataclass(frozen=True)
class SignalConfig:
    signal: str
    system: str
    n_chips: int
    count: int
    first_channel: int
    acquisition_dump_filename: Optional[str]
    tracking_dump_filename: Optional[str]
    telemetry_dump_filename: Optional[str]
    acquisition_implementation: Optional[str]
    tracking_implementation: Optional[str]
    telemetry_implementation: Optional[str]

    @property
    def last_channel(self):
        if self.count <= 0:
            return None
        return self.first_channel + self.count - 1

    @property
    def channels(self):
        return range(self.first_channel, self.first_channel + self.count)

    def range_label(self):
        if self.count == 0:
            return "none"
        if self.count == 1:
            return str(self.first_channel)
        return f"{self.first_channel}..{self.last_channel}"


@dataclass(frozen=True)
class ChannelDump:
    channel: int
    signal: str
    file_prefix: str


def add_conf_argument(parser):
    parser.add_argument(
        "--conf",
        type=Path,
        default=None,
        help="GNSS-SDR configuration file. When provided, missing plotting "
        "options such as dump filename, channel range, signal type, sampling "
        "frequency, and PVT period are inferred from it.",
    )


def load_gnss_sdr_conf(path):
    return GnssSdrConfig(Path(path))


class GnssSdrConfig:
    def __init__(self, path):
        self.path = Path(path)
        self.values = _parse_config_file(self.path)
        self._signals = None

    def get_str(self, key, default=None):
        return self.values.get(_normalize_key(key), default)

    def get_bool(self, key, default=None):
        value = self.get_str(key)
        if value is None:
            return default
        normalized = value.strip().lower()
        if normalized in ("true", "yes", "on", "1"):
            return True
        if normalized in ("false", "no", "off", "0"):
            return False
        return default

    def get_int(self, key, default=None):
        value = self.get_str(key)
        if value is None:
            return default
        try:
            return int(value, 0)
        except ValueError:
            return default

    def get_float(self, key, default=None):
        value = self.get_str(key)
        if value is None:
            return default
        try:
            return float(value)
        except ValueError:
            return default

    @property
    def internal_fs_sps(self):
        return self.get_float(
            "GNSS-SDR.internal_fs_sps",
            self.get_float("GNSS-SDR.internal_fs_hz"),
        )

    @property
    def pvt_output_rate_ms(self):
        return self.get_float("PVT.output_rate_ms")

    @property
    def observables_dump_filename(self):
        return self.get_str("Observables.dump_filename")

    @property
    def pvt_dump_filename(self):
        return self.get_str("PVT.dump_filename")

    @property
    def signals(self):
        if self._signals is None:
            self._signals = self._build_signals()
        return self._signals

    @property
    def enabled_signals(self):
        return [signal for signal in self.signals if signal.count > 0]

    @property
    def total_channels(self):
        return sum(signal.count for signal in self.signals)

    def signal(self, signal_type):
        requested = signal_type.upper()
        for signal in self.signals:
            if signal.signal == requested:
                return signal
        return None

    def require_enabled_signal(self, signal_type):
        signal = self.signal(signal_type)
        if signal is None:
            choices = ", ".join(SIGNAL_ORDER)
            raise ConfigError(
                f"Unsupported signal type {signal_type!r}. Choices: {choices}."
            )
        if signal.count <= 0:
            raise ConfigError(
                f"Configuration {self.path} enables no {signal.signal} "
                "channels."
            )
        return signal

    def select_signal(
        self,
        requested_signal=None,
        *,
        default_signal=None,
        prefer_default_on_ambiguous=False,
    ):
        if requested_signal is not None:
            return self.require_enabled_signal(requested_signal)

        enabled = self.enabled_signals
        if len(enabled) == 1:
            return enabled[0]

        if (
            default_signal is not None
            and prefer_default_on_ambiguous
            and self.signal(default_signal) is not None
            and self.signal(default_signal).count > 0
        ):
            return self.signal(default_signal)

        if not enabled:
            raise ConfigError(
                f"Configuration {self.path} enables no GNSS-SDR channels."
            )

        available = ", ".join(
            f"{signal.signal} (channels {signal.range_label()})"
            for signal in enabled
        )
        raise ConfigError(
            f"Configuration {self.path} enables multiple signals: {available}. "
            "Pass --signal-type to select one."
        )

    def select_signals(self, requested_signal=None):
        if requested_signal is not None:
            return [self.require_enabled_signal(requested_signal)]

        enabled = self.enabled_signals
        if not enabled:
            raise ConfigError(
                f"Configuration {self.path} enables no GNSS-SDR channels."
            )
        return enabled

    def channel_dump_plan(
        self,
        *,
        signals,
        dump_filename_attr,
        default_file_prefix,
        override_file_prefix=None,
    ):
        plan = []
        for signal in signals:
            file_prefix = (
                override_file_prefix
                or getattr(signal, dump_filename_attr)
                or default_file_prefix
            )
            for channel in signal.channels:
                plan.append(ChannelDump(channel, signal.signal, file_prefix))
        return plan

    def summary_lines(self):
        lines = [f"Configuration: {self.path}"]
        if self.internal_fs_sps is not None:
            lines.append(f"internal_fs_sps: {self.internal_fs_sps:g}")
        lines.append("signals:")
        for signal in self.enabled_signals:
            parts = [
                f"  {signal.signal}: channels {signal.range_label()}",
            ]
            if signal.tracking_dump_filename:
                parts.append(f"tracking dump {signal.tracking_dump_filename}")
            if signal.acquisition_dump_filename:
                parts.append(
                    f"acquisition dump {signal.acquisition_dump_filename}"
                )
            lines.append(", ".join(parts))
        if self.observables_dump_filename:
            lines.append(f"observables dump: {self.observables_dump_filename}")
        if self.pvt_dump_filename:
            lines.append(f"PVT dump: {self.pvt_dump_filename}")
        return lines

    def _build_signals(self):
        signals = []
        first_channel = 0
        for signal_type in SIGNAL_ORDER:
            system, n_chips = SIGNAL_TYPES[signal_type]
            count = self.get_int(f"Channels_{signal_type}.count", 0)
            if count is None:
                count = 0
            if count < 0:
                raise ConfigError(
                    f"Configuration {self.path} has a negative channel count "
                    f"for Channels_{signal_type}.count."
                )
            signals.append(
                SignalConfig(
                    signal=signal_type,
                    system=system,
                    n_chips=n_chips,
                    count=count,
                    first_channel=first_channel,
                    acquisition_dump_filename=self._block_value(
                        "Acquisition", signal_type, "dump_filename"
                    ),
                    tracking_dump_filename=self._block_value(
                        "Tracking", signal_type, "dump_filename"
                    ),
                    telemetry_dump_filename=self._block_value(
                        "TelemetryDecoder", signal_type, "dump_filename"
                    ),
                    acquisition_implementation=self._block_value(
                        "Acquisition", signal_type, "implementation"
                    ),
                    tracking_implementation=self._block_value(
                        "Tracking", signal_type, "implementation"
                    ),
                    telemetry_implementation=self._block_value(
                        "TelemetryDecoder", signal_type, "implementation"
                    ),
                )
            )
            first_channel += count
        return signals

    def _block_value(self, role, signal_type, option):
        return self.get_str(
            f"{role}_{signal_type}.{option}",
            self.get_str(f"{role}.{option}"),
        )


def _parse_config_file(path):
    if not path.exists():
        raise ConfigError(f"Configuration file not found: {path}")

    flat_values = {}
    gnss_values = {}
    has_gnss_section = False
    active_section = None

    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except UnicodeDecodeError:
        lines = path.read_text(encoding="latin-1").splitlines()

    for line_no, raw_line in enumerate(lines, start=1):
        if raw_line.lstrip().startswith("#"):
            continue
        line = _strip_inline_comment(raw_line).strip()
        if not line:
            continue
        if line.startswith("#"):
            continue

        if line.startswith("["):
            if not line.endswith("]"):
                raise ConfigError(
                    f"{path}:{line_no}: malformed section header."
                )
            active_section = line[1:-1].strip().lower()
            if active_section == "gnss-sdr":
                has_gnss_section = True
            continue

        if "=" not in line:
            raise ConfigError(f"{path}:{line_no}: expected key=value.")

        key, value = line.split("=", 1)
        key = _normalize_key(key)
        value = value.strip()
        if active_section is None:
            flat_values[key] = value
        elif active_section == "gnss-sdr":
            gnss_values[key] = value

    values = gnss_values if has_gnss_section else flat_values
    if not values:
        raise ConfigError(f"No GNSS-SDR configuration values found in {path}.")
    return values


def _normalize_key(key):
    return key.strip().lower()


def _strip_inline_comment(line):
    return line.split(";", 1)[0]
