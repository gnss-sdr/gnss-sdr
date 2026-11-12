"""Tests for the units used by the GPS L1 C/A telemetry plots.

Copyright (C) 2010-2026 (see AUTHORS file for a list of contributors)
SPDX-License-Identifier: GPL-3.0-or-later
"""

import importlib.util
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


_UTILS_PATH = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location(
    'telemetry_plot', _UTILS_PATH / 'gps_l1_ca_telemetry_plot_sample.py')
telemetry_plot = importlib.util.module_from_spec(_SPEC)
with mock.patch.object(sys, 'path', [str(_UTILS_PATH)] + sys.path):
    _SPEC.loader.exec_module(telemetry_plot)


class TelemetryPlotTest(unittest.TestCase):
    def test_current_symbol_tow_uses_dump_seconds(self):
        records = {
            0: [(345600.125, 10, 345600.0, 1, 3),
                (345600.145, 11, 345600.0, 1, 3)],
            1: [(345601.25, 20, 345600.0, 1, 8),
                (345601.27, 21, 345600.0, 1, 8)],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            for channel, rows in records.items():
                # The decoder writes these scalars separately in native byte
                # order. Both TOW fields have already been converted to seconds.
                data = b''.join(
                    struct.pack('=' + fmt, value)
                    for row in rows
                    for fmt, value in zip(('d', 'Q', 'd', 'i', 'i'), row))
                (path / f'telemetry{channel}.dat').write_bytes(data)

            args = SimpleNamespace(
                input_path=path, file_prefix='telemetry', first_channel=0,
                channels=2, channel_a=0, channel_b=1, fig_path=path / 'plots',
                output_format='png', show=False)
            plotter = mock.Mock()
            with mock.patch.object(telemetry_plot, 'parse_args',
                                   return_value=args), \
                    mock.patch.object(telemetry_plot, 'apply_publication_style'), \
                    mock.patch.object(telemetry_plot, 'plt', plotter):
                # Exercise main() and the real binary reader without opening
                # figures or requiring an interactive plotting backend.
                telemetry_plot.main()

        calls = plotter.plot.call_args_list
        self.assertEqual(len(calls), 4)
        for channel in (0, 1):
            with self.subTest(channel=channel):
                self.assertEqual(calls[channel].args[0],
                                 [row[1] for row in records[channel]])
                self.assertEqual(calls[channel].args[1],
                                 [row[0] for row in records[channel]])
        for channel in (0, 1):
            self.assertEqual(calls[channel + 2].args[1], [345600.0, 345600.0])


if __name__ == '__main__':
    unittest.main()
