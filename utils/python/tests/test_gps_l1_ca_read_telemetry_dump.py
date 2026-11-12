"""Tests for the GPS L1 C/A telemetry dump reader.

Copyright (C) 2010-2026 (see AUTHORS file for a list of contributors)
SPDX-License-Identifier: GPL-3.0-or-later
"""

import importlib.util
import io
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock


_READER_PATH = (Path(__file__).resolve().parents[1] / 'lib'
                / 'gps_l1_ca_read_telemetry_dump.py')
_SPEC = importlib.util.spec_from_file_location('telemetry_reader', _READER_PATH)
telemetry_reader = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(telemetry_reader)

_FIELDS = ('tow_current_symbol_ms', 'tracking_sample_counter', 'tow',
           'nav_simbols', 'prn')


def pack_record(record):
    # The C++ writer emits each scalar separately in native byte order.
    return b''.join(struct.pack('=' + fmt, value)
                    for fmt, value in zip(('d', 'Q', 'd', 'i', 'i'), record))


class TelemetryDumpReaderTest(unittest.TestCase):
    def read_dump(self, data):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / 'telemetry0.dat'
            filename.write_bytes(data)
            return telemetry_reader.gps_l1_ca_read_telemetry_dump(filename)

    def test_reads_signed_navigation_symbols(self):
        records = [(100.0, 1, 99.0, -1, 3), (100.02, 2, 99.0, 1, 3)]
        result = self.read_dump(b''.join(map(pack_record, records)))
        self.assertEqual(result['nav_simbols'], [-1, 1])

    def test_preserves_large_unsigned_sample_counters(self):
        counters = [2**53 + 1, 2**64 - 1]
        records = [(100.0, counter, 99.0, 1, 3) for counter in counters]
        result = self.read_dump(b''.join(map(pack_record, records)))
        self.assertEqual(result['tracking_sample_counter'], counters)
        self.assertTrue(all(isinstance(value, int)
                            for value in result['tracking_sample_counter']))

    def test_reads_all_fields_from_multiple_records(self):
        records = [(100.5, 123456789, 96.0, 1, 3),
                   (101.25, 123456790, 102.0, 1, 12),
                   (105.0, 123456791, 102.0, 1, 32)]
        result = self.read_dump(b''.join(map(pack_record, records)))
        self.assertEqual(result, dict(zip(_FIELDS, map(list, zip(*records)))))

    def test_empty_file_returns_empty_columns(self):
        self.assertEqual(self.read_dump(b''), {field: [] for field in _FIELDS})

    def test_incomplete_first_record_returns_empty_columns(self):
        record = pack_record((100.0, 123456789, 96.0, 1, 3))
        for length in range(1, len(record)):
            with self.subTest(length=length):
                self.assertEqual(self.read_dump(record[:length]),
                                 {field: [] for field in _FIELDS})

    def test_ignores_incomplete_tail_without_partial_columns(self):
        complete = (100.0, 123456789, 96.0, 1, 3)
        tail = pack_record((101.0, 123456790, 96.0, -1, 3))
        expected = dict(zip(_FIELDS, ([value] for value in complete)))
        for length in range(1, len(tail)):
            with self.subTest(length=length):
                result = self.read_dump(pack_record(complete) + tail[:length])
                self.assertEqual(result, expected)

    def test_closes_file_at_clean_or_partial_end(self):
        record = pack_record((100.0, 123456789, 96.0, 1, 3))
        for data in (b'', record, record + record[:7]):
            with self.subTest(size=len(data)):
                stream = io.BytesIO(data)
                self.addCleanup(stream.close)
                with mock.patch.object(telemetry_reader, 'open',
                                       return_value=stream, create=True):
                    telemetry_reader.gps_l1_ca_read_telemetry_dump('unused.dat')
                self.assertTrue(stream.closed)

    def test_closes_file_if_read_raises(self):
        class FailedRead(io.BytesIO):
            def read(self, size=-1):
                raise OSError('read failed')

        stream = FailedRead()
        self.addCleanup(stream.close)
        with mock.patch.object(telemetry_reader, 'open', return_value=stream,
                               create=True):
            with self.assertRaises(OSError):
                telemetry_reader.gps_l1_ca_read_telemetry_dump('unused.dat')
        self.assertTrue(stream.closed)


if __name__ == '__main__':
    unittest.main()
