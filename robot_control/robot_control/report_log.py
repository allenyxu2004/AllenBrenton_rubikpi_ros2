"""Automatically named, line-flushed report files, independent of ROS logging."""

from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path


class ReportLog:
    """Keep one exclusive report file per controller run."""

    def __init__(self, directory, node_name, run_id):
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(ZoneInfo('America/Los_Angeles')).strftime('%Y%m%dT%H%M%S.%f%z')
        self.path = directory / f'{node_name}_{timestamp}_{run_id[:8]}.log'
        self.stream = self.path.open('x', encoding='utf-8', buffering=1)

    def write(self, line):
        """Flush each full record so it can be parsed during a run."""
        self.stream.write(line + '\n')
        self.stream.flush()

    def close(self):
        self.stream.close()
