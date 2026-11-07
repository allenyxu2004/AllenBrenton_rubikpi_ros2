"""One JSON-lines report per run, named in Los Angeles time."""

from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo

REPORT_TIMEZONE = ZoneInfo('America/Los_Angeles')


class ReportLog:
    def __init__(self, directory, logger):
        self.logger = logger
        self.stream = None
        try:
            directory = Path(directory).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(REPORT_TIMEZONE).strftime('%Y%m%dT%H%M%S.%f%z')
            self.path = directory / f'tag_route_{stamp}.jsonl'
            self.stream = self.path.open('x', encoding='utf-8', buffering=1)
            logger.info(f'Automatic report log: {self.path}')
        except OSError as error:
            logger.error(f'Cannot create report; stdout only: {error}')

    def write(self, record):
        line = json.dumps(record, allow_nan=False)
        if self.stream is None:
            self.logger.info(line)
            return
        try:
            self.stream.write(line + '\n')
        except OSError as error:
            self.logger.error(f'Report write failed; stdout only: {error}')
            self.close()
            self.logger.info(line)

    def close(self):
        if self.stream is not None:
            try:
                self.stream.close()
            except OSError as error:
                self.logger.error(f'Report close failed: {error}')
            self.stream = None
