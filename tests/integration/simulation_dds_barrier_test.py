"""Real DDS participants on reserved domain 222; CTest RUN_SERIAL."""
import json
from pathlib import Path
import subprocess
import sys

assert "222" in json.loads(Path(__file__).with_name("dds_domains.json").read_text())["reserved"]

for order in ("state-first", "result-first"):
    result = subprocess.run([sys.argv[1], order], capture_output=True,
                            text=True, timeout=10)
    print(order, result.stdout, result.stderr)
    assert result.returncode == 0, result.returncode
