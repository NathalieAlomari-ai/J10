# j10-shm-protocol

The shared-memory wire contract between the CV node and the MAVLink bridge, factored out
as its own installable package so neither service depends on the other's internals — both
`mavlink_bridge` and `cv_node` depend on this instead.

No dependencies beyond the Python standard library (`multiprocessing.shared_memory`,
`struct`). See the module docstring in `j10_shm_protocol/__init__.py` for the full wire
format (36 bytes, little-endian) and the seqlock concurrency scheme.

```python
# writer side (the CV node)
from j10_shm_protocol import CVCommandWriter
writer = CVCommandWriter(name="j10_cv_cmd")
writer.write(vx=0.2, vy=0.0, vz=0.0, yaw_rate=0.1, valid=True)

# reader side (the MAVLink bridge)
from j10_shm_protocol import CVCommandReader
reader = CVCommandReader(name="j10_cv_cmd")
cmd = reader.read()   # CVCommand | None
```

## Testing this package on its own

```bash
cd companion/j10_shm_protocol
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest -v
```

For how this fits into the rest of PT1, see [`../README.md`](../README.md).
