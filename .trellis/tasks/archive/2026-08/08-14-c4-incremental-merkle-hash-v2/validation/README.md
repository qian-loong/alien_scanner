# C4 Incremental Merkle v2 Validation

Only the `evidence-v3-*` directories are authoritative for the short Gate. Earlier
`short-*`, `scale-*`, `optimized-*`, `final-*`, `gate-*`, `evidence-v2-*`, and the
unversioned `evidence-*` directories are exploratory runs from older executable or
provenance revisions and are excluded from every final aggregate.

Complete raw evidence, including those exploratory runs, is retained locally under the
ignored path:

```text
profiling-archive/c4-merkle-v2-20260815/raw/
```

The relocation preserved 905 files, 4,699,734 bytes, and tree SHA-256
`301d653297508f41433545caffa14810afd0d719b1e751340d0a076e2beada34`.
See `relocation-provenance.txt` for the path mapping and manifest format.

## Authoritative files

- `short-gate-report.md`: decision, boundaries, and human-readable tables.
- `short-gate-aggregate.json` / `.csv`: values recomputed from 21 v3 raw runs.
- `run_short_gate.py`: PID/starttime-safe benchmark and `smaps_rollup` capture.
- `analyze_short_gate.py`: identity, mode-isolation, raw-row, and aggregate checks.
- `evidence-v3-*`: Git retains each benchmark summary and provenance manifest; the raw
  CSV, memory samples, smaps, stdout, and stderr remain in the local archive.
- `tool-memcheck-v3`: Memcheck workload summary. The original Valgrind log remains in
  the local archive; its strict result is recorded in `short-gate-report.md`.
- `tool-heaptrack-v3`: Heaptrack workload summary. The `.gz` trace remains in the local
  archive.

Every v3 manifest records the executable SHA-256/build ID, Docker image SHA, Git HEAD,
relevant source hashes, compiler/OpenSSL versions, command, PID, process-group ID, and
`/proc/<pid>/stat` starttime. The aggregate refuses mixed identities, missing raw smaps,
flat work in Merkle performance mode, Merkle work in flat performance mode, or a summary
mean that differs from the raw CSV.

The aggregate can be replayed without restoring raw into the task directory:

```text
python .trellis/tasks/08-14-c4-incremental-merkle-hash-v2/validation/analyze_short_gate.py \
  --raw-root profiling-archive/c4-merkle-v2-20260815/raw \
  --output-dir .trellis/tasks/08-14-c4-incremental-merkle-hash-v2/validation
```

For a fixed raw tree and analyzer, the aggregate is byte-deterministic. The mirrored
evidence files are hash-verified copies; capture-time absolute paths inside raw manifests
remain unchanged.
