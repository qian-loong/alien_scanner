# Incremental Merkle v2 Short Gate Report

Date: 2026-08-15

## Decision

- **B/C algorithm Gate: GO.** Fixed-map delta identity is no longer proportional to the
  complete known-cell count. The effect is far larger than the local +/-30% CPU noise.
- **Production rollout: NO-GO in this task.** Production remains
  `Vector + flat SHA-256 v1`. C4.3 must complete ROS schema negotiation, v1/v2 rollout,
  production resource quotas, and a 3 x 300 s integrated ROS matrix before changing the
  default or allowing C5d to bind a production content-identity wire contract.
- This is an opt-in ROS-free feasibility result. It does not modify `MapUpdate.msg`, C4
  routing, C5 aggregate manifests, or the current visualized map path.

## Provenance

All 21 v3 runs passed the aggregate identity checks:

| Field | Value |
| --- | --- |
| Build | `RelWithDebInfo`, GCC 13.3.0, OpenSSL 3.0.13 |
| Benchmark SHA-256 | `da4275f033a1f0100f839ae901cd12d3c696fe58eb3e6dd7e1e681f959094082` |
| ELF build ID | `f1846243459ab1b80dafccac414c4e80798dbefb` |
| Docker image | `sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52` |
| Git baseline | `5875a41773b5f152e0bd51cb073387bb00d58cfd` plus manifest-listed working-tree source hashes |
| Source manifest SHA-256 | `d1430955463c0278fd77b3d7d8deb3eae10b13ce6f6129490d1ce0c98d426745` |
| Aggregate JSON SHA-256 | `4dabc249a8305a0bd05bb9b11f966a3f027134cc9dba73cf0c0272193a886f04` |

Absolute time values are scale indicators on this WSL2/Docker Desktop host. PSS/USS,
business counts, roots, node counts, and within-run phase relationships are the stronger
evidence. No sub-10% CPU gate is applied.

## Correctness

- A 400-sample update replay and a 200-sample insert replay both report
  `all_flat_hash_match=true` and `all_content_match=true`.
- The core gtest replay independently compares incremental and full-rebuild roots at 400
  three-dimensional insert/replace/delete checkpoints.
- Golden tests cover signed 192-bit coordinates, empty trie root, and a single leaf.
- Keyframe, delta, revision-only, last-cell deletion, remove tombstone, wrong base,
  descriptor drift, unknown kind, and tampered envelope paths are covered.
- Latest normal build/test: `398 tests, 0 errors, 0 failures, 0 skipped`.

## Update workload

Each delta touches four existing chunks. A is the current production layout shadow,
B is the fixed C4.1 baseline, and C is the v2 candidate.

| Known cells | A Vector+flat apply | B chunk+flat apply | C chunk+Merkle apply | B/C | B PSS | C PSS | C-B PSS |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 10k | 318.6 us | 483.3 us | 23.6 us | 20.5x | 5409 KiB | 5501 KiB | +92 KiB (+1.7%) |
| 100k | 3.399 ms | 6.131 ms | 32.4 us | 189x | 8571 KiB | 9017 KiB | +446 KiB (+5.2%) |
| 500k | 18.49 ms | 40.22 ms | 46.9 us | 858x | 22895 KiB | 24481 KiB | +1586 KiB (+6.9%) |

At 100k, C mean phase time is 6.9 us store, 0.8 us mutation preparation,
24.4 us tree/root, and 0.03 us commit. The tree phase contains about 13.2 us leaf hash,
6.0 us branch hash, and 0.54 us outer content hash. Performance-mode C samples contain
zero flat-hash work; this is enforced by `analyze_short_gate.py`.

## Keyframe workload

Flat keyframe totals include both store replacement and the complete flat SHA-256. Merkle
totals include chunk store build, full Patricia build, and commit.

| Cells | A Vector+flat | B chunk+flat | C chunk+Merkle | C/B |
| ---: | ---: | ---: | ---: | ---: |
| 10k | 0.411 ms | 0.927 ms | 1.145 ms | 1.23x |
| 100k | 3.894 ms | 10.975 ms | 11.851 ms | 1.08x |
| 500k | 20.171 ms | 70.838 ms | 65.511 ms | 0.92x |

The one-time keyframe cost remains O(N). The candidate is slightly slower at 10k/100k,
but the regression is bounded and is amortized by subsequent deltas; no hidden 10% gate
is used.

## Touched-chunk scaling

Fixed 100k update workload:

| Touched chunks | Mean apply | P95 apply | P95 tree+store candidate bytes |
| ---: | ---: | ---: | ---: |
| 1 | 13.2 us | 17.4 us | 12.1 KiB |
| 4 | 30.1 us | 40.2 us | 20.4 KiB |
| 16 | 91.1 us | 121.0 us | 51.8 KiB |
| 64 | 331.6 us | 435.5 us | 175.0 KiB |

Fixed 100k insert workload:

| New chunks/update | Mean apply | P95 apply | P95 tree+store candidate bytes |
| ---: | ---: | ---: | ---: |
| 1 | 7.9 us | 9.5 us | 10.1 KiB |
| 4 | 21.7 us | 31.0 us | 17.4 KiB |
| 16 | 83.2 us | 124.6 us | 62.4 KiB |
| 64 | 401.3 us | 581.2 us | 478.9 KiB |

The 100k insert A/B/C run is 3.779 ms / 5.968 ms / 21.7 us. After 330 updates
insert four new chunks each, the reachable Merkle node-object bytes are 765.7 KiB, which
tracks live chunk growth rather than revision history.

`owned_bytes` counts reachable `Node` objects and excludes allocator/control-block
overhead. PSS/USS are therefore the authority for whole-process resident memory.

## Memory tools

- ASan/LSan/UBSan: 18 Merkle core tests passed, including the 400-checkpoint replay; no
  sanitizer diagnostic.
- Strict Memcheck correctness run: 152,701 allocations, 152,701 frees, 0 bytes at exit,
  0 errors, 0 suppressed errors.
- Heaptrack 100k update run: 91,993 allocations, 10.16 MiB peak heap, 19.10 MiB peak RSS
  including Heaptrack overhead. Its reported 4.10 KiB at exit is one libc stdout buffer
  allocated from `main` output, not a Merkle/COW allocation; Memcheck and LSan both show
  no retained allocation.

## Remaining production work

1. Define ROS-visible v2 descriptor fields, negotiation, dual-read/write, downgrade, and
   rollback. A bare 32-byte SHA-256 cannot distinguish flat v1 from Merkle v2.
2. Add production admission limits for keyframe cells, live chunks/nodes, candidate bytes,
   and resync envelopes before exposing v2 to untrusted transport input.
3. Integrate v2 into the real receiver behind an opt-in flag, then run the repository's
   3 x 300 s ROS matrix and visualization regression. Those runs are not executable against
   the current deliberately ROS-free prototype and are **not marked passed** here.
4. Only after that integrated Gate may production default selection be reconsidered.
