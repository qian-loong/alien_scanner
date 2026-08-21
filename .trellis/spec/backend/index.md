# Backend Development Guidelines

> Best practices for backend development in this project.

---

## Overview

This directory contains guidelines for backend development. Fill in each file with your project's specific conventions.

---

## Guidelines Index

| Guide | Description | Status |
|-------|-------------|--------|
| [Directory Structure](./directory-structure.md) | Module organization and file layout | To fill |
| [Database Guidelines](./database-guidelines.md) | ORM patterns, queries, migrations | To fill |
| [Error Handling](./error-handling.md) | Error types, handling strategies | To fill |
| [Quality Guidelines](./quality-guidelines.md) | Code standards, forbidden patterns | To fill |
| [Logging Guidelines](./logging-guidelines.md) | Structured logging, log levels | To fill |
| [Perception Ray Evidence Contract](./perception-ray-evidence-contract.md) | LiDAR hit/free-ray capability, validation, and cross-layer propagation | Active |
| [Local Observation Map Contract](./local-observation-map-contract.md) | C2 pose/health gates, epoch/revision semantics, and revision-locked reads | Active |
| [Map State Update Contract](./map-state-update-contract.md) | C3 canonical keyframe/delta chains, atomic apply, bounded async production, and resync | Active |
| [Swarm Data Plane Contract](./swarm-data-plane-contract.md) | C4 routed envelope, RMW/DDS boundary, admission, resync barrier, aggregate atomicity, and trust rejection | Active |
| [Swarm Topology Contract](./swarm-topology-contract.md) | C5a stable identity, membership lifecycle, three logical graphs, and versioned routes | Active |
| [Swarm Role Contract](./swarm-role-contract.md) | C5b declared/effective capability, primary roles, services, global role epoch, and transition barriers | Active |
| [Swarm Runtime Contract](./swarm-runtime-contract.md) | C5c Explorer runtime gates, pure Relay forwarding, failover, resync correlation, and runtime visualization | Active |
| [Performance Measurement Boundaries](./performance-measurement-boundaries.md) | 本机（WSL2/LinuxKit）可测与不可测的性能指标、分辨率下限与措辞纪律 | Active |

---

## How to Fill These Guidelines

For each guideline file:

1. Document your project's **actual conventions** (not ideals)
2. Include **code examples** from your codebase
3. List **forbidden patterns** and why
4. Add **common mistakes** your team has made

The goal is to help AI assistants and new team members understand how YOUR project works.

---

**Language**: All documentation should be written in **English**.
