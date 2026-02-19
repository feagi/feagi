# FEAGI Documentation Restructuring Plan

## Overview
This document outlines the plan for restructuring the FEAGI documentation according to the new standards defined in `guide-documentation-standards.md`.

## `/docs` Folder Restructuring

### Files to Rename (Stay in `/docs`)

| Current File | New File | Description |
|-------------|----------|-------------|
| `architecture.md` | `arch-system-overview.md` | Main system architecture document |
| `gpu_arch.md` | `arch-gpu.md` | GPU architecture specification |
| `ipc_architecture.md` | `arch-ipc.md` | Inter-process communication architecture |
| `state_management.md` | `arch-state-management.md` | System state management architecture |
| `shared_memory_protocol.md` | `spec-shared-memory.md` | Shared memory protocol specification |
| `coding_guidelines.md` | `guide-coding-standards.md` | Coding standards and guidelines |
| `naming_convention.md` | `guide-naming-conventions.md` | Code and component naming conventions |
| `test_coverage_plan.md` | `plan-testing-strategy.md` | Test coverage strategy |
| `usage.md` | `guide-usage.md` | System usage guide |
| `DESIGN.md` | `arch-design-principles.md` | Core design principles |

### Files to Consolidate

| Files to Consolidate | New File | Action |
|---------------------|----------|--------|
| `api_refactoring.md`, `api_refactoring_plan.md`, `api_refactoring_progress.md` | `adr-api-refactoring.md` | Merge into a single Architecture Decision Record |
| `naming_convention.md`, `naming_convention_implementation.md` | `guide-naming-conventions.md` | Consolidate naming conventions into a single guide |
| `refactoring_plan.md`, `implementation_checklist.md` | `plan-implementation.md` | Create a unified implementation plan |
| `api_response_formats.md`, `api_compatibility.md` | `spec-api-formats.md` | Merge API specifications |
| `genome_connectome_sync.md` | `arch-genome-connectome.md` | Document synchronization architecture |

### Files to Move to Module Directories

| Current File | New Location | New Name |
|-------------|--------------|----------|
| `api_refactoring.md` | `/feagi/api/` | `README.md` (after consolidation with other API docs) |
| `feagi_processes.md` | `/feagi/core/` | `arch-processes.md` |
| `legacy_agent_registration.md` | `/feagi/api/` | `adr-agent-registration.md` |

### Files to Review/Archive

| File | Recommendation | Rationale |
|------|---------------|-----------|
| `FSMP.md` | Delete/Archive | Empty file |
| `quickstart.md` | Delete/Archive | Empty file |
| `installation.md` | Create proper content | Currently empty |
| `transition_summary.md` | Archive | Historical transition document |
| `versioning_implementation_summary.md` | Archive | Completed implementation |
| `cleanup_protocols_folder.md` | Archive | One-time cleanup notes |
| `audit.md` | Review for relevant content | May contain valuable information |

## Creation of New Documentation

### New System-Level Documents

| Document to Create | Description |
|-------------------|-------------|
| `README.md` | Overview of the documentation structure |
| `arch-overview.md` | High-level architectural overview |
| `guide-contribution.md` | Guidelines for contributing to FEAGI |
| `spec-protocols.md` | Consolidated protocols specification |

### New Module-Level Documents

| Module | Documents to Create |
|--------|-------------------|
| `/feagi/api/` | `README.md`, `guide-api-usage.md` |
| `/feagi/bdu/` | `README.md`, `arch-bdu.md` |
| `/feagi/npu/` | `README.md`, `arch-npu.md` |
| `/feagi_bytes/` | `README.md`, `guide-feagi-bytes.md` |
| `/feagi_connector/` | `README.md`, `guide-connector-usage.md` |

## Implementation Phases

1. **Phase 1: Create Foundation**
   - Set up documentation standards (✅ done)
   - Create this restructuring plan
   - Set up assets directory for diagrams

2. **Phase 2: Restructure `/docs` Folder**
   - Rename files according to new convention
   - Move module-specific files to their respective directories
   - Consolidate overlapping documents

3. **Phase 3: Update Content**
   - Review and update existing documentation for accuracy
   - Create missing documentation
   - Ensure cross-references are updated

4. **Phase 4: Create Module Documentation**
   - Add README.md to each module
   - Develop module-level documentation

## Next Steps

1. Create `/docs/assets` directory for diagrams
2. Begin Phase 2 by renaming files in `/docs`
3. Start consolidation of similar documents
