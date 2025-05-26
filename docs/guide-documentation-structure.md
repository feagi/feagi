# FEAGI Documentation Structure Diagram

*Last Updated: May 15, 2025*

The following diagram illustrates the new FEAGI documentation structure.

## System Documentation Structure

```mermaid
graph TD
    subgraph "/docs - System Documentation"
        README["/docs/README.md"] --> ARCH["Architecture (arch-*)"]
        README --> SPEC["Specifications (spec-*)"]
        README --> GUIDE["Guides (guide-*)"] 
        README --> ADR["Architecture Decision Records (adr-*)"]
        README --> PLAN["Planning Documents (plan-*)"]
        
        ARCH --> ARCH_SYS["arch-system-overview.md"]
        ARCH --> ARCH_GPU["arch-gpu.md"]
        ARCH --> ARCH_IPC["arch-ipc.md"]
        ARCH --> ARCH_STATE["arch-state-management.md"]
        ARCH --> ARCH_DESIGN["arch-design-principles.md"]
        
        SPEC --> SPEC_SHM["spec-shared-memory.md"]
        SPEC --> SPEC_API["spec-api-formats.md"]
        SPEC --> SPEC_PROT["spec-protocols.md"]
        
        GUIDE --> GUIDE_DOC["guide-documentation-standards.md"]
        GUIDE --> GUIDE_CODE["guide-coding-standards.md"]
        GUIDE --> GUIDE_NAME["guide-naming-conventions.md"]
        GUIDE --> GUIDE_USE["guide-usage.md"]
        
        ADR --> ADR_API["adr-api-refactoring.md"]
        
        PLAN --> PLAN_TEST["plan-testing-strategy.md"]
        PLAN --> PLAN_DOC["plan-documentation-restructuring.md"]
        PLAN --> PLAN_IMPL["plan-implementation.md"]
    end
    
    style README fill:#f9f,stroke:#333,stroke-width:2px
    style ARCH fill:#bbf,stroke:#333,stroke-width:1px
    style SPEC fill:#bfb,stroke:#333,stroke-width:1px
    style GUIDE fill:#fbb,stroke:#333,stroke-width:1px
    style ADR fill:#fdb,stroke:#333,stroke-width:1px
    style PLAN fill:#dbf,stroke:#333,stroke-width:1px
```

## Module-Level Documentation Structure

```mermaid
graph TD
    subgraph "Module Documentation"
        MOD_ROOT["FEAGI Root"] --> FEAGI["feagi/"]
        MOD_ROOT --> BYTES["feagi_bytes/"]
        MOD_ROOT --> CONN["feagi_connector/"]
        
        FEAGI --> API["api/"]
        FEAGI --> BDU["bdu/"]
        FEAGI --> NPU["npu/"]
        FEAGI --> CORE["core/"]
        
        API --> API_README["README.md"]
        API --> API_GUIDE["guide-api-usage.md"]
        
        BDU --> BDU_README["README.md"]
        BDU --> BDU_ARCH["arch-bdu.md"]
        
        NPU --> NPU_README["README.md"]
        NPU --> NPU_ARCH["arch-npu.md"]
        
        BYTES --> BYTES_README["README.md"]
        BYTES --> BYTES_GUIDE["guide-feagi-bytes.md"]
        
        CONN --> CONN_README["README.md"]
        CONN --> CONN_GUIDE["guide-connector-usage.md"]
    end
    
    style MOD_ROOT fill:#ddd,stroke:#333,stroke-width:2px
    style API_README fill:#f9f,stroke:#333,stroke-width:2px
    style BDU_README fill:#f9f,stroke:#333,stroke-width:2px
    style NPU_README fill:#f9f,stroke:#333,stroke-width:2px
    style BYTES_README fill:#f9f,stroke:#333,stroke-width:2px
    style CONN_README fill:#f9f,stroke:#333,stroke-width:2px
    style API_GUIDE fill:#fbb,stroke:#333,stroke-width:1px
    style BYTES_GUIDE fill:#fbb,stroke:#333,stroke-width:1px
    style CONN_GUIDE fill:#fbb,stroke:#333,stroke-width:1px
    style BDU_ARCH fill:#bbf,stroke:#333,stroke-width:1px
    style NPU_ARCH fill:#bbf,stroke:#333,stroke-width:1px
```

## Documentation Category Legend

| Category | Prefix | Color | Purpose |
|----------|--------|-------|---------|
| Architecture | `arch-` | Blue | System design and component architecture |
| Specification | `spec-` | Green | Technical specifications and protocols |
| Guide | `guide-` | Red | Usage guides and standards |
| ADR | `adr-` | Orange | Architecture Decision Records |
| Planning | `plan-` | Purple | Project planning documents |
| README | `README.md` | Pink | Directory overviews | 