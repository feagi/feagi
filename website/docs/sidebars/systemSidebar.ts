import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for system-level documentation
 */
const systemSidebar: SidebarsConfig = {
  docs: [
    {
      type: 'category',
      label: 'Overview',
      items: [
        'arch-system-overview',
        'arch-overview',
        'arch-design-principles',
        'arch-system-diagrams',
      ],
    },
    {
      type: 'category',
      label: 'Architecture',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Core Systems',
          items: [
            'arch-data-structures',
            'arch-state-management',
            'arch-genome-connectome',
            'arch-burst-engine-lifecycle',
          ],
        },
        {
          type: 'category',
          label: 'Communication & Protocols',
          items: [
            'arch-protocols',
            'arch-ipc',
            'arch-zmq',
            'arch-api-decorator-architecture',
          ],
        },
        {
          type: 'category',
          label: 'Performance & Hardware',
          items: [
            'arch-gpu',
            'arch-gpu-optimization',
            'arch-embedded-mode',
          ],
        },
        {
          type: 'category',
          label: 'Integration',
          items: [
            'arch-godot-bridge-integration',
          ],
        },
        {
          type: 'category',
          label: 'Migration & Compatibility',
          items: [
            'arch-rust-rtos-migration',
            'rust-rtos-migration-guide',
            'npu_rtos_rust_compatibility',
            'npu_wgpu_compatibility',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Technical Specifications',
      items: [
        'spec-api-formats',
        'spec-protocols',
        'spec-shared-memory',
      ],
    },
    {
      type: 'category',
      label: 'Implementation Reports',
      items: [
        'wgpu-implementation-summary',
        'simd-optimization-assessment',
        'simd-implementation-report',
      ],
    },
    {
      type: 'category',
      label: 'Developer Guides',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Getting Started',
          items: [
            'guide-installation',
            'guide-usage',
            'guide-how-to-debug',
          ],
        },
        {
          type: 'category',
          label: 'Standards & Guidelines',
          items: [
            'guide-documentation-standards',
            'guide-documentation-structure',
            'guide-coding-standards',
            'guide-naming-conventions',
          ],
        },
        {
          type: 'category',
          label: 'Contributing',
          items: [
            'guide-contribution',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Architecture Decision Records',
      items: [
        'adr-api-refactoring',
        'adr-visualization-threading-enhancement',
      ],
    },
    {
      type: 'category',
      label: 'Project Planning',
      items: [
        'plan-documentation-restructuring',
        'plan-testing-strategy',
      ],
    },
  ],
};

export default systemSidebar; 