import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for system-level documentation
 */
const systemSidebar: SidebarsConfig = {
  docs: [
    {
      type: 'category',
      label: 'Architecture',
      items: [
        'arch-system-overview',
        'arch-overview',
        'arch-design-principles',
        'arch-gpu',
        'arch-ipc',
        'arch-state-management',
        'arch-genome-connectome',
        'arch-zmq',
        'system-architecture',
      ],
    },
    {
      type: 'category',
      label: 'Specifications',
      items: [
        'spec-api-formats',
        'spec-protocols',
        'spec-shared-memory',
      ],
    },
    {
      type: 'category',
      label: 'Guides',
      items: [
        'guide-documentation-standards',
        'guide-coding-standards',
        'guide-naming-conventions',
        'guide-usage',
        'guide-contribution',
        'installation',
      ],
    },
    {
      type: 'category',
      label: 'Architecture Decisions',
      items: [
        'adr-api-refactoring',
      ],
    },
    {
      type: 'category',
      label: 'Planning',
      items: [
        'plan-documentation-restructuring',
        'plan-testing-strategy',
      ],
    },
  ],
};

export default systemSidebar; 