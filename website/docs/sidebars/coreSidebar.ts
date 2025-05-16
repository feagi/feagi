import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for Core module documentation
 */
const coreSidebar: SidebarsConfig = {
  docs: [
    {
      type: 'category',
      label: 'Core',
      items: [
        'README',
        'arch-core',
        'arch-state-manager',
      ],
    },
  ],
};

export default coreSidebar; 