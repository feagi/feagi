import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for PNS module documentation
 */
const pnsSidebar: SidebarsConfig = {
  docs: [
    {
      type: 'category',
      label: 'PNS',
      items: [
        'README',
        'arch-pns',
      ],
    },
  ],
};

export default pnsSidebar; 