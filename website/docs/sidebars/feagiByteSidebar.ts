import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for FEAGI Bytes documentation
 */
const feagiByteSidebar: SidebarsConfig = {
  docs: [
    {
      type: 'category',
      label: 'FEAGI Bytes',
      items: [
        'README',
        'guide-feagi-bytes',
      ],
    },
  ],
};

export default feagiByteSidebar; 