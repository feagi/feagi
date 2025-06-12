import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for Evolutionary module documentation
 */
const evoSidebar: SidebarsConfig = {
  docs: [
    {
      type: 'category',
      label: 'Evolution',
      items: [
        'spec-genome',
      ],
    },
  ],
};

export default evoSidebar;
