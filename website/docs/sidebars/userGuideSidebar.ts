import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for end-user documentation
 */
const userGuideSidebar: SidebarsConfig = {
  docs: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: [
        'getting-started/installation',
        'getting-started/quick-start',
      ],
    },
    {
      type: 'category',
      label: 'User Guide',
      items: [
        'user-guide/configuration',
        'user-guide/visualization',
        'user-guide/agents',
        'user-guide/tutorials',
      ],
    },
  ],
};

export default userGuideSidebar;
