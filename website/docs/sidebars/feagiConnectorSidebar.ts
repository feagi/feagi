import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for FEAGI Connector documentation
 */
const feagiConnectorSidebar: SidebarsConfig = {
  docs: [
    {
      type: 'category',
      label: 'FEAGI Connector',
      items: [
        'README',
        'guide-connector-usage',
      ],
    },
  ],
};

export default feagiConnectorSidebar;
