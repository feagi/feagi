import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for API module documentation
 */
const apiSidebar: SidebarsConfig = {
  docs: [
    {
      type: 'category',
      label: 'API',
      items: [
        'README', 
        'arch-api-design',
        'spec-implementation',
        'arch-server',
        'guide-api-testing',
        'adr-api-server-issues',
        'guide-api-usage'
      ],
    },
    {
      type: 'category',
      label: 'Protocols',
      items: [
        'protocols/arch-protocols',
        'protocols/adr-protocol-cleanup',
        'protocols/guide-protocol-migration',
      ],
    },
    {
      type: 'category',
      label: 'ZMQ',
      items: [
        'zmq/README',
        'zmq/arch-zmq',
        'zmq/spec-zmq-implementation',
      ],
    }
  ],
};

export default apiSidebar; 