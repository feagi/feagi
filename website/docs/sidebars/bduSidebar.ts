import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for BDU module documentation
 */
const bduSidebar: SidebarsConfig = {
  docs: [
    {
      type: 'category',
      label: 'BDU',
      items: [
        'arch-bdu-design',
      ],
    },
    {
      type: 'category',
      label: 'Specifications',
      items: [
        'docs/spec-brain-region',
        'docs/spec-connectivity-rules',
        'docs/spec-connectome',
        'docs/spec-cortical-mapping',
        'models/spec-cortical-area',
        'models/spec-neuron',
        'models/spec-synapse',
      ],
    }
  ],
};

export default bduSidebar;
