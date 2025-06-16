import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for NPU module documentation
 */
const npuSidebar: SidebarsConfig = {
  docs: [
    {
      type: 'category',
      label: 'NPU',
      items: [
        'README',
        'arch-npu',
      ],
    },
  ],
};

export default npuSidebar;
