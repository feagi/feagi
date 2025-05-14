class CoreAPIService:
    def get_processed_genome(self):
        """
        Returns the post-processed genome with proper cortical area hierarchies.
        
        Unlike get_genome() which returns the raw genome data, this method returns
        the genome after all processing and hierarchicalization has been applied.
        
        Returns:
            dict: The processed genome with all cortical areas properly structured
        """
        # Get the connectome manager which should have the processed genome
        connectome = self.get_connectome()
        if connectome:
            # Return connectome's version of the genome if available
            return getattr(connectome, 'genome', None)
        
        return None
    
    def get_cortical_area_ids(self):
        """
        Returns the list of 6-letter cortical area IDs.
        
        These are the string identifiers (e.g., "ABCDEF") used for external APIs
        and user interfaces.
        
        Returns:
            list: A list of all 6-letter cortical area IDs
        """
        connectome = self.get_connectome()
        if connectome and hasattr(connectome, '_areas'):
            return list(connectome._areas.keys())
        return []
    
    def get_cortical_area_indices(self):
        """
        Returns the list of numerical indices for cortical areas.
        
        These are the integer indices (e.g., 0, 1, 2) used internally by the FCL
        and other performance-critical components.
        
        Returns:
            list: A list of all cortical area indices
        """
        connectome = self.get_connectome()
        
        # Try different possible attribute names based on the documentation
        for attr_name in ['cortical_indices', 'area_indices', 'fcl_indices']:
            if hasattr(connectome, attr_name):
                indices = getattr(connectome, attr_name)
                if indices:
                    return list(indices)
        
        # If no direct list is found, try to extract from a mapping
        for attr_name in ['cortical_index_map', 'area_index_map', 'id_to_index_map']:
            if hasattr(connectome, attr_name):
                mapping = getattr(connectome, attr_name)
                if mapping:
                    return list(mapping.values())
        
        # Last resort: if there's a method to get indices
        if hasattr(connectome, 'get_cortical_indices'):
            return connectome.get_cortical_indices()
        
        return []
    
    def get_connectome(self):
        """
        Gets the connectome manager instance.
        
        Returns:
            ConnectomeManager or None: The connectome manager instance, or None if unavailable
        """
        # Use the existing dependency injection system if possible
        from feagi.api.rest.dependencies import get_connectome
        try:
            return get_connectome()
        except:
            # Fall back to direct import if dependency injection isn't available
            from feagi.bdu import ConnectomeManager
            return ConnectomeManager.instance()

    def _get_connectome_manager(self):
        """
        Helper method to get the connectome manager instance.
        
        Returns:
            ConnectomeManager or None: The connectome manager instance, or None if unavailable
        """
        # Implementation depends on how the connectome manager is accessed in the current architecture
        # This might involve importing a singleton or getting it from another service
        from feagi.bdu import get_connectome_manager
        return get_connectome_manager() 