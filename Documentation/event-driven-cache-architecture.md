# Event-Driven Cache Architecture

## Overview

FEAGI 2.0 implements a sophisticated event-driven cache system that eliminates time-based expiry guessing and ensures data consistency across all services. This architecture follows the producer-responsibility pattern where data producers (ConnectomeManager) are responsible for maintaining cache accuracy.

## Architecture Principles

### 1. **Single Source of Truth**
- **StateManager** is the only location where cached data is stored
- All services read from StateManager's centralized cache
- No duplicate cache storage across multiple services

### 2. **Producer Responsibility**
- **ConnectomeManager** triggers cache updates on every data modification
- Cache updates happen immediately when underlying data changes
- No polling or periodic refresh mechanisms

### 3. **Zero Time-Based Expiry**
- No hardcoded timeout values (e.g., 30-second expiry)
- No guessing when data might be stale
- Cache is always accurate or explicitly marked as dirty

### 4. **Event-Driven Updates**
- Cache invalidation triggered by actual data change events
- Immediate consistency between cache and underlying data
- No eventual consistency delays

## Cache Types

### Cortical Areas Cache
- **Location**: `StateManager._cortical_areas_cache`
- **Purpose**: Caches all cortical area properties for API endpoints
- **Triggers**: Updated when cortical areas are added, modified, deleted, or their mappings change
- **Fallback**: Direct ConnectomeManager access (with warning) if StateManager unavailable

### Memory Area Cache (FCL Window Size)
- **Location**: `StateManager._memory_area_cache` (FCLWindowSizeCache)
- **Purpose**: Caches FCL window size computations for memory areas
- **Triggers**: Updated when memory areas registered/unregistered or temporal depth changes
- **Fallback**: None (always required for proper memory system operation)

## Implementation Details

### StateManager Cache Methods

```python
# Cache management
def invalidate_cortical_areas_cache(self) -> None
def get_cortical_areas_cache(self, connectome_manager=None) -> List[Dict]
def update_cortical_areas_cache(self, cortical_id: str, operation: str) -> None
```

### ConnectomeManager Trigger Points

1. **add_cortical_area()** → `update_cortical_areas_cache(area_id, 'add')`
2. **update_cortical_area_properties()** → `update_cortical_areas_cache(cortical_id, 'update')`
3. **delete_cortical_area()** → `update_cortical_areas_cache(cortical_id, 'delete')`
4. **update_cortical_connection()** → `update_cortical_areas_cache(source_id, 'mapping_update')`

### NeuroEmbryogenesis Trigger Points

1. **Direct mapping updates** → `update_cortical_areas_cache(src_area_id, 'mapping_update')`
   - When `src_area.properties["mapping"]` is directly modified

### GenomeService Trigger Points

1. **update_cortical_mapping()** → `state_manager.invalidate_cortical_areas_cache()`
   - After successful mapping updates through genome service

## Data Flow

```
API Request → GenomeService → NeuroEmbryogenesis → ConnectomeManager
                    ↓              ↓                    ↓
              StateManager.invalidate_cache()    StateManager.update_cache()
                    ↓              ↓                    ↓
                       StateManager (Single Source of Truth)
                                    ↓
                         CorticalAreaService.get_all_areas()
                                    ↓
                              API Response
```

## Benefits

### 1. **Eliminates Stale Data**
- Cache is updated immediately when data changes
- No window where API returns outdated information
- Consistent behavior across all endpoints

### 2. **Performance Optimization**
- Expensive `get_all_cortical_area_properties()` calls are cached
- Cache hits avoid ConnectomeManager traversal overhead
- Minimal memory overhead with event-driven invalidation

### 3. **Architectural Compliance**
- Follows FEAGI 2.0 "no fallbacks" principle
- Producer-driven consistency model
- Single source of truth pattern

### 4. **Debugging and Monitoring**
- Clear cache invalidation logging
- Operation-specific cache update tracking
- Explicit cache hit/miss reporting

## Migration from Time-Based Cache

### Before (Problematic)
```python
# CorticalAreaService (OLD)
cache_max_age = 30  # seconds - HARDCODED TIMEOUT
current_time = time.time()
if (current_time - self._cortical_areas_cache_timestamp) < cache_max_age:
    return self._cortical_areas_cache  # MAY BE STALE
```

### After (Event-Driven)
```python
# StateManager + ConnectomeManager (NEW)
# In ConnectomeManager:
state_manager.update_cortical_areas_cache(cortical_id, 'add')

# In CorticalAreaService:
return self.state_manager.get_cortical_areas_cache(self._connectome_manager)
```

## Testing and Validation

### Cache Accuracy Tests
- Verify cache updates trigger on all ConnectomeManager modifications
- Confirm API endpoints return fresh data immediately after changes
- Test cache invalidation across service boundaries

### Performance Tests
- Measure cache hit rates and response time improvements
- Validate memory usage remains bounded
- Benchmark against direct ConnectomeManager access

## Future Enhancements

### 1. **Selective Cache Updates**
- Instead of full invalidation, update specific cache entries
- Maintain change logs for granular cache management

### 2. **Cache Warming**
- Pre-populate cache during system initialization
- Background refresh for frequently accessed data

### 3. **Cache Analytics**
- Hit/miss ratio monitoring
- Performance impact measurement
- Cache size optimization

## Troubleshooting

### Common Issues

1. **Cache Not Updating**: Check if ConnectomeManager modification calls `update_cortical_areas_cache()`
2. **Stale Data**: Verify StateManager.invalidate_cortical_areas_cache() is called after changes
3. **Performance Degradation**: Monitor cache hit ratios and invalidation frequency

### Debug Logging
- `[CACHE]` prefix for all cache-related operations
- Operation-specific cache update logging
- Cache hit/miss tracking in service layers

## Conclusion

The event-driven cache architecture provides FEAGI 2.0 with a robust, performant, and architecturally compliant caching system. By eliminating time-based expiry and implementing producer-driven updates, the system ensures data consistency while optimizing performance for frequently accessed cortical area data. 