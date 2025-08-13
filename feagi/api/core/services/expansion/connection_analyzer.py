"""
Connection Analyzer for Cortical Area Expansion

This module analyzes existing connectivity patterns and morphology usage
to support intelligent expansion decisions.
"""

from typing import Any, Dict, List, Optional

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class ConnectionAnalyzer:
    """
    Analyzes connectivity patterns and morphology usage for expansion planning.

    This class examines existing synaptic connections and morphology definitions
    to determine how expansion should handle dimension-sensitive vs dimension-agnostic
    connectivity patterns.
    """

    def __init__(self, connectome_manager, state_manager):
        """
        Initialize ConnectionAnalyzer.

        Args:
            connectome_manager: Reference to ConnectomeManager
            state_manager: Reference to FeagiStateManager
        """
        self.connectome_manager = connectome_manager
        self.state_manager = state_manager
        self.logger = logger

    def analyze_area_connectivity(self, cortical_id: str) -> Dict[str, Any]:
        """
        Analyze connectivity patterns for a cortical area.

        Args:
            cortical_id: ID of the cortical area to analyze

        Returns:
            Dictionary containing connectivity analysis results
        """
        self.logger.info(
            f"🔍 [CONNECTION-ANALYZER] Starting connectivity analysis for {cortical_id}"
        )

        try:
            # Get incoming and outgoing connections
            self.logger.info(
                "🔍 [CONNECTION-ANALYZER] Searching for mappings in genome..."
            )
            incoming_mappings = self._get_incoming_mappings(cortical_id)
            outgoing_mappings = self._get_outgoing_mappings(cortical_id)
            internal_mappings = self._get_internal_mappings(cortical_id)

            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER] Received from _get_internal_mappings: {len(internal_mappings)} mappings"
            )
            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER] Internal mappings content: {[m.get('morphology', 'unknown') for m in internal_mappings]}"
            )

            self.logger.info(
                "🔍 [CONNECTION-ANALYZER] Raw mapping search results:"
            )
            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER]   - Incoming: {len(incoming_mappings)}"
            )
            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER]   - Outgoing: {len(outgoing_mappings)}"
            )
            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER]   - Internal: {len(internal_mappings)}"
            )

            # Log details of internal mappings if found
            if internal_mappings:
                self.logger.info(
                    "🔍 [CONNECTION-ANALYZER] Internal mapping details:"
                )
                for i, mapping in enumerate(internal_mappings):
                    morphology = mapping.get("morphology", "unknown")
                    source = mapping.get("source", "unknown")
                    dest = mapping.get("destination", "unknown")
                    self.logger.info(
                        f"🔍 [CONNECTION-ANALYZER]   [{i + 1}] {source} -> {dest} ({morphology})"
                    )
            else:
                self.logger.warning(
                    f"🔍 [CONNECTION-ANALYZER] No internal mappings found for {cortical_id}"
                )

            # Log details of outgoing mappings if any
            if outgoing_mappings:
                self.logger.info(
                    "🔍 [CONNECTION-ANALYZER] Outgoing mapping details:"
                )
                for i, mapping in enumerate(
                    outgoing_mappings[:3]
                ):  # Show first 3
                    morphology = mapping.get("morphology", "unknown")
                    dest = mapping.get("destination", "unknown")
                    self.logger.info(
                        f"🔍 [CONNECTION-ANALYZER]   [{i + 1}] {cortical_id} -> {dest} ({morphology})"
                    )
                if len(outgoing_mappings) > 3:
                    self.logger.info(
                        f"🔍 [CONNECTION-ANALYZER]   ... and {len(outgoing_mappings) - 3} more"
                    )

            # Analyze morphology sensitivity (avoid double-counting)
            dimension_sensitive_mappings = []
            dimension_agnostic_mappings = []

            # Create a set to track unique mappings (avoid duplicates from internal mappings)
            unique_mappings = []
            seen_mappings = set()

            for mapping_list in [
                incoming_mappings,
                outgoing_mappings,
                internal_mappings,
            ]:
                for mapping in mapping_list:
                    # Create a unique key for each mapping
                    mapping_key = (
                        mapping.get("source"),
                        mapping.get("destination"),
                        mapping.get("morphology"),
                    )
                    if mapping_key not in seen_mappings:
                        seen_mappings.add(mapping_key)
                        unique_mappings.append(mapping)

            for mapping in unique_mappings:
                morphology_id = mapping.get("morphology")
                if morphology_id:
                    morphology_def = self._get_morphology_definition(
                        morphology_id
                    )
                    if morphology_def:
                        is_sensitive = morphology_def.get(
                            "dimension_sensitive", False
                        )
                        if is_sensitive:
                            dimension_sensitive_mappings.append(mapping)
                        else:
                            dimension_agnostic_mappings.append(mapping)

            analysis = {
                "cortical_id": cortical_id,
                "total_mappings": len(unique_mappings),
                "incoming_count": len(incoming_mappings),
                "outgoing_count": len(outgoing_mappings),
                "internal_count": len(internal_mappings),
                "dimension_sensitive_count": len(dimension_sensitive_mappings),
                "dimension_agnostic_count": len(dimension_agnostic_mappings),
                "incoming_mappings": incoming_mappings,
                "outgoing_mappings": outgoing_mappings,
                "internal_mappings": internal_mappings,
                "dimension_sensitive_mappings": dimension_sensitive_mappings,
                "dimension_agnostic_mappings": dimension_agnostic_mappings,
            }

            self.logger.info(
                f"[CONNECTION-ANALYZER] Analysis complete for {cortical_id}:"
            )
            self.logger.info(f"  Total mappings: {analysis['total_mappings']}")
            self.logger.info(
                f"  Dimension-sensitive: {analysis['dimension_sensitive_count']}"
            )
            self.logger.info(
                f"  Dimension-agnostic: {analysis['dimension_agnostic_count']}"
            )
            self.logger.info(
                f"  Internal mappings: {analysis['internal_count']}"
            )

            return analysis

        except Exception as e:
            self.logger.error(
                f"[CONNECTION-ANALYZER] Error analyzing {cortical_id}: {e}"
            )
            return {
                "cortical_id": cortical_id,
                "error": str(e),
                "total_mappings": 0,
            }

    def _get_incoming_mappings(self, cortical_id: str) -> List[Dict[str, Any]]:
        """Get mappings where this area is the destination (excluding internal mappings)."""
        mappings = []
        try:
            genome = self.state_manager.genome
            if genome and "blueprint" in genome:
                blueprint = genome["blueprint"]

                # Search through all cortical areas for mappings TO this cortical_id
                for source_area_id, area_data in blueprint.items():
                    if (
                        isinstance(area_data, dict)
                        and "cortical_mapping_dst" in area_data
                    ):
                        cortical_mappings = area_data["cortical_mapping_dst"]

                        # Check if this area has mappings to our target cortical_id
                        if (
                            isinstance(cortical_mappings, dict)
                            and cortical_id in cortical_mappings
                        ):
                            connection_specs = cortical_mappings[cortical_id]

                            if isinstance(connection_specs, list):
                                for spec in connection_specs:
                                    # CRITICAL FIX: Handle both dict and array formats
                                    if isinstance(spec, dict):
                                        # Dict format
                                        if (
                                            source_area_id != cortical_id
                                        ):  # Only include if it's not an internal mapping
                                            mapping = {
                                                "source": source_area_id,
                                                "destination": cortical_id,
                                                "morphology": spec.get(
                                                    "morphology_id"
                                                ),
                                                "spec": spec,
                                            }
                                            mappings.append(mapping)
                                    elif (
                                        isinstance(spec, list)
                                        and len(spec) >= 1
                                    ):
                                        # Array format: ["lateral_+x", [1, 1, 1], 1.0, False, 1, 1, 1]
                                        if (
                                            source_area_id != cortical_id
                                        ):  # Only include if it's not an internal mapping
                                            morphology_id = (
                                                spec[0]
                                                if len(spec) > 0
                                                else None
                                            )
                                            mapping = {
                                                "source": source_area_id,
                                                "destination": cortical_id,
                                                "morphology": morphology_id,
                                                "spec": spec,
                                            }
                                            mappings.append(mapping)

            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER] Found {len(mappings)} incoming mappings for {cortical_id}"
            )
        except Exception as e:
            self.logger.error(
                f"[CONNECTION-ANALYZER] Error getting incoming mappings: {e}"
            )
        return mappings

    def _get_outgoing_mappings(self, cortical_id: str) -> List[Dict[str, Any]]:
        """Get mappings where this area is the source."""
        mappings = []
        try:
            genome = self.state_manager.genome
            if genome and "blueprint" in genome:
                blueprint = genome["blueprint"]

                # Check if this cortical area has outgoing mappings
                if cortical_id in blueprint:
                    area_data = blueprint[cortical_id]
                    if (
                        isinstance(area_data, dict)
                        and "cortical_mapping_dst" in area_data
                    ):
                        cortical_mappings = area_data["cortical_mapping_dst"]

                        if isinstance(cortical_mappings, dict):
                            for (
                                dst_area_id,
                                connection_specs,
                            ) in cortical_mappings.items():
                                if isinstance(connection_specs, list):
                                    for spec in connection_specs:
                                        # CRITICAL FIX: Handle both dict and array formats
                                        if isinstance(spec, dict):
                                            # Dict format
                                            if (
                                                dst_area_id != cortical_id
                                            ):  # Only include if it's not an internal mapping
                                                mapping = {
                                                    "source": cortical_id,
                                                    "destination": dst_area_id,
                                                    "morphology": spec.get(
                                                        "morphology_id"
                                                    ),
                                                    "spec": spec,
                                                }
                                                mappings.append(mapping)
                                        elif (
                                            isinstance(spec, list)
                                            and len(spec) >= 1
                                        ):
                                            # Array format: ["lateral_+x", [1, 1, 1], 1.0, False, 1, 1, 1]
                                            if (
                                                dst_area_id != cortical_id
                                            ):  # Only include if it's not an internal mapping
                                                morphology_id = (
                                                    spec[0]
                                                    if len(spec) > 0
                                                    else None
                                                )
                                                mapping = {
                                                    "source": cortical_id,
                                                    "destination": dst_area_id,
                                                    "morphology": morphology_id,
                                                    "spec": spec,
                                                }
                                                mappings.append(mapping)

            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER] Found {len(mappings)} outgoing mappings for {cortical_id}"
            )
        except Exception as e:
            self.logger.error(
                f"[CONNECTION-ANALYZER] Error getting outgoing mappings: {e}"
            )
        return mappings

    def _get_internal_mappings(self, cortical_id: str) -> List[Dict[str, Any]]:
        """Get internal mappings within this area (source == destination)."""
        mappings = []
        try:
            genome = self.state_manager.genome

            # Add detailed debugging
            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER] Debugging genome structure for {cortical_id}"
            )
            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER] Genome type: {type(genome)}"
            )

            if genome:
                self.logger.info(
                    f"🔍 [CONNECTION-ANALYZER] Genome keys: {list(genome.keys())}"
                )

                if "blueprint" in genome:
                    blueprint = genome["blueprint"]
                    self.logger.info(
                        f"🔍 [CONNECTION-ANALYZER] Blueprint type: {type(blueprint)}"
                    )
                    self.logger.info(
                        f"🔍 [CONNECTION-ANALYZER] Blueprint keys: {list(blueprint.keys()) if isinstance(blueprint, dict) else 'not a dict'}"
                    )

                    # Check if this cortical area has internal mappings (mappings to itself)
                    if cortical_id in blueprint:
                        area_data = blueprint[cortical_id]
                        self.logger.info(
                            f"🔍 [CONNECTION-ANALYZER] Found {cortical_id} in blueprint"
                        )
                        self.logger.info(
                            f"🔍 [CONNECTION-ANALYZER] Area data type: {type(area_data)}"
                        )
                        self.logger.info(
                            f"🔍 [CONNECTION-ANALYZER] Area data keys: {list(area_data.keys()) if isinstance(area_data, dict) else 'not a dict'}"
                        )

                        if (
                            isinstance(area_data, dict)
                            and "cortical_mapping_dst" in area_data
                        ):
                            cortical_mappings = area_data[
                                "cortical_mapping_dst"
                            ]
                            self.logger.info(
                                "🔍 [CONNECTION-ANALYZER] Found cortical_mapping_dst"
                            )
                            self.logger.info(
                                f"🔍 [CONNECTION-ANALYZER] Mapping data type: {type(cortical_mappings)}"
                            )
                            self.logger.info(
                                f"🔍 [CONNECTION-ANALYZER] Mapping data: {cortical_mappings}"
                            )

                            # Check for mappings to itself (internal mappings)
                            if (
                                isinstance(cortical_mappings, dict)
                                and cortical_id in cortical_mappings
                            ):
                                connection_specs = cortical_mappings[
                                    cortical_id
                                ]
                                self.logger.info(
                                    "🔍 [CONNECTION-ANALYZER] Found internal mappings!"
                                )
                                self.logger.info(
                                    f"🔍 [CONNECTION-ANALYZER] Connection specs type: {type(connection_specs)}"
                                )
                                self.logger.info(
                                    f"🔍 [CONNECTION-ANALYZER] Connection specs: {connection_specs}"
                                )

                                if isinstance(connection_specs, list):
                                    for spec in connection_specs:
                                        # CRITICAL FIX: Handle both dict and array formats
                                        if isinstance(spec, dict):
                                            # Dict format: {"morphology_id": "lateral_+x", ...}
                                            mapping = {
                                                "source": cortical_id,
                                                "destination": cortical_id,
                                                "morphology": spec.get(
                                                    "morphology_id"
                                                ),
                                                "spec": spec,
                                            }
                                            mappings.append(mapping)
                                            self.logger.info(
                                                f"🔍 [CONNECTION-ANALYZER] Added internal mapping (dict): {mapping}"
                                            )
                                        elif (
                                            isinstance(spec, list)
                                            and len(spec) >= 1
                                        ):
                                            try:
                                                # Array format: ["lateral_+x", [1, 1, 1], 1.0, False, 1, 1, 1]
                                                morphology_id = (
                                                    spec[0]
                                                    if len(spec) > 0
                                                    else None
                                                )
                                                mapping = {
                                                    "source": cortical_id,
                                                    "destination": cortical_id,
                                                    "morphology": morphology_id,
                                                    "spec": spec,
                                                }
                                                mappings.append(mapping)
                                                self.logger.info(
                                                    f"🔍 [CONNECTION-ANALYZER] Added internal mapping (array): {mapping}"
                                                )
                                                self.logger.info(
                                                    f"🔍 [CONNECTION-ANALYZER] Current mappings count: {len(mappings)}"
                                                )
                                            except Exception as mapping_ex:
                                                self.logger.error(
                                                    f"🔍 [CONNECTION-ANALYZER] Exception adding internal mapping: {mapping_ex}"
                                                )
                                                self.logger.error(
                                                    f"🔍 [CONNECTION-ANALYZER] Spec that caused error: {spec}"
                                                )
                                                import traceback

                                                self.logger.error(
                                                    f"🔍 [CONNECTION-ANALYZER] Traceback: {traceback.format_exc()}"
                                                )
                                        else:
                                            self.logger.warning(
                                                f"🔍 [CONNECTION-ANALYZER] Unknown spec format: {type(spec)} - {spec}"
                                            )
                            else:
                                self.logger.info(
                                    f"🔍 [CONNECTION-ANALYZER] No self-mapping found. Available destinations: {list(cortical_mappings.keys()) if isinstance(cortical_mappings, dict) else 'not a dict'}"
                                )
                        else:
                            self.logger.info(
                                "🔍 [CONNECTION-ANALYZER] No cortical_mapping_dst found in area data"
                            )
                    else:
                        self.logger.info(
                            f"🔍 [CONNECTION-ANALYZER] {cortical_id} not found in blueprint"
                        )
                        self.logger.info(
                            f"🔍 [CONNECTION-ANALYZER] Available cortical areas: {list(blueprint.keys()) if isinstance(blueprint, dict) else 'not a dict'}"
                        )
                else:
                    self.logger.info(
                        "🔍 [CONNECTION-ANALYZER] No blueprint found in genome"
                    )
            else:
                self.logger.info(
                    "🔍 [CONNECTION-ANALYZER] No genome available"
                )

            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER] Found {len(mappings)} internal mappings for {cortical_id}"
            )
            self.logger.info(
                f"🔍 [CONNECTION-ANALYZER] About to return mappings: {[m.get('morphology', 'unknown') for m in mappings]}"
            )
        except Exception as e:
            self.logger.error(
                f"[CONNECTION-ANALYZER] Error getting internal mappings: {e}"
            )
            import traceback

            self.logger.error(
                f"[CONNECTION-ANALYZER] Traceback: {traceback.format_exc()}"
            )
            mappings = []  # Return empty list on error

        self.logger.info(
            f"🔍 [CONNECTION-ANALYZER] Final return: {len(mappings)} mappings"
        )
        return mappings

    def _get_morphology_definition(
        self, morphology_id: str
    ) -> Optional[Dict[str, Any]]:
        """Get morphology definition from genome."""
        try:
            genome = self.state_manager.genome
            if genome and "neuron_morphologies" in genome:
                return genome["neuron_morphologies"].get(morphology_id)
        except Exception as e:
            self.logger.error(
                f"[CONNECTION-ANALYZER] Error getting morphology {morphology_id}: {e}"
            )
        return None

    def get_expansion_recommendation(self, cortical_id: str) -> Dict[str, Any]:
        """
        Get recommendations for how to handle expansion for this area.

        Args:
            cortical_id: ID of the cortical area

        Returns:
            Dictionary with expansion recommendations
        """
        analysis = self.analyze_area_connectivity(cortical_id)

        recommendation = {
            "cortical_id": cortical_id,
            "should_preserve_patterns": analysis["dimension_agnostic_count"]
            > 0,
            "should_reconstruct_patterns": analysis[
                "dimension_sensitive_count"
            ]
            > 0,
            "has_internal_connectivity": analysis["internal_count"] > 0,
            "preservation_candidates": analysis.get(
                "dimension_agnostic_mappings", []
            ),
            "reconstruction_candidates": analysis.get(
                "dimension_sensitive_mappings", []
            ),
        }

        self.logger.info(
            f"[CONNECTION-ANALYZER] Expansion recommendation for {cortical_id}:"
        )
        self.logger.info(
            f"  Preserve patterns: {recommendation['should_preserve_patterns']}"
        )
        self.logger.info(
            f"  Reconstruct patterns: {recommendation['should_reconstruct_patterns']}"
        )
        self.logger.info(
            f"  Has internal connectivity: {recommendation['has_internal_connectivity']}"
        )

        return recommendation
