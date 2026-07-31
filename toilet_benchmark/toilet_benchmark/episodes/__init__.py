"""Episode schema and validation contracts."""

from .schema import (
    BENCHMARK_SCHEMA_VERSION,
    CollisionPolicy,
    EpisodeSpec,
    TrackType,
    episode_from_manual_selection,
    episodes_from_manual_config,
)
from .validator import (
    EpisodeValidationError,
    ValidationErrorCode,
    require_valid_episode,
    validate_episode,
)
from .manifest import (
    EpisodeManifest,
    ManifestEpisode,
    EPISODE_MANIFEST_SCHEMA_VERSION,
    build_episode_manifest,
    compute_episode_hash,
    dump_episode_spec,
    dump_episode_manifest,
    fill_episode_hashes,
    load_episode_spec,
    load_episode_manifest,
)
from .splits import SplitAssignment, SplitValidationError, assign_episodes_to_splits

__all__ = [
    "BENCHMARK_SCHEMA_VERSION",
    "CollisionPolicy",
    "EpisodeSpec",
    "TrackType",
    "episode_from_manual_selection",
    "episodes_from_manual_config",
    "EpisodeValidationError",
    "ValidationErrorCode",
    "require_valid_episode",
    "validate_episode",
    "EPISODE_MANIFEST_SCHEMA_VERSION",
    "EpisodeManifest",
    "ManifestEpisode",
    "build_episode_manifest",
    "compute_episode_hash",
    "dump_episode_spec",
    "dump_episode_manifest",
    "fill_episode_hashes",
    "load_episode_spec",
    "load_episode_manifest",
    "SplitAssignment",
    "SplitValidationError",
    "assign_episodes_to_splits",
]
