"""Small topology/schema adapters; no control state machine or SDK calls."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Configuration:
    name: str
    arms: tuple
    camera_defaults: tuple

    @property
    def cameras(self):
        return tuple(row[0] for row in self.camera_defaults)

    @property
    def dimension(self):
        return 7 * len(self.arms)

    @property
    def names(self):
        if self.name == "single":
            from piper_recording_common import make_names
            return make_names(7)
        from unified_piper_storage import NAMES
        return list(NAMES)

    def configure_metadata(self, metadata):
        if self.name == "single":
            from unified_piper_single import configure_metadata
            configure_metadata(metadata)

    def add_sources(self, recorder):
        if self.name == "single":
            from unified_piper_single import add_sources
            add_sources(recorder)

    def episode(self, root, metadata, anchor):
        if self.name == "single" and metadata.get("cameras"):
            from unified_piper_single import SingleEpisode
            return SingleEpisode(root, metadata, anchor)
        from unified_piper_storage import Episode
        return Episode(root, metadata, anchor)


CONFIGURATIONS = {
    "dual": Configuration("dual", (("right", "R"), ("left", "L")),
        (("front", "/dev/video4", 1280, 720), ("wrist1", "/dev/video2", 640, 480),
         ("wrist2", "/dev/video0", 640, 480))),
    "single": Configuration("single", (("single", ""),),
        (("front", "/dev/video6", 640, 480), ("wrist", "/dev/video0", 640, 480))),
}


def configuration(name):
    return CONFIGURATIONS[name]
