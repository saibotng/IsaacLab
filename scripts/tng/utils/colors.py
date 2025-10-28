from enum import Enum
import random
from typing import Tuple, List, Optional

RGB = Tuple[float, float, float]


def rgb_from_255(rgb255: Tuple[int, int, int]) -> RGB:
    r, g, b = rgb255
    return (r / 255.0, g / 255.0, b / 255.0)


class Color(Enum):
    BLACK      = (0.0, 0.0, 0.0)
    WHITE      = (1.0, 1.0, 1.0)
    RED         = rgb_from_255((178, 0,   0))
    PINK        = rgb_from_255((255, 51, 153))
    PURPLE      = rgb_from_255((50,  0, 100))
    BLUE       = rgb_from_255((0, 0, 150))
    CYAN       = rgb_from_255((0, 210, 210))
    TEAL        = rgb_from_255((0,  40, 40))
    GREEN       = rgb_from_255((0,  40,   0))
    LIME        = rgb_from_255((0, 255,  0))
    YELLOW      = rgb_from_255((255, 255, 0))
    ORANGE      = rgb_from_255((200, 50,   0))
    BROWN       = rgb_from_255((50,  22, 17))
    GREY        = rgb_from_255((50,  50,  50))





    @property
    def rgb(self) -> RGB:
        return self.value

    @classmethod
    def sample(
        cls,
        n: int,
        rng: random.Random,
        *,
        exclude: Optional[List["Color"]] = None,
    ) -> List["Color"]:
        """Sample n distinct colors, optionally excluding some."""
        exclude_set = set(exclude or [])
        pool = [c for c in cls if c not in exclude_set]

        if n > len(pool):
            raise ValueError(
                f"Requested {n} colors but only {len(pool)} available after exclusions."
            )

        # Use global random state since seed is already set in calling scripts
        return rng.sample(pool, n)

    @classmethod
    def from_rgb(cls, rgb: RGB) -> "Color":
        """Get the Color enum member corresponding to an RGB value."""
        for color in cls:
            if color.rgb == rgb:
                return color
        raise ValueError(f"No color found for RGB value {rgb}")

    @property
    def pretty(self) -> str:
        """Readable name, e.g. 'Deep Purple' for Color.DEEP_PURPLE."""
        return self.name.replace("_", " ").lower()
    
    @classmethod
    def get_excluded_colors_for_limited_set(cls) -> List["Color"]:
        """Returns a list of colors that should be excluded when limit_colors is True."""
        return [Color.PINK, Color.LIME, Color.ORANGE, Color.GREY, Color.CYAN, Color.TEAL]
