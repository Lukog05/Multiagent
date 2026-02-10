from enum import Enum, unique


@unique
class Color(Enum):
    Blue = 0
    Red = 1
    Cyan = 2
    Purple = 3
    Green = 4
    Orange = 5
    Pink = 6
    Grey = 7
    Lightblue = 8
    Brown = 9

    @staticmethod #belongs to class. Cannot be accessed with self or cls (which are instances of the class).
    def from_string(s: str) -> "Color | None":
        return _STR_TO_COLOR.get(s.lower())


_STR_TO_COLOR = {
    "blue": Color.Blue,
    "red": Color.Red,
    "cyan": Color.Cyan,
    "purple": Color.Purple,
    "green": Color.Green,
    "orange": Color.Orange,
    "pink": Color.Pink,
    "grey": Color.Grey,
    "lightblue": Color.Lightblue,
    "brown": Color.Brown,
}

#colors are reprsented as enum
"""
agent_color = Color.Blue  # Store as enum member
box_color = Color.Red
"""
#__STR_TO_COLOR is defined outside of class. It is defined at module level. Pythonn executes class definiton first and then module code. So for instance, color.Blue exists when dictionary is created.
"""
Example usage:
# Parsing level file:
color_line = "blue: 0, A"  # From #colors section

# Extract color name
color_name = "blue"

# Convert to enum
color_enum = Color.from_string(color_name)
# → Color.Blue

# Now assign to agent/box
agent.color = color_enum  # Store as Color.Blue
box.color = color_enum
"""