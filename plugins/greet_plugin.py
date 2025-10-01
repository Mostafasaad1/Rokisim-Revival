#TODO ADD dynamic import Later
from src.rokisim_revival.plugin import Plugin

class GreetPlugin(Plugin):
    name = "Greet"
    description = "Returns a greeting message."
    version = "1.0.0"
    author = "None"

    def execute(self, name: str = "User") -> str:
        return f"Hello, {name}!"