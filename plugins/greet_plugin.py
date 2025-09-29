# plugins/greet_plugin.py
from plugin import Plugin

class GreetPlugin(Plugin):
    name = "Greet"
    description = "Returns a greeting message."
    version = "1.0.0"
    author = "RokiSim Team"

    def execute(self, name: str = "User") -> str:
        return f"Hello, {name}!"