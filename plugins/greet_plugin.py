from src.rokisim_revival.plugin import Plugin

print("test")
class GreetPlugin(Plugin):
    name = "Greet"
    description = "Returns a greeting message."
    version = "1.0.0"
    author = "RokiSim Team"

    def execute(self, name: str = "User") -> str:
        return f"Hello, {name}!"