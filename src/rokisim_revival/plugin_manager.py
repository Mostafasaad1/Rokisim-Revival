# plugin_manager.py
import importlib.util
import os
import sys
import logging
import traceback
from typing import Dict, Optional, Any, List
from pathlib import Path
from multiprocessing import Process, Queue
from .plugin import Plugin

logger = logging.getLogger(__name__)
PLUGIN_DIR = Path("plugins").resolve()
print(PLUGIN_DIR)
class PluginManager:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self.plugins: Dict[str, Plugin] = {}
        self.disabled_plugins: set = set()
        self._initialized = True
        self.load_plugins()

    def discover_plugin_files(self) -> List[Path]:
        if not PLUGIN_DIR.exists():
            PLUGIN_DIR.mkdir(parents=True, exist_ok=True)
        return [f for f in PLUGIN_DIR.glob("*.py") if f.name != "__init__.py"]

    def load_plugins(self) -> None:
        for plugin_file in self.discover_plugin_files():
            try:
                module_name = plugin_file.stem
                spec = importlib.util.spec_from_file_location(module_name, plugin_file)
                if spec is None or spec.loader is None:
                    continue
                module = importlib.util.module_from_spec(spec)
                sys.modules[module_name] = module
                spec.loader.exec_module(module)

                for attr_name in dir(module):
                    attr = getattr(module, attr_name)
                    if (
                        isinstance(attr, type)
                        and issubclass(attr, Plugin)
                        and attr is not Plugin
                    ):
                        plugin_instance = attr()
                        if plugin_instance.name in self.plugins:
                            logger.warning(f"Plugin name conflict: {plugin_instance.name}")
                            continue
                        self.plugins[plugin_instance.name] = plugin_instance
                        logger.info(f"Loaded plugin: {plugin_instance.name}")
            except Exception as e:
                logger.error(f"Failed to load plugin from {plugin_file}: {e}")

    def list_plugins(self) -> Dict[str, Dict]:
        return {
            name: plugin.get_metadata()
            for name, plugin in self.plugins.items()
            if name not in self.disabled_plugins
        }

    def enable_plugin(self, name: str) -> bool:
        if name in self.disabled_plugins:
            self.disabled_plugins.remove(name)
            return True
        return False

    def disable_plugin(self, name: str) -> bool:
        if name in self.plugins:
            self.disabled_plugins.add(name)
            return True
        return False

    def get_plugin(self, name: str) -> Optional[Plugin]:
        if name in self.disabled_plugins:
            return None
        return self.plugins.get(name)

    def _execute_in_sandbox(self, plugin_name: str, args: tuple, kwargs: dict, result_queue: Queue):
        try:
            manager = PluginManager()
            plugin = manager.get_plugin(plugin_name)
            if plugin is None:
                result_queue.put({"error": f"Plugin '{plugin_name}' not found or disabled."})
                return
            if not plugin.validate():
                result_queue.put({"error": f"Plugin '{plugin_name}' failed validation."})
                return
            result = plugin.execute(*args, **kwargs)
            result_queue.put({"result": result})
        except Exception as e:
            result_queue.put({"error": str(e), "traceback": traceback.format_exc()})

    def execute_plugin(self, name: str, *args, **kwargs) -> Any:
        if name not in self.plugins:
            raise ValueError(f"Plugin '{name}' not found.")
        if name in self.disabled_plugins:
            raise ValueError(f"Plugin '{name}' is disabled.")

        result_queue = Queue()
        process = Process(target=self._execute_in_sandbox, args=(name, args, kwargs, result_queue), daemon=True)
        process.start()
        process.join(timeout=10)

        if process.is_alive():
            process.terminate()
            process.join()
            raise RuntimeError(f"Plugin '{name}' timed out.")

        if result_queue.empty():
            raise RuntimeError(f"Plugin '{name}' returned no result.")

        output = result_queue.get()
        if "error" in output:
            logger.error(f"Plugin error: {output['error']}")
            raise RuntimeError(f"Plugin '{name}' failed: {output['error']}")

        return output["result"]